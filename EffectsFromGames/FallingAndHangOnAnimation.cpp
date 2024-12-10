#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

// #include <assimp/Importer.hpp>      // C++ importer interface
// #include <assimp/scene.h>           // Output data structure
// #include <assimp/postprocess.h>     // Post processing flags

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags


extern SimpleMath::Vector3 CollisionImpactNormal;
extern SimpleMath::Vector3 CollisionImpactPoint;
extern std::string CollisionObjectName;
extern int CollisionObjectType;

extern char* state_hanging_Hand_Name;
extern SimpleMath::Vector3 state_hanging_Hand_Location;

extern bool state_hanging;
extern std::string state_hanging_Ledge_Name;
extern Box state_hanging_Ledge_Box;
extern int state_hanging_Ledge_BoxIndex;

extern SimpleMath::Vector3 gravitation;

extern IAnimationGraph2 * EveAnimationGraph;

SimpleMath::Vector3 GetCharacterJointTranslation(CharacterSkelet * characterSkelet, int Index);

extern Character * Eve;

extern World GWorld;

extern IKSolverInterface * EveIKSolver;

extern JointsRefsChainCollection jointsRefsChains;

extern bool FallingAndHangOn_TargetToRightHandDefine;
extern SimpleMath::Vector3 FallingAndHangOn_TargetToRight;
extern SimpleMath::Vector3 FallingAndHangOn_01;
extern SimpleMath::Vector3 FallingAndHangOn_02;
extern SimpleMath::Vector3 FallingAndHangOn_03;

extern IClimbingPathHelper* ClimbingPathHelper;

void GetRelativeJoint(JointSQT & Joint1, JointSQT & Joint2);
SimpleMath::Quaternion CombineJointsRotations(JointSQT & Joint1, JointSQT & Joint2);

extern char DebugBuffer[1024];

void Debug();

SimpleMath::Vector3 vec4ToVec3(SimpleMath::Vector4 v4)
{
	return SimpleMath::Vector3(v4.x, v4.y, v4.z);
}

namespace Curve{
	SimpleMath::Vector3 quadratic(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, float t)
	{
		auto p0 = SimpleMath::Vector3::Lerp(a, b, t);
		auto p1 = SimpleMath::Vector3::Lerp(b, c, t);
		return SimpleMath::Vector3::Lerp(p0, p1, t);
	}
	SimpleMath::Vector3 bezier(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float t)
	{
		auto p0 = quadratic(a, b, c, t);
		auto p1 = quadratic(b, c, d, t);
		return SimpleMath::Vector3::Lerp(p0, p1, t);
	}
	SimpleMath::Vector3 bezierDerivative(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float t)
	{
		float tt = t*t;
		return
			a * (-3 + 6 * t - 3 * tt)+
			b * (3 - 12 * t + 9 * tt)+
			c * (0 +  6 * t - 9 * tt)+
			d * (0 +  0     + 3 * tt);
	}
	float arcLength(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float t)
	{
		static const float arg[] = { -0.7745966f, 0, 0.7745966f };
		static const float weight[] = { 0.5555556f, 0.8888889f, 0.5555556f };

		const float lower = 0.0f;
		const float upper = t;
		const float half_upper_lower = 0.5f*(upper - lower);

		float sum = 0.0f;

		for (int i = 0; i < 3; i++)
		{
			sum += weight[i] * bezierDerivative(a, b, c, d, lower + half_upper_lower * (arg[i] + 1)).Length();
		}

		return sum * half_upper_lower;
	}
	float getTByS(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float s)
	{
		float t = s / arcLength(a, b, c, d, 1.0f);
		float lower = 0.0f;
		float upper = 1.0f;

		for (int i = 0; i < 100; i++)
		{
			float f = arcLength(a, b, c, d, t) - s;

			if (fabs(f) < 0.01f)
			{
				//char buffer[1024]; sprintf(buffer, "getTByS ---- %d \n", i); OutputDebugStringA(buffer);
				return t;
			}

			float new_t =  t - f / bezierDerivative(a, b, c, d, t).Length();

			if (f > 0.0f)
			{
				upper = t;
				if (new_t <= 0.0f)
					t = 0.5f*(upper + lower);
				else
					t = new_t;
			}
			else
			{
				lower = t;
				if (new_t >= 1.0f)
					t = 0.5f*(upper + lower);
				else
					t = new_t;
			}
		}
	}
}

extern Character* Eve;
Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);

struct FallingAndHangOnAnimation : public Animation
{
	SimpleMath::Vector3 bX;
	SimpleMath::Vector3 bY;

	SimpleMath::Vector3 A;
	SimpleMath::Vector3 B;
	SimpleMath::Vector3 C;
	SimpleMath::Vector3 D;
	
	SimpleMath::Vector3 PathPrevLoc;

	AnimationRep* Impl;

	Animation* FirstPose;
	Animation* SecondPose;
	Animation* ThirdPose;

	/// Pose Sequence
	std::vector<JointSQT>* Poses[10];
	double PosesTimes[10];

	double PoseTime;

	int PoseIndex;
	std::vector<JointSQT>* PreviousPoseJoints;
	std::vector<JointSQT>* NextPoseJoints;
	double PoseMaxTime;
	/// Pose Sequence

	std::vector<JointSQT> StartPoseJoints;
	std::vector<JointSQT> StartRightHandJoints;
	std::vector<JointSQT> StartLeftHandJoints;

	bool Transaction;
	float TimeOffset;
	SimpleMath::Vector3 Origin;

	float Speed;
	
	bool PrevIntoTargetBox;

	float ElapsedTime;

	SimpleMath::Matrix modelTransform;

	//RightHand Data//////
	bool RightHandTargetDefined;
	SimpleMath::Vector3 RightHandTarget;
	bool RightHandTargetCatched;
	SimpleMath::Vector3 RightArmJointPosition;
	SimpleMath::Vector3 RightHandJointPosition;
	SimpleMath::Vector3 LeftHandJointPosition;
	SimpleMath::Vector3 RightHandJointWorldPosition;
	//////////////////////

	//LeftHand Data///////
	bool LeftHandTargetDefined;
	SimpleMath::Vector3 LeftHandTarget;
	//////////////////////

	std::vector<JointSQT> ChainsJoints[5];

	float HipsX, HipsZ;

	//////////////////////
	float PendulumAlfa;
	float PendulumAlfa2;
	float PendulumAlfa3;
	float PendulumMinAlfa;
	float PendulumMaxAlfa;
	float PendulumSign;
	float PendulumSign2;
	float PendulumSign3;
	bool PendulumActivated;
	bool PendulumActivatedForLegs;
	JointSQT PendulumRootFrame;
	SimpleMath::Vector4 PrevRootLocation;
	//////////////////////
	float PendulumPrimaryInitAlfa;
	float PendulumRightArmInitAlfa;
	float PendulumLegsInitAlfa;

	float PendulumPrimaryTimeOffset;
	float PendulumRightArmTimeOffset;
	float PendulumLegsTimeOffset;

	float PendulumPrimaryAlfa[10000];
	float PendulumRightArmAlfa[10000];
	float PendulumLegsAlfa[10000];

	float g;
	float L;
	float mu;
	float Theta;
	float delta_t;
	float ThetaDot;

	float RightDt;
	float LeftDt;

	bool InitAdjustOrientationToEdge;

	//////////////////////
	SimpleMath::Quaternion PrevRootRotation;
	SimpleMath::Vector3 RotationAxis;
	//////////////////////

	//////////////////////
	std::function<void __cdecl(bool state)> onPlayingChanged;
	//////////////////////

	//////////////////////
	std::map<void*, float> pendulumTimeToStop;
	//////////////////////

	bool IsAllPendulumesNearStop()
	{
		if (IsPendulumeTargetTimeAchieved(PendulumPrimaryAlfa, PendulumPrimaryTimeOffset, 1.0f))
			if (IsPendulumeTargetTimeAchieved(PendulumRightArmAlfa, PendulumRightArmTimeOffset, 1.0f))
				if (IsPendulumeTargetTimeAchieved(PendulumLegsAlfa, PendulumLegsTimeOffset, 1.0f))
					return true;
		return false;
	}

	float PendulumAccelValue()
	{
		return -mu*ThetaDot - (g / L)*sin(Theta);
	}

	void ComputePendulumAlfa(float aTheta, float aThetaDot, float aMu, float aL, float* PendulumAlfa)
	{
		L = aL;
		mu = aMu;
		Theta = aTheta;
		ThetaDot = aThetaDot;
		for (int i = 0; i < 10000; i++)
		{
			float ThetaDoubleDot = PendulumAccelValue();
			Theta += ThetaDot*delta_t;
			ThetaDot += ThetaDoubleDot*delta_t;
			
			if (fabs(ThetaDoubleDot) < 0.0001 && pendulumTimeToStop[PendulumAlfa] == 0.0f)
			{
				pendulumTimeToStop[PendulumAlfa] = i;
			}

			PendulumAlfa[i] = Theta;
		}
	}

	float IsPendulumeTargetTimeAchieved(float* PendulumAlfa, float offset, float normalizedTime)
	{
		float time = Impl->prev_local_time - offset;
		float k = time / delta_t;
		return k > normalizedTime*pendulumTimeToStop[PendulumAlfa];
	}

	float GetPendulumeAlfa(float* PendulumAlfa, float offset)
	{
		float time = Impl->prev_local_time - offset;

		float k = time / delta_t;
		int i = int(k);
		k = k - i;

		if (i < 9999)
		{
			return PendulumAlfa[i] + k*(PendulumAlfa[i + 1] - PendulumAlfa[i]);
		}
		else
		{
			return 0.0f;
		}
	}

	float GetPendulumTime(float* PendulumAlfa, float alfa)
	{
		for (int i = 0; i < 10000; i++)
		{
			if (fabs(PendulumAlfa[i]) < alfa)
			{
				return i*delta_t;
			}
		}
		return 0.0f;
	}

	//////////////////////
	FallingAndHangOnAnimation(std::map<std::string, unsigned int> & FramesNamesIndex)
	{
		////configuration
		g = 9.8f;
		Speed = 5.f;
		delta_t = 0.01f;
		ComputePendulumAlfa((PendulumLegsInitAlfa = 15.0f)*(3.14f / 180.f), 0.0f, 0.1f, 4, PendulumLegsAlfa);
		ComputePendulumAlfa((PendulumRightArmInitAlfa = 10.f)*(3.14f / 180.f), 0.0f, 0.1f, 4, PendulumRightArmAlfa);
		////configuration

		Impl = new AnimationRep();

		Impl->looped = false;
		Impl->Rate = 1.0f;

		Transaction = false;

		FirstPose = loadAnimationFromBlender("Media\\Animations\\FallingAndHangOn_01.dae", FramesNamesIndex);

		SecondPose = loadAnimationFromBlender("Media\\Animations\\FallingAndHangOn_02.dae", FramesNamesIndex);

		ThirdPose = loadAnimationFromBlender("Media\\Animations\\FallingAndHangOn_03.dae", FramesNamesIndex);

		Poses[0] = &StartPoseJoints;
		Poses[1] = &FirstPose->CurrentJoints;
		Poses[2] = &SecondPose->CurrentJoints;
		Poses[3] = &ThirdPose->CurrentJoints;
		Poses[4] = nullptr;

		(*Poses[3])[64][1] = SimpleMath::Quaternion::CreateFromYawPitchRoll(-180.f*(acos(-1.0f) / 180.f), -36.613449f*(acos(-1.0f) / 180.f), -0.814534f*(acos(-1.0f) / 180.f));

		PosesTimes[0] = 1;
		PosesTimes[1] = 0.33;
		PosesTimes[2] = 0.33;

		StartRightHandJoints.resize(32);
		StartLeftHandJoints.resize(32);

		for (int i = 0; i < jointsRefsChains.size(); i++)
		{
			ChainsJoints[i].resize(32);
		}

		onPlayingChanged = nullptr;

		pendulumTimeToStop.insert(std::map<void*, float>::value_type(PendulumPrimaryAlfa, 0));
		pendulumTimeToStop.insert(std::map<void*, float>::value_type(PendulumRightArmAlfa, 0));
		pendulumTimeToStop.insert(std::map<void*, float>::value_type(PendulumLegsAlfa, 0));
	}
	~FallingAndHangOnAnimation()
	{
		delete Impl;
		delete FirstPose;
		delete SecondPose;
		delete ThirdPose;
	};
	//////////////////////////////////////////////
	int LoadChainForSolver(int index, int size)
	{
		int StartIndex = jointsRefsChains[index].size() - size;
		for (int i = 0; i < size; i++)
		{
			EveIKSolver->chainRef(0)[i] = ChainsJoints[index][StartIndex + i];
		}
		return size;
	}
	void SolvedChainToLocal(int index, int size)
	{
		JointHelpers::modelToLocal(size, EveIKSolver->chainRef(0));
		GetRelativeJoint(ChainsJoints[index][jointsRefsChains[index].size() - size - 1], EveIKSolver->chainRef(0)[0]);
	}
	//////////////////////////////////////////////
	bool ComputeRightHandTarget()
	{
		bool Result;

		SimpleMath::Vector3 RightArmPos = (ChainsJoints[0][jointsRefsChains[0].size() - 3].matrix() * modelTransform).Translation();
		SimpleMath::Vector3 RightShoulderPos = (ChainsJoints[0][jointsRefsChains[0].size() - 4].matrix() * modelTransform).Translation();
		SimpleMath::Vector3 Spine2Pos = (ChainsJoints[0][jointsRefsChains[0].size() - 5].matrix() * modelTransform).Translation();

		//auto N = SimpleMath::Vector3(RightArmPos - Spine2Pos).Cross(RightShoulderPos - Spine2Pos);
		//auto L = vec4(N, -Spine2Pos.Dot(N));

		auto M = GWorld.Capsules["eve"].getMatrix();
		auto L = vec4(M.Backward(), -M.Backward().Dot(M.Translation() + .0f * M.Backward()));

		auto S = state_hanging_Ledge_Box.origin;
		auto V = -state_hanging_Ledge_Box.worldForward;

		auto t = -(L.Dot(vec4(S, 1)) / L.Dot(vec4(V, 0)));

		RightHandTargetDefined = Result = -0.5f < t && t < 0.5f;

		RightHandTarget = S + t*V;

		return Result;
	};
	void SaveRightHandJoint()
	{
		JointHelpers::AnimToChain(jointsRefsChains.RightArmRightHand, this, StartRightHandJoints);
	}
	void SolveRightHandToTarget()
	{
		if (RightHandTargetCatched)
		{
			EveIKSolver->solve(LoadChainForSolver(0, 3), RightHandJointPosition);
		}
		else
		{
			EveIKSolver->solve(LoadChainForSolver(0, 3), SimpleMath::Vector3::Transform(RightHandTarget, modelTransform.Invert()));
		}

		SolvedChainToLocal(0, 3);
	};
	////
	bool ComputeLeftHandTarget()
	{
		//auto M = GWorld.Capsules["eve"].getMatrix();
		//
		//auto L = vec4(M.Backward(), -M.Backward().Dot(M.Translation() - 1.0f * M.Backward()));
		//
		//auto S = state_hanging_Ledge_Box.origin;
		//auto V = -state_hanging_Ledge_Box.worldForward;
		//
		//auto t = -(L.Dot(vec4(S, 1)) / L.Dot(vec4(V, 0)));
		//
		//LeftHandTargetDefined = true;
		//
		//LeftHandTarget = S + t*V;
		
		LeftHandTargetDefined = true;

		auto TargetBackSide = state_hanging_Ledge_Box.worldForward; //SimpleMath::Vector3(0, 0, -1);
		TargetBackSide.Normalize();

		LeftHandJointPosition = RightHandJointPosition + SimpleMath::Vector3::TransformNormal(1.1f * TargetBackSide, modelTransform.Invert());

		LeftHandTarget = SimpleMath::Vector3::Transform(LeftHandJointPosition, modelTransform);

		//FallingAndHangOn_TargetToRightHandDefine = LeftHandTargetDefined;

		//FallingAndHangOn_TargetToRight = LeftHandTarget;

		return true;
	}
	void SaveLeftHandJoint()
	{
		JointHelpers::AnimToChain(jointsRefsChains.LeftArmLeftHand, this, StartLeftHandJoints);
	}
	void SolveLeftHandToTarget()
	{
		EveIKSolver->solve(LoadChainForSolver(1, 3), LeftHandJointPosition);

		SolvedChainToLocal(1, 3);
	};
	//////////////////////////////////////////////
	void LerpCurrentToSolvedHand(std::vector<int> & chain, float lerpParameter, std::vector<JointSQT> & StartJoints)
	{
		auto chain_size = chain.size();

		lerpParameter = lerpParameter > 1.0f ? 1.0f : lerpParameter;

		for (int i = 0; i < chain_size; i++)
		{
			auto & joint1 = StartJoints[i];
			auto & joint2 = EveIKSolver->chainRef(0)[i];
			auto & joint = CurrentJoints[chain[i]];

			joint[0] = SimpleMath::Vector4::Lerp(joint1[0], joint2[0], lerpParameter);
			joint[1] = SimpleMath::Quaternion::Lerp(joint1[1], joint2[1], lerpParameter);
			joint[2] = SimpleMath::Vector4::Lerp(joint1[2], joint2[2], lerpParameter);
		}
	};
	void LerpPoses(float lerpParameter)
	{
		auto size = CurrentJoints.size();

		lerpParameter = lerpParameter > 1.0f ? 1.0f : lerpParameter;

		for (int i = 0; i < size; i++)
		{
			if ((RightHandTargetCatched || RightHandTargetDefined) && IsJointBelongsHand(i, jointsRefsChains.RightArmRightHand))
				continue;

			if (LeftHandTargetDefined && IsJointBelongsHand(i, jointsRefsChains.LeftArmLeftHand))
				continue;

			auto & joint1 = (*PreviousPoseJoints)[i];
			auto & joint2 = (*NextPoseJoints)[i];
			auto & joint = CurrentJoints[i];

			joint[0] = SimpleMath::Vector4::Lerp(joint1[0], joint2[0], lerpParameter);
			joint[1] = SimpleMath::Quaternion::Lerp(joint1[1], joint2[1], lerpParameter);
			if (i != 64)
			{
				joint[2] = SimpleMath::Vector4::Lerp(joint1[2], joint2[2], lerpParameter);
			}
		}

		//if (PoseIndex == 0)
		//{
			//CurrentJoints[64][2] = (*Poses[0])[64][2];// (*PreviousPoseJoints)[64][2];
			//CurrentJoints[64][2] = (*PreviousPoseJoints)[64][2];
		//}
	}
	void SetupBlendPoses()
	{
		PreviousPoseJoints = Poses[PoseIndex];
		NextPoseJoints = Poses[PoseIndex + 1];
			
		PoseMaxTime = PosesTimes[PoseIndex];
	}
	//////////////////////////////////////////////
	bool IsJointBelongsHand(int index, std::vector<int> & chain)
	{
		auto chain_size = chain.size();

		for (int i = 0; i < chain_size; i++)
		{
			if (index == chain[i])
				return true;
		}

		return false;
	}
	void ChainHipsToRightHandFromLocalToModelSpace()
	{
		JointHelpers::AnimToChain(jointsRefsChains[0], this, ChainsJoints[0]);

		JointHelpers::localToModel(jointsRefsChains[0].size(), ChainsJoints[0]);
	}
	void ChainHipsToLeftHandFromLocalToModelSpace()
	{
		JointHelpers::AnimToChain(jointsRefsChains[1], this, ChainsJoints[1]);

		JointHelpers::localToModel(jointsRefsChains[1].size(), ChainsJoints[1]);
	}
	void ChainHipsToRightToeEndFromLocalToModelSpace()
	{
		JointHelpers::AnimToChain(jointsRefsChains[3], this, ChainsJoints[3]);

		JointHelpers::localToModel(jointsRefsChains[3].size(), ChainsJoints[3]);
	}
	void ChainHipsToLeftToeEndFromLocalToModelSpace()
	{
		JointHelpers::AnimToChain(jointsRefsChains[4], this, ChainsJoints[4]);

		JointHelpers::localToModel(jointsRefsChains[4].size(), ChainsJoints[4]);
	}
	bool IsHandIntoBox()
	{
		auto box = state_hanging_Ledge_Box;
		box.size -= SimpleMath::Vector3(0, 0.1f, 0.1f);
		box.evalWorldSize();
		return box.Containe(SimpleMath::Vector3::Transform(GetRightHandJointLocation(), modelTransform));
	}
	SimpleMath::Vector3 GetRightArmJointLocation()
	{
		int RightHandIndex = 0;

		return ChainsJoints[RightHandIndex][jointsRefsChains[RightHandIndex].size() - 3].matrix().Translation();
	}
	SimpleMath::Vector3 GetRightHandJointLocation()
	{
		int RightHandIndex = 0;

		return ChainsJoints[RightHandIndex][jointsRefsChains[RightHandIndex].size() - 1].matrix().Translation();
	}
	SimpleMath::Vector3 GetRootJointLocation()
	{
		int RightHandIndex = 0;

		return ChainsJoints[RightHandIndex][0].matrix().Translation();
	}
	JointSQT GetRootJoint()
	{
		int RightHandIndex = 0;

		return ChainsJoints[RightHandIndex][0];
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void ComputePendulumAlfa()
	{
		auto AB = GetRightHandJointLocation() - GetRightArmJointLocation();

		PendulumPrimaryInitAlfa = atan2(abs(AB.z), abs(AB.y));

		PendulumMinAlfa = -atan2(abs(AB.z), abs(AB.y)) + -0.8f*atan2(abs(AB.z), abs(AB.y));
		PendulumMaxAlfa = 0.0f;

		PendulumAlfa = 0.0f;

		PendulumAlfa2 = 0.0f;

		PendulumAlfa3 = 0.0f;

		PendulumSign2 = 1.0f;

		PendulumSign3 = 1.0f;

		ComputePendulumAlfa(PendulumPrimaryInitAlfa, -.5f, 0.5f, 4, PendulumPrimaryAlfa);
	}
	void RotateAroundRightHand()
	{
		/*
		if (PendulumAlfa <= PendulumMinAlfa)
		{
			PendulumSign = 1.0f;
		}
		else if (PendulumMaxAlfa <= PendulumAlfa)
		{
			PendulumSign = -1.0f;
		}
		if (ElapsedTime != 0.0f)
		{
			PendulumAlfa += PendulumSign * 15.0f * 0.064f * (3.14f / 180.f);
		}
		*/
		PendulumAlfa = GetPendulumeAlfa(PendulumPrimaryAlfa, PendulumPrimaryTimeOffset);

		auto RotToOrigin = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, -PendulumPrimaryInitAlfa);

		auto Rot = SimpleMath::Quaternion::Concatenate(SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, PendulumAlfa), RotToOrigin);
		auto Loc = SimpleMath::Vector3::Transform(vec4ToVec3(CurrentJoints[64][2]) - RightHandJointPosition, Rot) + RightHandJointPosition;

		PendulumRootFrame[0] = SimpleMath::Vector4(1, 1, 1, 1);
		PendulumRootFrame[1] = SimpleMath::Quaternion::Concatenate(Rot, CurrentJoints[64][1]);
		PendulumRootFrame[2] = SimpleMath::Vector4(Loc.x, Loc.y, Loc.z, 1);

		CurrentJoints[64] = PendulumRootFrame;
	}
	void RotateAroundRightArm()
	{
		/*
		if (PendulumAlfa2 <= -RotateRightArmAlfa*(3.14f / 180.f))
		{
			PendulumSign2 = 1.0f;
		}
		else if (RotateRightArmAlfa*(3.14f / 180.f) <= PendulumAlfa2)
		{
			PendulumSign2 = -1.0f;
		}
		if (ElapsedTime != 0.0f)
		{
			PendulumAlfa2 += PendulumSign2 * 7.5f * 0.064f * (3.14f / 180.f);
		}
		*/
		PendulumAlfa2 = GetPendulumeAlfa(PendulumRightArmAlfa, PendulumRightArmTimeOffset);

		auto TargetRot = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, PendulumAlfa2);

		{
			int RightHandIndex = 0;

			int LastIndex = jointsRefsChains[RightHandIndex].size() - 1;

			auto RightArmJointLocation = ChainsJoints[RightHandIndex][LastIndex - 2].matrix().Translation();

			for (int i = LastIndex - 4; i >= 0; i--)
			{
				auto & ModelJoint = ChainsJoints[RightHandIndex][i];

				auto Loc = SimpleMath::Vector3::Transform(vec4ToVec3(ModelJoint[2]) - RightArmJointLocation, TargetRot) + RightArmJointLocation;
				auto Rot = SimpleMath::Quaternion::Concatenate(TargetRot, ModelJoint[1]);

				ModelJoint[1] = Rot;
				ModelJoint[2] = vec4(Loc, 1);
			}

			auto Local = ChainsJoints[RightHandIndex];

			JointHelpers::modelToLocal(jointsRefsChains[RightHandIndex].size(), Local);

			JointHelpers::ChainToAnim(jointsRefsChains[RightHandIndex], Local, this);
		}
	}
	void RotateLegs()
	{
		/*
		if (PendulumAlfa3 <= -RotateLegsAlfa*(3.14f / 180.f))
		{
			PendulumSign3 = 1.0f;
		}
		else if (RotateLegsAlfa*(3.14f / 180.f) <= PendulumAlfa3)
		{
			PendulumSign3 = -1.0f;
		}
		if (ElapsedTime != 0.0f)
		{
			PendulumAlfa3 += PendulumSign3 * 15.0f * 0.064f * (3.14f / 180.f);
		}
		*/
		PendulumAlfa3 = GetPendulumeAlfa(PendulumLegsAlfa, PendulumLegsTimeOffset);

		auto TargetRot = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, PendulumAlfa3);

		for (int j = 3; j < 5; j++)
		{
			int LegIndex = j;

			int LastIndex = jointsRefsChains[LegIndex].size() - 1;

			auto UpperLegLocation = ChainsJoints[LegIndex][1].matrix().Translation();

			for (int i = 1; i <= LastIndex; i++)
			{
				auto & ModelJoint = ChainsJoints[LegIndex][i];

				auto Loc = SimpleMath::Vector3::Transform(vec4ToVec3(ModelJoint[2]) - UpperLegLocation, TargetRot) + UpperLegLocation;
				auto Rot = SimpleMath::Quaternion::Concatenate(TargetRot, ModelJoint[1]);

				ModelJoint[1] = Rot;
				ModelJoint[2] = vec4(Loc, 1);
			}

			JointHelpers::modelToLocal(jointsRefsChains[LegIndex].size(), ChainsJoints[LegIndex]);

			JointHelpers::ChainToAnim(jointsRefsChains[LegIndex], ChainsJoints[LegIndex], this);
		}
	}
	//////////////////////////////////////////////
	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		ElapsedTime = elapsedTime;

		Impl->prev_frameNo = Impl->frameNo;
		Impl->prev_local_time = Impl->local_time;

		DeltaTranslation = SimpleMath::Vector3::Zero;
		DeltaRotation = SimpleMath::Quaternion::Identity;
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if (!RightHandTargetCatched)
		{
			modelTransform = SimpleMath::Matrix::CreateScale(0.01f) * SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		LerpPoses(PoseTime / PoseMaxTime);

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Correction capsule offset

		if (RightHandTargetCatched)
		{
			ChainHipsToRightHandFromLocalToModelSpace();

			auto Offset = RightArmJointPosition - GetRightArmJointLocation(); //RightHandJointPosition - GetRightHandJointLocation();

			CurrentJoints[64][2] = CurrentJoints[64][2] + vec4(Offset, 0.f);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if (RightHandTargetDefined || RightHandTargetCatched)
		{
			ChainHipsToRightHandFromLocalToModelSpace();

			SolveRightHandToTarget();

			LerpCurrentToSolvedHand(jointsRefsChains.RightArmRightHand, (Impl->prev_local_time - RightDt) / 0.5f, StartRightHandJoints);
		}

		if (RightHandTargetCatched)
		{
			bool FallingAndHangAdjustOrientationToEdge(bool& Init, float DeltaTime, const SimpleMath::Vector3 & ForwardVector, SimpleMath::Vector3 TargetForwardVector, SimpleMath::Vector3 HandMidLocation, JointSQT& RootJoint);

			auto Rotated180Forward = -1.f*GWorld.Capsules["eve"].getMatrix().Right();

			auto TargetForward = -state_hanging_Ledge_Box.worldBackSide; //SimpleMath::Vector3(0, 0, -1);
			TargetForward.Normalize();

			FallingAndHangAdjustOrientationToEdge(InitAdjustOrientationToEdge, elapsedTime, Rotated180Forward, TargetForward, 0.5f*(LeftHandJointPosition + RightHandJointPosition), CurrentJoints[64]);
				
			ChainHipsToRightHandFromLocalToModelSpace();

			auto Offset = RightHandJointPosition - GetRightHandJointLocation();

			CurrentJoints[64][2] = CurrentJoints[64][2] + vec4(Offset, 0.f);
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Oscilation

		if (PendulumActivated)
		{
			ChainHipsToRightHandFromLocalToModelSpace();

			RotateAroundRightHand();

			{
				ChainHipsToRightHandFromLocalToModelSpace();

				RotateAroundRightArm();
			}
		}
		if (PendulumActivatedForLegs)
		{
			ChainHipsToRightToeEndFromLocalToModelSpace();

			ChainHipsToLeftToeEndFromLocalToModelSpace();

			RotateLegs();
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Left And Right Hand To Target If Target Define

		if (LeftHandTargetDefined)
		{
			ChainHipsToLeftHandFromLocalToModelSpace();

			SolveLeftHandToTarget();

			LerpCurrentToSolvedHand(jointsRefsChains.LeftArmLeftHand, (Impl->prev_local_time - LeftDt) / 0.5f, StartLeftHandJoints);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//States Changer

		if (!RightHandTargetDefined)
		{
			if ((PoseTime / PoseMaxTime) > 0.5f)
			{
				SaveRightHandJoint();

				ComputeRightHandTarget();

				RightDt = Impl->local_time + elapsedTime;
			}
		}

		if (!RightHandTargetCatched)
		{
			ChainHipsToRightHandFromLocalToModelSpace();

			bool NextIntoTargetBox = IsHandIntoBox();

			if (PrevIntoTargetBox && !NextIntoTargetBox)
			{
				RightHandTargetDefined = false;

				RightHandTargetCatched = true;

				/////////////////////////////////////////////////////////////////////////////////////////////////////
				RightArmJointPosition = GetRightArmJointLocation();

				RightHandJointPosition = GetRightHandJointLocation();

				Simulation::AddLedge(state_hanging_Ledge_Name, state_hanging_Ledge_Box, state_hanging_Ledge_BoxIndex);
				state_hanging_Hand_Name = Hands[1];
				state_hanging_Hand_Location = RightHandJointWorldPosition = SimpleMath::Vector3::Transform(RightHandJointPosition, modelTransform);

				sprintf(DebugBuffer, "RotationAxis %f %f %f\n", state_hanging_Ledge_Box.worldForward.x, state_hanging_Ledge_Box.worldForward.y, state_hanging_Ledge_Box.worldForward.z); Debug();
				RotationAxis = SimpleMath::Vector3::TransformNormal(-state_hanging_Ledge_Box.worldForward, modelTransform.Invert());
				RotationAxis.Normalize();

				PrevRootRotation = CurrentJoints[64][1];
			}

			PrevIntoTargetBox = NextIntoTargetBox;
		}
		
		if (!LeftHandTargetDefined)
		{
			if (RightHandTargetCatched && PoseIndex > 0 && (PoseTime / PoseMaxTime) > 0.5f)
			{
				SaveLeftHandJoint();

				ComputeLeftHandTarget();

				LeftDt = Impl->local_time + elapsedTime;
			}
		}

		if (!PendulumActivated)
		{
			if (RightHandTargetCatched)
			{
				PendulumActivated = true;

				PendulumPrimaryTimeOffset = Impl->local_time + elapsedTime;

				PendulumRightArmTimeOffset = Impl->local_time + elapsedTime - GetPendulumTime(PendulumRightArmAlfa, 0.01f);

				/////////////////////////////////////////////////////////////////////////////////////////////////////
				ChainHipsToRightHandFromLocalToModelSpace();

				ComputePendulumAlfa();
				/////////////////////////////////////////////////////////////////////////////////////////////////////
			}
		}

		if (!PendulumActivatedForLegs)
		{
			if (PendulumActivated && PoseIndex == 2)
			{
				PendulumActivatedForLegs = true;

				PendulumLegsTimeOffset = Impl->local_time + elapsedTime - GetPendulumTime(PendulumLegsAlfa, 0.01f);
			}
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Compute Delta Translation
		{
			const auto Delta = SimpleMath::Vector3::TransformNormal(vec4ToVec3(CurrentJoints[64][2] - PrevRootLocation), modelTransform) + getDeltaTranslation();

			PrevRootLocation = CurrentJoints[64][2];

			DeltaTranslation = Delta;

			CurrentJoints[64][2] = (*Poses[0])[64][2];
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Compute Delta Rotation
		if (RightHandTargetCatched)
		{
			Quat Q;

			Q.decompose(0.0f, PrevRootRotation, CurrentJoints[64][1]);

			PrevRootRotation = CurrentJoints[64][1];

			DeltaRotation = Q.DeltaRotation;

			CurrentJoints[64][1] = Q.RootRotation;
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//Advanse Animation Time

		if (Impl->playing)
		{
			if (RightHandTargetCatched)
			{
				bool state = 0;

				if (IsPendulumeTargetTimeAchieved(PendulumPrimaryAlfa, PendulumPrimaryTimeOffset, .15f))
				{
					(char&)state = 1;
					if (onPlayingChanged)
						onPlayingChanged(state);
				}

				if (IsAllPendulumesNearStop())
				{
					(char&)state = 2;
					if (onPlayingChanged)
						onPlayingChanged(state);
				}
			}

			if (PoseTime < PoseMaxTime)
			{
				PoseTime = PoseTime + elapsedTime;
			}

			if (RightHandTargetCatched)
			{
				if (PoseIndex < 2 && PoseTime > PoseMaxTime)
				{
					PoseTime = PoseIndex==0 ? 0.0f : (PoseTime - PoseMaxTime);
					++PoseIndex;
					SetupBlendPoses();
				}
				else if (PoseIndex == 2 && PoseTime > PoseMaxTime)
				{
					auto TargetForward = -state_hanging_Ledge_Box.worldBackSide;// SimpleMath::Vector3(0, 0, -1);
					TargetForward.Normalize();
					auto Up = SimpleMath::Vector3(0, 1, 0);
					DeltaRotation = SimpleMath::Quaternion::Identity;
					GWorld.Capsules["eve"].orientation = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(TargetForward, Up, TargetForward.Cross(Up)));
				}
			}

			double local_time = Impl->local_time + elapsedTime;

			Impl->frameNo += 1;
			Impl->local_time = local_time;
		}
	}
	//////subscribe
	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
	{
		onPlayingChanged = callback;
	}
	/////////////////
	bool getPlaying()
	{
		return Impl->playing;
	}
	void setPlaying(bool value)
	{
		if (Impl->playing != value)
		{
			bool state = 0;
			Impl->playing = value;
			if (value)
				(char&)state = 3;
			else
				(char&)state = 4;
			if (onPlayingChanged)
				onPlayingChanged(state);
		}
	}
	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}
	void reset()
	{
		PendulumPrimaryTimeOffset = -1.f;
		PendulumRightArmTimeOffset = -1.f;
		PendulumLegsTimeOffset = -1.f;

		PendulumActivatedForLegs = PendulumActivated = false;

		StartPoseJoints = CurrentJoints;

		constructFrame(bX, bY);

		A = SimpleMath::Vector3();
		B = SimpleMath::Vector3(2.0f / 3.0f*2.0f + 0.62f, 0.0f, 0.0f);
		C = SimpleMath::Vector3(2.0f + 0.62f, 1.0f / 3.0f*(0.01*5.0*72.0), 0.0f);
		D = SimpleMath::Vector3(2.0f + 0.62f, (0.01*5.0*72.0), 0.0f);

		PathPrevLoc = SimpleMath::Vector3();

		Transaction = false;

		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->prev_local_time = Impl->local_time = .0;

		Impl->playing = false;

		PoseIndex = 0;
		PoseTime = 0.0;

		SetupBlendPoses();

		RightHandTargetCatched = RightHandTargetDefined = false;

		PrevIntoTargetBox = false;

		LeftHandTargetDefined = false;

		PrevRootLocation = (*Poses[0])[64][2];
		PrevRootRotation = (*Poses[0])[64][1];

		InitAdjustOrientationToEdge = false;
	}
	////////////////////////////////////////////////////////////////////////////
	void constructFrame(SimpleMath::Vector3& x, SimpleMath::Vector3& y)
	{
		//auto Offset = GWorld.Capsules["eve"].origin - CollisionImpactPoint;
		//x = Offset;
		//x.x = 0.0f;
		//x.y = 0.0f;

		x = state_hanging_Ledge_Box.worldBackSide;
		x.Normalize();
		y = SimpleMath::Vector3(0, -1, 0);
	}
	////////////////////////////////////////////////////////////////////////////
	SimpleMath::Vector3 getDeltaTranslation()
	{
		SimpleMath::Vector3 Delta = SimpleMath::Vector3::Zero;

		if (!RightHandTargetCatched)
		{
			SimpleMath::Vector3 PathNextLoc;

			float Distantce = Speed * Impl->prev_local_time;

			if (Distantce <= Curve::arcLength(A, B, C, D, 1.0f))
			{
				float Parameter = Curve::getTByS(A, B, C, D, Distantce);

				PathNextLoc = Curve::bezier(A, B, C, D, Parameter);
				PathNextLoc = PathNextLoc.x*bX + PathNextLoc.y*bY;
			}
			else
			{
				if (!Transaction)
				{
					TimeOffset = Impl->prev_local_time;
					Origin = Curve::bezier(A, B, C, D, 1.0f).x*bX + Curve::bezier(A, B, C, D, 1.0f).y*bY;
					Transaction = true;

					auto leftDistantce = Distantce - Curve::arcLength(A, B, C, D, 1.0f);
					PathNextLoc = leftDistantce * bY + Origin;
					Origin = PathNextLoc;
				}
				else
				{
					Distantce = Speed * (Impl->prev_local_time - TimeOffset);

					PathNextLoc = Distantce * bY + Origin;
				}
			}

			Delta = PathNextLoc - PathPrevLoc;

			PathPrevLoc = PathNextLoc;
		}

		//if (RightHandTargetCatched && ElapsedTime != 0.0f)
		//{
		//	auto offset = GetRightArmJointPositon();
		//	auto D = RightArmJointPosition - offset;
		//	RightArmJointPosition = offset;
			//Delta += SimpleMath::Vector3::TransformNormal(D, modelTransform);
		//}

		return Delta;
	}

	double getRate(){ return 0; };

	void setRate(double speed){ };

	int getFrame(){ return 0; };

	double getLocTime(){ return Impl->prev_local_time; };

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f){ };

	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f){ };

	void setLooping(bool loop){ };

	bool IsLoop(){
		return false;
	};
};

Animation * createFallingAndHangOnAnimation(std::map<std::string, unsigned int> & FramesNamesIndex)
{
	return new FallingAndHangOnAnimation(FramesNamesIndex);
}

void FallingAndHangOnAnimationGetRightHandWorldJointLocation(AnimationBase *animation, SimpleMath::Vector3& Location)
{
	auto FallingAndHangOn = ((FallingAndHangOnAnimation*)animation);

	Location = FallingAndHangOn->RightHandJointWorldPosition;
}

bool FallingAndHangAdjustOrientationToEdge(bool& Init, float DeltaTime, const SimpleMath::Vector3 & ForwardVector, SimpleMath::Vector3 TargetForwardVector, SimpleMath::Vector3 HandMidLocation, JointSQT& RootJoint)
{
	static float TotalTime = 0.f;

	static float CurAngle = 0;

	static SimpleMath::Vector3 RotationAxis;

	static float AnglePerTime = 0;

	if (!Init)
	{
		TotalTime = 0.f;

		CurAngle = 0.f;

		RotationAxis = ForwardVector.Cross(TargetForwardVector);

		if (XMVector3Equal(RotationAxis, XMVectorZero()))
		{
			AnglePerTime = 0.0f;
		}
		else
		{
			//планируем пройти этот угол за 0.33 секунды
			AnglePerTime = atan2(RotationAxis.Length(), ForwardVector.Dot(TargetForwardVector))/.33f;
		}

		Init = true;
	}

	{
		TotalTime = min(TotalTime + DeltaTime, .33f);

		CurAngle = AnglePerTime * TotalTime;

		auto Rot = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, CurAngle);
		//auto Loc = SimpleMath::Vector3::Transform(vec4ToVec3(RootJoint[1]) - HandMidLocation, Rot) + HandMidLocation;

		RootJoint[1] = SimpleMath::Quaternion::Concatenate(Rot, RootJoint[1]);
		//RootJoint[2] = SimpleMath::Vector4(Loc.x, Loc.y, Loc.z, 1);
	}

	return TotalTime >= .33f;
}
