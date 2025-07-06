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

std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);
Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);
SimpleMath::Vector3 GetEdgeHorizontalJumpAnimation_GetHandLocation(bool IsLeft);

void GetRelativeJoint(JointSQT & Joint1, JointSQT & Joint2);
void GetEdgeHorizontalJumpAnimation_RotateChain(std::vector<JointSQT>& ChainJoints, SimpleMath::Quaternion DeltaRot, int ChainJoints_OriginIndex, int ChainJoints_Count);

extern IKSolverInterface * EveIKSolver;
extern JointsRefsChainCollection jointsRefsChains;
extern World GWorld;
void GetRelativeJoint(JointSQT & Joint1, JointSQT & Joint2);
extern IAnimationGraph2 * EveAnimationGraph;
extern Character * Eve;

extern EvePath g_EvePath;

namespace
{
	SimpleMath::Vector3 normalize(SimpleMath::Vector3 v)
	{
		v.Normalize();

		return v;
	}
}

#undef min // use __min instead
#undef max // use __max instead

struct BracedHangHopUpAnimation : public Animation
{
	AnimationRep* Impl;
	std::function<void __cdecl(bool state)> onPlayingChanged;
	Animation* CurrentPose;
	
	bool LeftHand_Solved;
	float savedElapsedTime, handUpTotalTime;
	SimpleMath::Vector3 RootVelocity, HandVelocity;
	SimpleMath::Vector3 Initial_LeftHand, Initial_RightHand;
	SimpleMath::Vector3 Initial_HandUpLeftHand;

	TDeltaRotation2 SpinRotation, ArmRotation;

	BracedHangHopUpAnimation()
	{
		Impl = new AnimationRep();
	}

	~BracedHangHopUpAnimation()
	{
		delete CurrentPose;

		delete Impl;
	}

	SimpleMath::Vector3 HandIK(int Index, AnimationBase * Anim, std::vector<int>& jointsRefs, int Size, SimpleMath::Vector3 Target, int total_iter = 10, float epsilon = 0.001f)
	{
		JointHelpers::AnimToChain(jointsRefs, Anim, EveIKSolver->chainRef(Index));
		JointHelpers::localToModel(jointsRefs.size(), EveIKSolver->chainRef(Index));

		EveIKSolver->solve(Index, jointsRefs.size() - Size, jointsRefs.size() - 1, Target, total_iter, epsilon);
		SimpleMath::Vector4 HandLS = EveIKSolver->chainRef(Index)[jointsRefs.size() - 1][2];

		JointHelpers::modelToLocal(jointsRefs.size() - Size - 1, jointsRefs.size() - 1, EveIKSolver->chainRef(Index));
		JointHelpers::ChainToAnim(jointsRefs.size() - Size, jointsRefs.size() - 1, jointsRefs, EveIKSolver->chainRef(Index), Anim);

		return HandLS;
	}

	JointSQT projectJointOnXPlane(JointSQT Src)
	{
		Src[2].x = 0.f;
		return Src;
	}
	
	SimpleMath::Vector3 substructJointsLocations(JointSQT& To, JointSQT& From)
	{
		return SimpleMath::Vector3(SimpleMath::Vector4(To[2]) - SimpleMath::Vector4(From[2]));
	}

	void CalcSpinAndArmRotations(SimpleMath::Vector3 Target, int total_iter = 10, float epsilon = 0.001f)
	{
		auto & chain = EveIKSolver->chainRef(2);
		auto & hipsHandChainRef = jointsRefsChains.HipsLeftHand;

		JointSQT refChain[3];

		JointHelpers::AnimToChain(hipsHandChainRef, EveAnimationGraph->getPlayingAnimation(), chain);
		JointHelpers::localToModel(hipsHandChainRef.size(), chain);

		refChain[0] = chain[0] = projectJointOnXPlane(chain[0]);
		refChain[1] = chain[1] = projectJointOnXPlane(chain[hipsHandChainRef.size() - 3]);
		refChain[2] = chain[2] = projectJointOnXPlane(chain[hipsHandChainRef.size() - 1]);

		Target.x = 0;

		EveIKSolver->solve(2, 0, 2, Target, total_iter, epsilon);

		SpinRotation = TDeltaRotation2(substructJointsLocations(refChain[1], refChain[0]), substructJointsLocations(chain[1], chain[0]));
		SpinRotation.Transform(refChain[1][2], refChain[0][2]);
		SpinRotation.Transform(refChain[2][2], refChain[0][2]);
		ArmRotation = TDeltaRotation2(substructJointsLocations(refChain[2], refChain[1]), substructJointsLocations(chain[2], chain[1]));

		//sprintf(DebugBuffer, "BracedHangHopUpAnimation CalcSpinAndArmRotations SpinRotation %f ArmRotation %f\n", (SpinRotation.RotationAngle / XM_PI)*180.f, (ArmRotation.RotationAngle / XM_PI)*180.f); Debug();

		/*{
			auto modelTransform = SimpleMath::Matrix::CreateScale(0.01f) * SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
			g_EvePath.Count = 3;
			g_EvePath.Pivots[0] = SimpleMath::Vector4(SimpleMath::Vector3::Transform(SimpleMath::Vector3(SimpleMath::Vector4(chain[0][2])), modelTransform));
			g_EvePath.Pivots[1] = SimpleMath::Vector4(SimpleMath::Vector3::Transform(SimpleMath::Vector3(SimpleMath::Vector4(chain[1][2])), modelTransform));
			g_EvePath.Pivots[2] = SimpleMath::Vector4(SimpleMath::Vector3::Transform(SimpleMath::Vector3(SimpleMath::Vector4(chain[2][2])), modelTransform));
			g_EvePath.Pivots[0].w = g_EvePath.Pivots[1].w = g_EvePath.Pivots[2].w = 1.f;
		}*/
	}

	void ApplySpinAndArmRotations()
	{
		auto & leftHandChain = EveIKSolver->chainRef(2);
		auto & rightHandChain = EveIKSolver->chainRef(3);

		auto & hipsLeftHandChainRef = jointsRefsChains.HipsLeftHand;
		auto & hipsRightHandChainRef = jointsRefsChains.HipsRightHand;

		auto& CurrentJoints = EveAnimationGraph->getPlayingAnimation()->CurrentJoints;

		//auto IndexRightArm = jointsRefsChains.HipsRightHand[jointsRefsChains.HipsRightHand.size() - 3];
		//SpinRotation.Transform(CurrentJoints[hipsHandChainRef[1]][2], CurrentJoints[hipsHandChainRef[0]][2]);
		//SpinRotation.Add(CurrentJoints[hipsHandChainRef[1]][1]);
		//ArmRotation.Add(CurrentJoints[hipsHandChainRef[hipsHandChainRef.size() - 3]][1]);
		//ArmRotation.Add(CurrentJoints[IndexRightArm][1]);

		JointHelpers::AnimToChain(hipsLeftHandChainRef, EveAnimationGraph->getPlayingAnimation(), leftHandChain);
		JointHelpers::AnimToChain(hipsRightHandChainRef, EveAnimationGraph->getPlayingAnimation(), rightHandChain);

		JointHelpers::localToModel(hipsLeftHandChainRef.size(), leftHandChain);

		/*
		SpinRotation.Transform(chain[1][2], chain[0][2]);
		SpinRotation.Add(chain[1][1]);
		GetRelativeJoint(chain[0], chain[1]);

		SpinRotation.Add(chain[hipsHandChainRef.size() - 3][1]);
		ArmRotation.Add(chain[hipsHandChainRef.size() - 3][1]);
		GetRelativeJoint(chain[hipsHandChainRef.size() - 4], chain[hipsHandChainRef.size() - 3]);

		CurrentJoints[hipsHandChainRef[1]] = chain[1];
		CurrentJoints[hipsHandChainRef[hipsHandChainRef.size() - 3]] = chain[hipsHandChainRef.size() - 3];
		*/

		SpinRotation.Transform(leftHandChain[1][2], leftHandChain[0][2]);
		GetEdgeHorizontalJumpAnimation_RotateChain(leftHandChain, SpinRotation.Delta, 1, hipsLeftHandChainRef.size() - 2);
		GetEdgeHorizontalJumpAnimation_RotateChain(leftHandChain, ArmRotation.Delta, hipsLeftHandChainRef.size() - 3, hipsLeftHandChainRef.size() - 2);
		//right hand
		std::memcpy(&rightHandChain[0], &leftHandChain[0], sizeof(leftHandChain[0])*(hipsLeftHandChainRef.size() - 4));
		JointHelpers::localToModel(hipsRightHandChainRef.size() - 4, hipsRightHandChainRef.size() - 2, rightHandChain);
		GetEdgeHorizontalJumpAnimation_RotateChain(rightHandChain, ArmRotation.Delta, hipsRightHandChainRef.size() - 3, hipsRightHandChainRef.size() - 2);
		//

		GetRelativeJoint(leftHandChain[0], leftHandChain[1]);
		GetRelativeJoint(leftHandChain[hipsLeftHandChainRef.size() - 4], leftHandChain[hipsLeftHandChainRef.size() - 3]);
		//right hand
		GetRelativeJoint(rightHandChain[hipsRightHandChainRef.size() - 4], rightHandChain[hipsRightHandChainRef.size() - 3]);
		//

		CurrentJoints[hipsLeftHandChainRef[1]] = leftHandChain[1];
		CurrentJoints[hipsLeftHandChainRef[hipsLeftHandChainRef.size() - 3]] = leftHandChain[hipsLeftHandChainRef.size() - 3];
		//right hand
		CurrentJoints[hipsRightHandChainRef[hipsRightHandChainRef.size() - 3]] = rightHandChain[hipsRightHandChainRef.size() - 3];
		//
	}

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		savedElapsedTime = elapsedTime;
		Impl->prev_frameNo = Impl->frameNo;
		
		if (RootVelocity.LengthSquared() == 0.f)
		{
			CurrentPose->advanse(elapsedTime, DeltaTranslation, DeltaRotation);
			CurrentJoints = CurrentPose->CurrentJoints;
			CurrentMetaChannels = CurrentPose->CurrentMetaChannels;
			if (Impl->global_time <= 0.55f && 0.55f <= (Impl->global_time + elapsedTime))
			{
				RootVelocity = 4.0f * (1.f / elapsedTime) * DeltaTranslation.Length() * SimpleMath::Vector3(0, 1, 0);
				HandVelocity = RootVelocity;
			}
		}
		else
		{
			DeltaTranslation = elapsedTime * RootVelocity;
			CurrentJoints = ::__AnimGetJointsByTime(CurrentPose, 0.55f);
			CurrentMetaChannels = CurrentPose->CurrentMetaChannels;
		}

		RootSampledRotation = SimpleMath::Quaternion(CurrentJoints[64][1]);
		CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(
			RootDeltaRotation,
			RootSampledRotation
		);

		Impl->frameNo += 1;
		//sprintf(DebugBuffer, "BracedHangHopUpAnimation advanse global_time %f %f %f \n", Impl->global_time, Impl->global_time + elapsedTime, elapsedTime); Debug();
		Impl->global_time += elapsedTime;
	}

	void PostActions()
	{
		SimpleMath::Quaternion InverseOrientation;
		GWorld.Capsules["eve"].orientation.Inverse(InverseOrientation);
		const auto InverseEeveSkinnedModel = SimpleMath::Matrix::CreateRotationY(90.0*PI / 180.0) * SimpleMath::Matrix::CreateScale(1.f / 5, 1.f / 5, 1.f / 5);

		const auto FromWSToLS = SimpleMath::Matrix::CreateTranslation(-GWorld.Capsules["eve"].origin) * SimpleMath::Matrix::CreateFromQuaternion(InverseOrientation) * InverseEeveSkinnedModel;

		if (HandVelocity.LengthSquared() == 0.f)
		{
			HandIK(0, EveAnimationGraph->getPlayingAnimation(), jointsRefsChains.HipsLeftHand, 3, (1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_LeftHand, FromWSToLS));
			HandIK(1, EveAnimationGraph->getPlayingAnimation(), jointsRefsChains.HipsRightHand, 3, (1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_RightHand, FromWSToLS));
			
			CalcSpinAndArmRotations((1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_LeftHand, FromWSToLS));
			ApplySpinAndArmRotations();
		}
		else if (Initial_HandUpLeftHand.LengthSquared() == 0.f)
		{
			HandIK(0, EveAnimationGraph->getPlayingAnimation(), jointsRefsChains.HipsLeftHand, 3, (1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_LeftHand, FromWSToLS), 500);
			HandIK(1, EveAnimationGraph->getPlayingAnimation(), jointsRefsChains.HipsRightHand, 3, (1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_RightHand, FromWSToLS));

			CalcSpinAndArmRotations((1.f / 0.01f) * SimpleMath::Vector3::Transform(Initial_LeftHand, FromWSToLS), 500);
			ApplySpinAndArmRotations();

			JointHelpers::AnimToChain(jointsRefsChains.HipsLeftHand, EveAnimationGraph->getPlayingAnimation(), EveIKSolver->chainRef(0));
			JointHelpers::localToModel(jointsRefsChains.HipsLeftHand.size(), EveIKSolver->chainRef(0));
			Initial_HandUpLeftHand = SimpleMath::Vector3(SimpleMath::Vector4(EveIKSolver->chainRef(0)[jointsRefsChains.HipsLeftHand.size() - 1][2]));

			JointHelpers::AnimToChain(jointsRefsChains.HipsLeftHand, EveAnimationGraph->getPlayingAnimation(), EveIKSolver->chainRef(0));
			JointHelpers::AnimToChain(jointsRefsChains.HipsRightHand, EveAnimationGraph->getPlayingAnimation(), EveIKSolver->chainRef(1));

			handUpTotalTime = 0.f;
			sprintf(DebugBuffer, "BracedHangHopUpAnimation State(Initial_HandUpLeftHand.LengthSquared() == 0.f)\n"); Debug();
		}
		else if (!LeftHand_Solved)
		{
			ApplySpinAndArmRotations();

			handUpTotalTime += savedElapsedTime;
			auto handUpTotalSquareDist = std::min((handUpTotalTime * HandVelocity).LengthSquared(), 6.04f);

			const auto DesiredHandLocation = Initial_HandUpLeftHand + (1.f / 0.01f) * SimpleMath::Vector3::TransformNormal(std::sqrt(handUpTotalSquareDist) * normalize(HandVelocity), FromWSToLS);
			auto ActualHandLocation = HandIK(0, EveAnimationGraph->getPlayingAnimation(), jointsRefsChains.HipsLeftHand, 3, DesiredHandLocation, 500);
			JointHelpers::ChainToAnim(jointsRefsChains.HipsRightHand.size() - 3, jointsRefsChains.HipsRightHand.size() - 1, jointsRefsChains.HipsRightHand, EveIKSolver->chainRef(1), EveAnimationGraph->getPlayingAnimation());

			if (handUpTotalSquareDist == 6.04f)
			{
				sprintf(DebugBuffer, "BracedHangHopUpAnimation State(!LeftHand_Solved) %f Done\n", (DesiredHandLocation - ActualHandLocation).Length()); Debug();
				LeftHand_Solved = true;
			}
			//sprintf(DebugBuffer, "BracedHangHopUpAnimation State(!LeftHand_Solved) %f %f (%f %f %f)\n", handUpTotalSquareDist, (DesiredHandLocation - ActualHandLocation).Length(), DesiredHandLocation.x, DesiredHandLocation.y, DesiredHandLocation.z); Debug();
		}
		else
		{
			ApplySpinAndArmRotations();

			JointHelpers::ChainToAnim(jointsRefsChains.HipsLeftHand.size() - 3, jointsRefsChains.HipsLeftHand.size() - 1, jointsRefsChains.HipsLeftHand, EveIKSolver->chainRef(0), EveAnimationGraph->getPlayingAnimation());
			JointHelpers::ChainToAnim(jointsRefsChains.HipsRightHand.size() - 3, jointsRefsChains.HipsRightHand.size() - 1, jointsRefsChains.HipsRightHand, EveIKSolver->chainRef(1), EveAnimationGraph->getPlayingAnimation());
		}

		fillSkeletonTransformFromJoint(EveAnimationGraph->getPlayingAnimation(), Eve->skelet);
		calculateFramesTransformations(Eve->frame, SimpleMath::Matrix::Identity);

		{
			g_EvePath.Count = 2;
			g_EvePath.Pivots[0] = SimpleMath::Vector4(Initial_LeftHand + SimpleMath::Vector3(0, 1000, 0));
			g_EvePath.Pivots[1] = SimpleMath::Vector4(Initial_LeftHand + SimpleMath::Vector3(0, -1000, 0));
			g_EvePath.Pivots[0].w = g_EvePath.Pivots[1].w = g_EvePath.Pivots[2].w = 1.f;
		}
	}

	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
	{
		onPlayingChanged = callback;
	}

	bool getPlaying()
	{
		return Impl->playing;
	}

	void setPlaying(bool value)
	{
		CurrentPose->setPlaying(value);
		if (Impl->playing != value)
		{
			Impl->playing = value;
			if (onPlayingChanged)
				onPlayingChanged(Impl->playing);
		}
	}

	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}

	void reset()
	{
		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->prev_local_time = Impl->local_time = .0;

		Impl->global_time = .0;

		Impl->playing = false;

		Impl->Rate = 1.f;

		CurrentPose->reset();

		Initial_HandUpLeftHand = RootVelocity = HandVelocity = SimpleMath::Vector3::Zero;

		LeftHand_Solved = false;
	}

	std::vector<JointSQT>& __AnimGetJointsByTime(float Time)
	{
		return ::__AnimGetJointsByTime(CurrentPose, Time);
	};

	void SetBlendTime(double& blendTime, bool& blendRoot){
		blendRoot = false;
		blendTime = 0.1f;
	};

	int getFrame()
	{
		return Impl->prev_frameNo;
	}

	double getLocTime(){
		return Impl->prev_local_time;
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	double getRate()
	{
		return 1;
	}

	void setRate(double speed)
	{
	};

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}

	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}

	void setLooping(bool loop)
	{
	}

	bool IsLoop(){
		return false;
	};


};

Animation* CreateBracedHangHopUpAnimation(std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
{
	auto bracedHangHopUpAnimation = loadAnimation("Media\\Animations\\BracedHangHopUp.dae", FramesNamesIndex);
	bracedHangHopUpAnimation->setRate(1.0 / 2.0f);
	bracedHangHopUpAnimation->setLooping(false);
	//bracedHangHopUpAnimation->AddOffset(-gravityInEveSystemCoordinates);
	extractAnimationMeta(bracedHangHopUpAnimation, true, 1.0f, getSkeletMatrix, calculateFramesTrans);
	//bracedHangHopUpAnimation->TransformMetaSamples(
	//	0,
	//	[](SimpleMath::Vector4 v){
	//	v = SimpleMath::Vector4(v.x, 1.0f*v.y, v.z, 1.0f);
	//	return v;
	//});

	auto Anim = new BracedHangHopUpAnimation();
	Anim->CurrentPose = bracedHangHopUpAnimation;

	return Anim;
}

void BracedHangHopUpAnimation_SetInitialHandsLocations(Animation* __Animation)
{
	auto Anim = ((BracedHangHopUpAnimation*)__Animation);

	Anim->Initial_LeftHand = GetEdgeHorizontalJumpAnimation_GetHandLocation(false);

	Anim->Initial_RightHand = GetEdgeHorizontalJumpAnimation_GetHandLocation(true);
}