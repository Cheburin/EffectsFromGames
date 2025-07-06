#include "main.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#include "AnimationImpl.h"

#undef min // use __min instead
#undef max // use __max instead

extern World GWorld;
extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;

extern char DebugBuffer[1024];
void Debug();

Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);
std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);
std::vector<SimpleMath::Vector3>& __AnimGetMetaByTime(AnimationBase* Anim, float Time);
SimpleMath::Quaternion __AnimSubstructRootDeltaRotation(AnimationBase* Anim);

void rotateHips(Animation * anim, SimpleMath::Quaternion Rotation);

void GetEdgeHorizontalJumpAnimation_RotateChain(std::vector<JointSQT>& ChainJoints, SimpleMath::Quaternion DeltaRot, int ChainJoints_OriginIndex, int ChainJoints_Count);

extern JointsRefsChainCollection jointsRefsChains;
extern IAnimationGraph2 * EveAnimationGraph;

extern SimpleMath::Vector3 LadderSize;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct CharacterSkelet
{
	CharacterSkelet() :TotalMatrix(), ReservMatrix(){};
	std::map<std::string, unsigned int> FramesNamesIndex;
	unsigned int TotalMatrix;
	unsigned int ReservMatrix;
	std::vector<SimpleMath::Matrix*> Transformation;
	std::vector<SimpleMath::Matrix> BoneOffSetTransformation;
	unsigned int findFrame(const std::string& name)
	{
		auto Iter = FramesNamesIndex.find(name);
		if (FramesNamesIndex.end() != Iter)
		{
			return (*Iter).second;
		}
		else
		{
			unsigned int OldSize = TotalMatrix;
			unsigned int NewSize = ++TotalMatrix;
			FramesNamesIndex.insert(std::pair<std::string, unsigned int>(name, OldSize));
			if (ReservMatrix <= NewSize)
			{
				ReservMatrix += 1024;
				Transformation.reserve(ReservMatrix);
				BoneOffSetTransformation.reserve(ReservMatrix);
			}
			Transformation.resize(NewSize);
			BoneOffSetTransformation.resize(NewSize);
			return OldSize;
		}
	}
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern bool state_climbing;
bool state_movement_on_ladder = false;
const double LadderAnimation_BlendTime = 0.99f;
extern SimpleMath::Vector2 input_move;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct LadderAnimation : public AnimationWithImpl
{
	static const int PoseCount = 3;
	std::shared_ptr<Animation> Poses[PoseCount];

	std::vector<JointSQT> CurrentPoseJoints;
	
	SimpleMath::Quaternion Pose1Orientation;
	SimpleMath::Vector3 Pose1Velocity_WS;
	SimpleMath::Vector3 Pose1Neck_WS;
	float Pose1RotationAngle2;

	float Pose1OffsetDistance2;
	float Pose1TotalOffsetDistance;
	SimpleMath::Vector3 Pose1OffsetDirection;

	SimpleMath::Vector3 Pose2Pivot;

	double TotalTime;

	double PoseTime;

	double PrevPoseTime1;
	double PrevPoseTime2;

	int CurrentPoseIndex;

	int prev_input;

	SimpleMath::Vector3 Pose3HoldHand;
	float Pose3Time;
	float Pose3HoldHandIndex;
	int Pose3LadderItemIndex;
	SimpleMath::Vector3 Pose3HandConstOffset;
	SimpleMath::Plane Pose3CharacterLadderPlane;

	LadderAnimation(std::map<std::string, unsigned int>& FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
	{
		Poses[0].reset(loadAnimation("Media\\Animations\\Kneeling.dae", FramesNamesIndex));
		extractAnimationMeta(Poses[0].get(), true, 1.0f, getSkeletMatrix, calculateFramesTrans);
		__AnimSubstructRootDeltaRotation(Poses[0].get());

		Poses[1].reset(loadAnimationFromBlender("Media\\Animations\\Ladder\\Ladder01.dae", FramesNamesIndex)); //ClimbingLadder.dae
		extractAnimationMeta(Poses[1].get(), true, 1.0f, getSkeletMatrix, calculateFramesTrans);
		__AnimSubstructRootDeltaRotation(Poses[1].get());

		Poses[2].reset(loadAnimation("Media\\Animations\\Ladder\\ClimbingLadder.dae", FramesNamesIndex));
		extractAnimationMeta(Poses[2].get(), true, 1.0f, getSkeletMatrix, calculateFramesTrans);
		//__AnimSubstructRootDeltaRotation(Poses[2].get());
		rotateHips(Poses[2].get(), SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY(PI)));
		Poses[2].get()->setLooping(true);
	}

	std::vector<JointSQT>& __AnimGetJointsByTime(float Time)
	{
		if (CurrentPoseIndex < 2)
		{
			return ::__AnimGetJointsByTime(Poses[CurrentPoseIndex].get(), 0.f);
		}
		else if (CurrentPoseIndex == 2)
		{
			auto& Joints = ::__AnimGetJointsByTime(Poses[CurrentPoseIndex].get(), 0.565217f); //0.565217f
			auto V1 = SimpleMath::Matrix::CreateFromQuaternion(Joints[64][1]).Forward();
			auto V2 = SimpleMath::Vector3(0, 0, -1);
			auto Delta = TDeltaRotation(V1, V2);
			sprintf(DebugBuffer, "LadderAnimation __AnimGetJointsByTime(%f)\n", (Delta.RotationAngle / XM_PI)*180.f); Debug();
			Joints[64][1] = SimpleMath::Quaternion::Concatenate(Delta.Delta, Joints[64][1]);
			return Joints;
		}
		else
		{
			return ::__AnimGetJointsByTime(Poses[2].get(), PoseTime);
		}
	};
	
	std::vector<SimpleMath::Vector3>& __AnimGetMetaByTime(float Time)
	{
		if (CurrentPoseIndex < 2)
		{
			return ::__AnimGetMetaByTime(Poses[CurrentPoseIndex].get(), 0.f);
		}
		else if (CurrentPoseIndex == 2)
		{
			return ::__AnimGetMetaByTime(Poses[CurrentPoseIndex].get(), 0.565217f); //0.565217f
		}
		else
		{
			return ::__AnimGetMetaByTime(Poses[2].get(), PoseTime);
		}
	};

	void SetBlendTime(double& blendTime, bool& blendRoot)
	{
		blendTime = CurrentPoseIndex == 3?0.f:LadderAnimation_BlendTime;
		blendRoot = false;
	};

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		Impl->prev_frameNo = Impl->frameNo;

		DeltaTranslation = SimpleMath::Vector3::Zero;
		DeltaRotation = SimpleMath::Quaternion::Identity;

		if (CurrentPoseIndex == 3)
		{
			Poses[2].get()->advanse(elapsedTime, DeltaTranslation, DeltaRotation);
			CurrentJoints = CurrentPoseJoints = Poses[2].get()->CurrentJoints;
		}

		RootSampledRotation = SimpleMath::Quaternion(CurrentPoseJoints[64][1]);
		CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(
			RootDeltaRotation,
			RootSampledRotation
		);

		Impl->frameNo += 1;
		Impl->global_time += elapsedTime;

		TotalTime += elapsedTime;

		if (CurrentPoseIndex != 3 || Poses[2].get()->getPlaying())
		{
			PrevPoseTime1 = PrevPoseTime2;
			PrevPoseTime2 = PoseTime;
			PoseTime += elapsedTime;
		}
	}

	SimpleMath::Vector3 GetJointLocation_WS(int index, int sub_index, AnimationBase* Anim)
	{
		std::vector<JointSQT> Joints; Joints.resize(256);
		const auto FromLSToWS = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		JointHelpers::AnimToChain(jointsRefsChains[index], Anim, Joints);
		JointHelpers::localToModel(jointsRefsChains[index].size(), Joints);
		return SimpleMath::Vector3::Transform(0.01f*SimpleMath::Vector3(SimpleMath::Vector4(Joints[jointsRefsChains[index].size() - sub_index][2])), FromLSToWS);
	}

	SimpleMath::Vector3 GetJointLocation_LS(int index, int sub_index, AnimationBase* Anim)
	{
		std::vector<JointSQT> Joints; Joints.resize(256);
		JointHelpers::AnimToChain(jointsRefsChains[index], Anim, Joints);
		JointHelpers::localToModel(jointsRefsChains[index].size(), Joints);
		return 0.01f*SimpleMath::Vector3(SimpleMath::Vector4(Joints[jointsRefsChains[index].size() - sub_index][2]));
	}
	
	SimpleMath::Vector3 GetJointLocation_LS(std::vector<int>& Chains, int sub_index, AnimationBase* Anim)
	{
		std::vector<JointSQT> Joints; Joints.resize(256);
		JointHelpers::AnimToChain(Chains, Anim, Joints);
		JointHelpers::localToModel(Chains.size(), Joints);
		return 0.01f*SimpleMath::Vector3(SimpleMath::Vector4(Joints[Chains.size() - sub_index][2]));
	}

	SimpleMath::Vector3 GetJointLocation_WS(std::vector<int>& Chains, int sub_index, AnimationBase* Anim)
	{
		std::vector<JointSQT> Joints; Joints.resize(256);
		const auto FromLSToWS = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		JointHelpers::AnimToChain(Chains, Anim, Joints);
		JointHelpers::localToModel(Chains.size(), Joints);
		return SimpleMath::Vector3::Transform(0.01f*SimpleMath::Vector3(SimpleMath::Vector4(Joints[Chains.size() - sub_index][2])), FromLSToWS);
	}

	std::vector<int> GetRightHandMiddleFingerChain()
	{
		std::vector<int> Hips_MiddleFinger_Chain = jointsRefsChains[0];
		Hips_MiddleFinger_Chain.insert(Hips_MiddleFinger_Chain.end(), jointsRefsChains.Hand_Childs[1][2].begin(), jointsRefsChains.Hand_Childs[1][2].end());
		return Hips_MiddleFinger_Chain;
	}

	std::vector<int> GetLeftHandMiddleFingerChain()
	{
		std::vector<int> Hips_MiddleFinger_Chain = jointsRefsChains[1];
		Hips_MiddleFinger_Chain.insert(Hips_MiddleFinger_Chain.end(), jointsRefsChains.Hand_Childs[0][2].begin(), jointsRefsChains.Hand_Childs[0][2].end());
		return Hips_MiddleFinger_Chain;
	}

	std::vector<JointSQT> GetJoints(std::vector<int>& Chains, AnimationBase* Anim)
	{
		std::vector<JointSQT> Joints; Joints.resize(Chains.size());
		JointHelpers::AnimToChain(Chains, Anim, Joints);
		JointHelpers::localToModel(Joints.size(), Joints);
		return Joints;
	}

	void PostActions()
	{
		if (CurrentPoseIndex == 1)
		{
			if (EveAnimationGraph->getAnimationBlend()->isPlaying())
			{
				const auto Current_NeckLocation_WS = GetJointLocation_WS(2, 3, EveAnimationGraph->getPlayingAnimation());

				const auto NeckDeltaTranslation = Pose1Neck_WS - Current_NeckLocation_WS;
				GWorld.Capsules["eve"].origin += SimpleMath::Vector3(NeckDeltaTranslation.x, NeckDeltaTranslation.y, NeckDeltaTranslation.z);

				const auto ElapsedTime = std::min(LadderAnimation_BlendTime, PrevPoseTime2) - std::min(LadderAnimation_BlendTime, PrevPoseTime1);
				const auto ForeArmDeltaTranslation = ElapsedTime*Pose1Velocity_WS;
				GWorld.Capsules["eve"].origin += ForeArmDeltaTranslation;

				Pose1Neck_WS += ForeArmDeltaTranslation;

				const auto Current_RightForeArm_WS = GetJointLocation_WS(0, 2, EveAnimationGraph->getPlayingAnimation());
				sprintf(DebugBuffer, "LadderAnimation PrevPoseTime1(%f) PrevPoseTime2(%f) LadderTranslation_WS(%f %f) PoseNeck_WS(%f %f) RightForeArm_WS(%f %f)\n", float(PrevPoseTime1), float(PrevPoseTime2),
					SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation().x, SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation().z,
					Current_NeckLocation_WS.x, Current_NeckLocation_WS.z, Current_RightForeArm_WS.x, Current_RightForeArm_WS.z); Debug();
			}
			else
			{
				const auto Pose1RotationAngle1 = Pose1RotationAngle2;
				Pose1RotationAngle2 += (PrevPoseTime2 - PrevPoseTime1) * /*velocity*/45.f;
				const auto Pose1DeltaRotation = std::min(90.f, Pose1RotationAngle2) - std::min(90.f, Pose1RotationAngle1);

				SimpleMath::Quaternion DeltaRot = SimpleMath::Quaternion::CreateFromAxisAngle(SimpleMath::Vector3::Up, Pose1DeltaRotation*(PI/180.f));

				const auto ForeArm_LS = GetJointLocation_LS(0, 2, EveAnimationGraph->getPlayingAnimation());
				const auto OriginDeltaTranslation_LS = SimpleMath::Vector3::Transform(SimpleMath::Vector3() - ForeArm_LS, DeltaRot) + ForeArm_LS;

				const auto FromLSToWS = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

				GWorld.Capsules["eve"].origin += SimpleMath::Vector3::TransformNormal(OriginDeltaTranslation_LS, FromLSToWS);
				GWorld.Capsules["eve"].orientation = SimpleMath::Quaternion::Concatenate(DeltaRot, GWorld.Capsules["eve"].orientation);

				Pose1RotationAngle2 = std::min(90.f, Pose1RotationAngle2);

				const auto Pose1OffsetDistance1 = Pose1OffsetDistance2;
				Pose1OffsetDistance2 += (PrevPoseTime2 - PrevPoseTime1) * /*velocity*/Pose1TotalOffsetDistance / 2.f;
				const auto Pose1DeltaOffset = std::min(Pose1TotalOffsetDistance, Pose1OffsetDistance2) - std::min(Pose1TotalOffsetDistance, Pose1OffsetDistance1);

				GWorld.Capsules["eve"].origin += Pose1DeltaOffset*Pose1OffsetDirection;
			}
		}
		else if (CurrentPoseIndex == 2)
		{
			const auto SubHand_WS = GetJointLocation_WS(GetRightHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());

			GWorld.Capsules["eve"].origin += Pose2Pivot - SubHand_WS;
		}
		else if (CurrentPoseIndex == 3)
		{
			if ((Pose3HoldHandIndex == 2 && 0.45f < Pose3Time) || (Pose3HoldHandIndex == 1 && 0.58f < Pose3Time))
			{
				auto DeltaTranslation = SimpleMath::Vector3::Zero;
				auto DeltaRotation = SimpleMath::Quaternion::Identity;

				Pose3Time = (Pose3HoldHandIndex == 1 ? 0.58f : 0.42f) - Pose3Time;
				//Poses[2].get()->advanse(-0.565217f - (PoseTime - int(PoseTime)), DeltaTranslation, DeltaRotation);
				Poses[2].get()->advanse(Pose3Time, DeltaTranslation, DeltaRotation);
				GWorld.Capsules["eve"].origin += DeltaTranslation;

				Pose3Time *= -1.f;
				Pose3HoldHandIndex = (Pose3HoldHandIndex == 1 ? 2 : 1);
				const auto __SubLeftHand_WS = GetJointLocation_WS(GetLeftHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());
				const auto __SubRightHand_WS = GetJointLocation_WS(GetRightHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());
				Pose3HoldHand = (Pose3HoldHandIndex == 1 ? __SubLeftHand_WS : __SubRightHand_WS);
				sprintf(DebugBuffer, "LadderAnimation CurrentPoseIndex == 3(true) PoseTime == %f Pose3HoldHandTime == %f Delta == %f\n", Poses[2].get()->getLocTime(), Pose3Time, DeltaTranslation.Length()); Debug();
				extern bool simulation_state_manual_control;
				//simulation_state_manual_control = true;
			}
			else
			{
				const auto SubLeftHand_WS = GetJointLocation_WS(GetLeftHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());

				const auto SubRightHand_WS = GetJointLocation_WS(GetRightHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());

				GWorld.Capsules["eve"].origin += Pose3HoldHand - (Pose3HoldHandIndex == 1 ? SubLeftHand_WS : SubRightHand_WS);
				//sprintf(DebugBuffer, "LadderAnimation CurrentPoseIndex == 3(false) PoseTime == %f Pose3HoldHandTime == %f\n", PoseTime, Pose3Time); Debug();
			}
			Pose3Time += (PrevPoseTime2 - PrevPoseTime1);
			//sprintf(DebugBuffer, "LadderAnimation CurrentPoseIndex == 3 DeltaTime == %f Pose3Time == %f\n", (PrevPoseTime2 - PrevPoseTime1), Pose3Time); Debug();
			return;

			const auto SubLeftHand_WS = GetJointLocation_WS(GetLeftHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());
			const auto SubRightHand_WS = GetJointLocation_WS(GetRightHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());
			
			auto HoldingHand = (Pose3HoldHandIndex == 1 ? SubLeftHand_WS : SubRightHand_WS);
			auto OriginOffset = Pose3HoldHand - HoldingHand;
			auto MovingHand = (Pose3HoldHandIndex == 1 ? SubRightHand_WS : SubLeftHand_WS) + OriginOffset;

			const auto LadderOrigin = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation();
			const auto LadderHeightBetweenItems = fabs(SimpleMath::Vector3::TransformNormal(SimpleMath::Vector3(0.f, 0.1448f, 0.f), SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"])).y);
			const auto LadderLeftDistanceBetweenTwoItems = MovingHand.y - (LadderOrigin.y - (Pose3LadderItemIndex + 2)*LadderHeightBetweenItems + Pose3HandConstOffset.y);

			bool NextLadderItem = false;

			if (LadderLeftDistanceBetweenTwoItems < 0.f)
			{
				OriginOffset.y -= LadderLeftDistanceBetweenTwoItems;
				MovingHand.y -= LadderLeftDistanceBetweenTwoItems;

				Pose3LadderItemIndex++;
				Pose3HoldHand = MovingHand;
				Pose3HoldHandIndex = (Pose3HoldHandIndex == 1 ? 2 : 1);

				NextLadderItem = true;
			}

			GWorld.Capsules["eve"].origin += OriginOffset;
			GWorld.Capsules["eve"].origin -= Pose3CharacterLadderPlane.DotCoordinate(GWorld.Capsules["eve"].origin)*Pose3CharacterLadderPlane.Normal(); //-36.64;

			Pose3Time += (PrevPoseTime2 - PrevPoseTime1);

			if (NextLadderItem) 
			{
				sprintf(DebugBuffer, "LadderAnimation CurrentPoseIndex == 3 PoseTime == %f Pose3HoldHandTime == %f Left == %f (%f %f %f)\n", PoseTime, Pose3Time, LadderLeftDistanceBetweenTwoItems, GWorld.Capsules["eve"].origin.x, GWorld.Capsules["eve"].origin.y, GWorld.Capsules["eve"].origin.z); Debug();
			}
		}
	}

	void ComputePosesStates()
	{
		SimpleMath::Vector3 ForeArm_WS;
		const auto FromLSToWS = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		{
			Poses[0].get()->CurrentJoints = ::__AnimGetJointsByTime(Poses[0].get(), 0.f);
			Poses[1].get()->CurrentJoints = ::__AnimGetJointsByTime(Poses[1].get(), 0.f);

			auto Pose0Neck_LS = GetJointLocation_LS(2, 3, Poses[0].get());
			auto Pose1Neck_LS = GetJointLocation_LS(2, 3, Poses[1].get());
			const auto ForeArm_LS = GetJointLocation_LS(0, 2, Poses[1].get()) + (Pose0Neck_LS - Pose1Neck_LS);

			ForeArm_WS = SimpleMath::Vector3::Transform(ForeArm_LS, FromLSToWS);
			Pose1Velocity_WS = (SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation() - ForeArm_WS) / LadderAnimation_BlendTime;
			Pose1Velocity_WS.y *= 0.65f;
			Pose1Neck_WS = SimpleMath::Vector3::Transform(Pose0Neck_LS, FromLSToWS);

			//sprintf(DebugBuffer, "LadderAnimation ComputePose1States PoseNeck_WS(%f %f)\n", Pose1Neck_WS.x, Pose1Neck_WS.z); Debug();
		}
		{
			Pose1RotationAngle2 = 0.f;

			auto X = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Forward(); X.Normalize();
			auto DeltaHeight = (SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation().y - ForeArm_WS.y) - (LadderAnimation_BlendTime*Pose1Velocity_WS).y;
			Pose1OffsetDirection = -0.60f*LadderSize.z*SimpleMath::Vector3::Up.Cross(X) + SimpleMath::Vector3(0, 0.55f*DeltaHeight, 0);
			Pose1TotalOffsetDistance = Pose1OffsetDirection.Length();
			Pose1OffsetDirection *= 1.f / Pose1TotalOffsetDistance;
			Pose1OffsetDistance2 = .0f;
		}
		{
			ForeArm_WS += LadderAnimation_BlendTime * Pose1Velocity_WS;
			const auto ForeArm_LS = (1.f / 0.01f)*SimpleMath::Vector3::Transform(ForeArm_WS, FromLSToWS.Invert());

			auto Joints = GetJoints(GetRightHandMiddleFingerChain(), Poses[1].get());

			const auto ForeArmDelta = SimpleMath::Vector3(SimpleMath::Vector4(ForeArm_LS) - SimpleMath::Vector4(Joints[jointsRefsChains[0].size() - 2][2]));
			for (int i = 0; i < Joints.size(); i++)
			{
				Joints[i][2] = SimpleMath::Vector4(Joints[i][2]) + SimpleMath::Vector4(ForeArmDelta);
			}
			GetEdgeHorizontalJumpAnimation_RotateChain(Joints, SimpleMath::Quaternion::CreateFromAxisAngle(SimpleMath::Vector3::Up, 90 * (PI / 180.f)), jointsRefsChains[0].size() - 2, Joints.size());

			const auto SubHand_WS = SimpleMath::Vector3::Transform(0.01f*SimpleMath::Vector3(SimpleMath::Vector4(Joints[Joints.size() - 4][2])), FromLSToWS);

			Pose2Pivot = SubHand_WS + Pose1TotalOffsetDistance * Pose1OffsetDirection;
		}
	}

	bool NextPose(IAnimationGraph2 * Graph, bool bStarting)
	{
		if (bStarting)
		{
			prev_input = 0;

			TotalTime = 0.f;

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			{
				auto V1 = SimpleMath::Matrix::CreateFromQuaternion(Graph->getPlayingAnimation()->CurrentJoints[64][1]).Forward();
				auto V2 = SimpleMath::Vector3(0, 0, -1);
				auto Delta = TDeltaRotation(V1, V2);
				Graph->getPlayingAnimation()->CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(Delta.Delta, Graph->getPlayingAnimation()->CurrentJoints[64][1]);
				GWorld.Capsules["eve"].orientation = SimpleMath::Quaternion::Concatenate(Delta.InverseDelta, GWorld.Capsules["eve"].orientation);
			}
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			CurrentPoseIndex = 0;
			PrevPoseTime1 = PrevPoseTime2 = PoseTime = 0.;

			CurrentJoints = CurrentPoseJoints = __AnimGetJointsByTime(.0f);
			CurrentMetaChannels = __AnimGetMetaByTime(.0f);

			{
				auto V1 = GWorld.Capsules["eve"].getMatrix().Right();
				auto V2 = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Forward(); V2.Normalize();
				TDeltaRotation DeltaRotation(V1, V2);
				auto X = V2;
				auto Y = SimpleMath::Vector3(0, 1, 0);
				Simulation::UpdateCapsuleRotation_SetParams(DeltaRotation.Delta, Pose1Orientation = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(X, Y, X.Cross(Y))));
			}

			return true;
		}
		else if (!Graph->getAnimationBlend()->isPlaying() && CurrentPoseIndex == 0)
		{
			CurrentPoseIndex++;
			PrevPoseTime1 = PrevPoseTime2 = PoseTime = 0.;

			CurrentJoints = CurrentPoseJoints = __AnimGetJointsByTime(.0f);
			CurrentMetaChannels = __AnimGetMetaByTime(.0f);

			ComputePosesStates();

			{
				Simulation::UpdateCapsuleRotation_SetParams(SimpleMath::Quaternion::Identity, Pose1Orientation);
			}

			return true;
		}
		else if (Pose1RotationAngle2 == 90.f && CurrentPoseIndex == 1)
		{
			CurrentPoseIndex++;
			PrevPoseTime1 = PrevPoseTime2 = PoseTime = 0.;

			CurrentJoints = CurrentPoseJoints = __AnimGetJointsByTime(.0f);
			CurrentMetaChannels = __AnimGetMetaByTime(.0f);

			{
				auto V1 = GWorld.Capsules["eve"].getMatrix().Right();
				auto V2 = -1.f*SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Forward(); V2.Normalize();
				TDeltaRotation DeltaRotation(V1, V2);
				auto X = V2;
				auto Y = SimpleMath::Vector3(0, 1, 0);
				Simulation::UpdateCapsuleRotation_SetParams(DeltaRotation.Delta, Pose1Orientation = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(X, Y, X.Cross(Y))));
			}

			return true;
		}
		else if (!Graph->getAnimationBlend()->isPlaying() && CurrentPoseIndex == 2)
		{
			CurrentPoseIndex++;
			PrevPoseTime1 = PrevPoseTime2 = PoseTime = -0.565217f;

			Simulation::UpdateCapsuleRotation_SetParams(SimpleMath::Quaternion::Identity, Pose1Orientation);

			Poses[2].get()->reset();
			Poses[2].get()->setCurrentAnimationTime(-PoseTime);
			Poses[2].get()->setPlaying(false);
			Poses[2].get()->setRate(-1);

			Pose3HoldHand = GetJointLocation_WS(GetLeftHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation());
			Pose3Time = 0.f;
			Pose3HoldHandIndex = 1;
			Pose3LadderItemIndex = 0;
			Pose3HandConstOffset = GetJointLocation_WS(GetRightHandMiddleFingerChain(), 4, EveAnimationGraph->getPlayingAnimation()) - SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation();
			{
				auto point = GWorld.Capsules["eve"].getMatrix().Translation();
				auto normal = -1.f*SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Forward(); normal.Normalize();
				Pose3CharacterLadderPlane = SimpleMath::Plane(point, normal);
			}

			return true;
		}
		else if (CurrentPoseIndex == 3)
		{
			if (prev_input == 0 && input_move.x != 0)
			{
				Poses[2].get()->setRate(input_move.x > 0 ? 1 : -1);
			}
			Poses[2].get()->setPlaying(true);// input_move.x != 0.f);
			prev_input = input_move.x;
			return false;
		}
		else
		{
			return false;
		}
	}
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CreateLadderAnimationAndRegisterIntoAnimationGraph(IAnimationGraph2 * Graph, CharacterSkelet * characterSkelet, TransformationFrame * frame)
{
	auto getSkeletMatrix = [characterSkelet](unsigned int index){return characterSkelet->Transformation[index]; };
	auto calculateFramesTrans = [frame](){ calculateFramesTransformations(frame, SimpleMath::Matrix::Identity); };

	auto LadderAnim = new LadderAnimation(characterSkelet->FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
	LadderAnim->subscribe("onPlayingChanged", [](bool state){
		state_climbing = state;
	});

	Graph->registerAnimation("ladder", LadderAnim);
	Graph->createLink(LadderAnim)
		.setEndPoint(Graph->getAnimation("walking"))
		.reverse([Graph, LadderAnim](){
			if (state_movement_on_ladder)
			{
				state_movement_on_ladder = false;
				return LadderAnim->NextPose(Graph, true);
			}
			else
			{
				return false;
			}
	});
	Graph->createLink(LadderAnim)
		.setEndPoint(LadderAnim)
		.reverse([Graph, LadderAnim](){
			return LadderAnim->NextPose(Graph, false);
	});
}