#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern char* state_hanging_Hand_Name;
extern SimpleMath::Vector3 state_hanging_Hand_Location;

namespace
{
	std::set<std::pair<char*, char*>> ApplyRotationByAnimChange;

	SimpleMath::Quaternion PrevRootRotation;

	SimpleMath::Quaternion TargetRootDeltaRotation = SimpleMath::Quaternion::Identity;

	SimpleMath::Quaternion InverseRootTotalRotation = SimpleMath::Quaternion::Identity;

	//char* CurrentAnimationName = nullptr;

	//bool SignalResetRotations = false;

	//int frame = 0;
	//SimpleMath::Vector3 GetNormalizedVector(SimpleMath::Vector3 V1)
	//{
	//	V1.Normalize();
	//	return V1;
	//}

	SimpleMath::Quaternion TargetOrientation;

	bool InProgress = false;
}

void GetStartYaw_PrevRoot_CurrentRoot(AnimationBase* blend, float& StartYaw, SimpleMath::Quaternion &PrevRootRotation, JointSQT* &CurrentRootRotation);
SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);
std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index, float Time = 0);

void __AnimSetRootDeltaRotation(AnimationBase* Anim, SimpleMath::Quaternion RootRotation);
void __AnimResampleCurrentRootRotation(AnimationBase* Anim);
void __BlendSetAnimation2RootDeltaRotation(AnimationBase* blend, SimpleMath::Quaternion InverseRotation);

namespace Simulation
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void UpdateCapsuleRotation_AlignHipsJointOrintationToCapsuleForward(XMFLOAT4 & HipsJoint)
	{
		auto V1 = SimpleMath::Matrix::CreateFromQuaternion(HipsJoint).Forward();
		auto V2 = SimpleMath::Vector3(0, 0, -1);

		TDeltaRotation(V1, V2).Log("Simulation::UpdateCapsuleRotation_AlignHipsJointOrintationToCapsuleForward").Add(HipsJoint);
	}
	void UpdateCapsuleRotation_ApplyTargetRootDeltaRotation(char* _animation1, char* _animation2, float BlendTime) //current, prev
	{
		const auto bConditionBackwardIter = ApplyRotationByAnimChange.find(std::pair<char*, char*>(_animation1, _animation2));
		const auto bConditionForwardIter = ApplyRotationByAnimChange.find(std::pair<char*, char*>(_animation2, _animation1));
		if (bConditionForwardIter != ApplyRotationByAnimChange.end() || bConditionBackwardIter != ApplyRotationByAnimChange.end())
		{
			InProgress = true;
			InverseRootTotalRotation = SimpleMath::Quaternion::Identity;

			UpdateCapsuleRotation_AlignHipsJointOrintationToCapsuleForward(GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 1)[64][1]);

			auto& From = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 2, BlendTime)[64][1];
			auto& To = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 1)[64][1];

			auto V1 = SimpleMath::Matrix::CreateFromQuaternion(From).Forward();
			auto V2 = SimpleMath::Matrix::CreateFromQuaternion(To).Forward();

			TDeltaRotation(V1, V2).Log("Simulation::UpdateCapsuleRotation_ApplyTargetRootDeltaRotation").Add(TargetRootDeltaRotation);

			__BlendSetAnimation2RootDeltaRotation(EveAnimationGraph->getAnimationBlend(), TargetRootDeltaRotation);

			PrevRootRotation = To;
		}
	}

	void UpdateCapsuleRotation_SetParams(const SimpleMath::Quaternion TheTargetRootDeltaRotation, const SimpleMath::Quaternion TheTargetOrientation)
	{
		TargetOrientation = TheTargetOrientation;
		TargetRootDeltaRotation = TheTargetRootDeltaRotation;
	}

	void UpdateCapsuleRotation(Capsule& capsule)
	{
		//frame++;

		static bool bInit = false;
		if (!bInit)
		{
			bInit = true;

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Hanging_Idle_With_Leg", "Climb_Look_Idle_L"));//c second на first конкатенирует InverseTotalRootRotation
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Hanging_Idle_With_Leg", "Climb_Look_Idle_R"));
			
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Climb_Look_Idle_L", "JumpFromWall"));
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Climb_Look_Idle_R", "JumpFromWall"));

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Jump_To_Hang_With_Leg", "BallisticFly"));
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Jump_To_Hang_WithOut_Leg", "BallisticFly"));

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("LeftShimmy", "Left_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Left_Edge_Horizontal_Jump", "Left_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Jump_To_Hang_With_Leg", "Left_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("RightShimmy", "Right_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation
			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Right_Edge_Horizontal_Jump", "Right_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation

			ApplyRotationByAnimChange.insert(std::pair<char*, char*>("Jump_To_Hang_With_Leg", "Right_Edge_Horizontal_Jump"));//c second на first конкатенирует InverseTotalRootRotation
		}

		////if (CurrentAnimationName != EveAnimationGraph->getAnimationName())// || (!EveAnimationGraph->IsBlendActivated() && SignalResetRotations))
		////{
		////PrevRootRotation = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64][1];
		////CurrentAnimationName = EveAnimationGraph->getAnimationName();

		//if (!EveAnimationGraph->IsBlendActivated())
		//{
		//	SignalResetRotations = false;
		//	SimpleMath::Quaternion TotalRootRotation;
		//	InverseTotalRootRotation.Inverse(TotalRootRotation);
		//	PrevRootRotation = SimpleMath::Quaternion::Concatenate(
		//		TotalRootRotation,
		//		PrevRootRotation
		//	);
		//	EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64][1] = PrevRootRotation;
		//	sprintf(DebugBuffer, "(%d)Simulation::UpdateCapsuleRotation SignalResetRotations Proccessed\n", frame); Debug();
		//}

		////InverseTotalRootRotation = SimpleMath::Quaternion::Identity;
		////}

		const auto bConditionForwardIter = ApplyRotationByAnimChange.find(std::pair<char*, char*>(EveAnimationGraph->getPrevAnimationName(), EveAnimationGraph->getAnimationName()));
		const auto bConditionBackwardIter = ApplyRotationByAnimChange.find(std::pair<char*, char*>(EveAnimationGraph->getAnimationName(), EveAnimationGraph->getPrevAnimationName()));

		if (bConditionForwardIter != ApplyRotationByAnimChange.end() || bConditionBackwardIter != ApplyRotationByAnimChange.end())
		{
			if (EveAnimationGraph->getAnimationBlend()->isPlaying())
			{
				//sprintf(DebugBuffer, "Simulation::UpdateCapsuleRotation InProgress \n"); Debug();
				//if (!EveAnimationGraph->_getAnimation()->isPlaying())
				//{
				//	sprintf(DebugBuffer, "UpdateCapsuleRotation PrevRootRotation %f %f %f\n", Quat().decompose0(PrevRootRotation).yaw0); Debug();
				//GetStartYaw_PrevRoot_CurrentRoot(CurrentAnimBlending, StartYaw, PrevRootRotation, CurrentRootRotation);
				//	return;
				//}
				{
					JointSQT& RootJoint = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64];
					auto RootRotation = SimpleMath::Quaternion(RootJoint[1]);
					//auto PrevRootRotation = SimpleMath::Matrix::CreateFromQuaternion(PrevRootRotation);
					//PrevRootRotation = RootJoint[1];

					auto V1 = SimpleMath::Matrix::CreateFromQuaternion(PrevRootRotation).Forward();
					auto V2 = SimpleMath::Matrix::CreateFromQuaternion(RootRotation).Forward(); 
					PrevRootRotation = RootRotation;

					TDeltaRotation DeltaRotation(V1, V2);

					DeltaRotation.Add(capsule.orientation);
					DeltaRotation.Substruct(InverseRootTotalRotation);

					RootJoint[1] = SimpleMath::Quaternion::Concatenate(InverseRootTotalRotation, RootJoint[1]);
					(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();

					//if (abs(RotationAngle) > 0.000001f)
					//{
						//sprintf(DebugBuffer, "(%d)Simulation::UpdateCapsuleRotation RotationAngle=%f RotationAxis=(%f,%f,%f)\n", frame, RotationAngle, RotationAxis.x, RotationAxis.y, RotationAxis.z); Debug();
					//}
					//	CurrentFrame.Right().x, CurrentFrame.Right().y, CurrentFrame.Right().z,
					//	CurrentFrame.Up().x, CurrentFrame.Up().y, CurrentFrame.Up().z,
					//	CurrentFrame.Backward().x, CurrentFrame.Backward().y, CurrentFrame.Backward().z); Debug();
				}

				//return;
				//Quat Q;
				//JointSQT& RootJoint = GetRootJoint();
				//Q.decompose(0.f, PrevRootRotation, RootJoint[1]);
				//PrevRootRotation = RootJoint[1];

				//capsule.orientation = SimpleMath::Quaternion::Concatenate(Q.DeltaRotation, capsule.orientation);
				//RootJoint[1] = Q.RootRotation;

				//(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();
				//sprintf(DebugBuffer, "UpdateCapsuleRotation %f %f %f [Anim1 Yaw %f] [BlendK %f]\n", Q.yaw0, Q.yaw1, Q.yaw1 - Q.yaw0, Quat().decompose0(GetAnimation1RootRotation()).yaw0, __GetBlendK()); Debug();
				{
					auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

					auto CapsulForwward_WS = SimpleMath::Matrix::CreateFromQuaternion(capsule.orientation).Right();
					CapsulForwward_WS.y = 0.f;
					CapsulForwward_WS.Normalize();
					const auto CapsulForwward_Alfa = atan2(CapsulForwward_WS.z, CapsulForwward_WS.x) * 180.f / XM_PI;

					auto HipsForwward_WS = ((*GetSkeletonMatrix(Eve->skelet, 64)) * FromModelSpaceToWorld).Forward();
					HipsForwward_WS.y = 0.f;
					HipsForwward_WS.Normalize();
					const auto HipsForwward_Alfa = atan2(HipsForwward_WS.z, HipsForwward_WS.x) * 180.f / XM_PI;

					static auto prev_CapsulForwward_Alfa = CapsulForwward_Alfa;
					static auto prev_HipsForwward_Alfa = HipsForwward_Alfa;
					if (fabs(prev_CapsulForwward_Alfa - CapsulForwward_Alfa) > 0.00001f || fabs(prev_HipsForwward_Alfa - HipsForwward_Alfa) > 0.00001f)
					{
						sprintf(DebugBuffer, "Simulation::UpdateCapsuleRotation InProgress %f(%f)\n", CapsulForwward_Alfa, HipsForwward_Alfa); Debug();
						prev_CapsulForwward_Alfa = CapsulForwward_Alfa;
						prev_HipsForwward_Alfa = HipsForwward_Alfa;
					}
				}
			}
			else if (InProgress)
			{
				InProgress = false;

				//SimpleMath::Quaternion TotalRootRotation;
				//InverseTotalRootRotation.Inverse(TotalRootRotation);
				//JointSQT& RootJoint = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64];
				//RootJoint[1] = SimpleMath::Quaternion::Concatenate(InverseTotalRootRotation, RootJoint[1]);//этот кадр еще содержит не единичный AnimRootRotation, так что корректируем орентацию Hips
				//(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();
				
				__AnimResampleCurrentRootRotation(EveAnimationGraph->getPlayingAnimation());

				EveAnimationGraph->getAnimationBlend()->CurrentJoints[64][1] = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64][1];

				__AnimSetRootDeltaRotation(EveAnimationGraph->getPlayingAnimation(), SimpleMath::Quaternion::Identity);

				JointSQT& RootJoint = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64];
				
				(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();

				capsule.orientation = TargetOrientation;

				{
					auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

					auto CapsulForwward_WS = SimpleMath::Matrix::CreateFromQuaternion(capsule.orientation).Right();
					CapsulForwward_WS.y = 0.f;
					CapsulForwward_WS.Normalize();
					const auto CapsulForwward_Alfa = atan2(CapsulForwward_WS.z, CapsulForwward_WS.x) * 180.f / XM_PI;

					auto HipsForwward_WS = ((*GetSkeletonMatrix(Eve->skelet, 64)) * FromModelSpaceToWorld).Forward();
					HipsForwward_WS.y = 0.f;
					HipsForwward_WS.Normalize();
					const auto HipsForwward_Alfa = atan2(HipsForwward_WS.z, HipsForwward_WS.x) * 180.f / XM_PI;

					//static auto prev_CapsulForwward_Alfa = CapsulForwward_Alfa;
					//static auto prev_HipsForwward_Alfa = HipsForwward_Alfa;
					//if (fabs(prev_CapsulForwward_Alfa - CapsulForwward_Alfa) > 0.00001f || fabs(prev_HipsForwward_Alfa - HipsForwward_Alfa) > 0.00001f)
					{
						sprintf(DebugBuffer, "Simulation::UpdateCapsuleRotation Done %f(%f)\n", CapsulForwward_Alfa, HipsForwward_Alfa); Debug();
						//prev_CapsulForwward_Alfa = CapsulForwward_Alfa;
						//prev_HipsForwward_Alfa = HipsForwward_Alfa;
					}
				}
			}
		}
	}
}