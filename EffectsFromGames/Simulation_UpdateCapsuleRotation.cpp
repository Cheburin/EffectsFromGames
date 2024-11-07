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
	std::set<char*> AnimSet;
	std::set<std::pair<char*, char*>> ApplyInverseRotationByAnimChange;

	SimpleMath::Quaternion PrevRootRotation;
	SimpleMath::Quaternion InverseTotalRootRotation;

	char* CurrentAnimationName = nullptr;

	//bool SignalResetRotations = false;

	int frame = 0;
	//SimpleMath::Vector3 GetNormalizedVector(SimpleMath::Vector3 V1)
	//{
	//	V1.Normalize();
	//	return V1;
	//}
}

void GetStartYaw_PrevRoot_CurrentRoot(AnimationBase* blend, float& StartYaw, SimpleMath::Quaternion &PrevRootRotation, JointSQT* &CurrentRootRotation);
SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);
std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index);

namespace Simulation
{
	SimpleMath::Quaternion UpdateCapsuleRotation_GetSubstructedRootRotation(char* _animation1, char* _animation2)
	{
		const auto bConditionIter = ApplyInverseRotationByAnimChange.find(std::pair<char*, char*>(_animation1, _animation2));
		if (bConditionIter != ApplyInverseRotationByAnimChange.end())
		{
			//SignalResetRotations = true;
			sprintf(DebugBuffer, "Simulation::UpdateCapsuleRotation GetSubstructedRootRotation\n"); Debug();

			SimpleMath::Quaternion DeltaCorrection = SimpleMath::Quaternion::Identity;
			auto Rotation1 = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 2)[64][1];
			auto Rotation2 = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 1)[64][1];
			auto V1 = SimpleMath::Matrix::CreateFromQuaternion(Rotation1).Forward(); V1.y = 0; V1.Normalize();
			auto V2 = SimpleMath::Matrix::CreateFromQuaternion(Rotation2).Forward(); V2.y = 0; V2.Normalize();
			auto RotationAxis = V1.Cross(V2);
			auto RotationAngle = atan2(RotationAxis.Length(), V1.Dot(V2));
			if (!XMVector3Equal(RotationAxis, XMVectorZero()))
			{
				sprintf(DebugBuffer, "Simulation::UpdateCapsuleRotation GetSubstructedRootRotation DeltaCorrection On Angle %f\n", RotationAngle); Debug();
				DeltaCorrection = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, RotationAngle);
			}

			return SimpleMath::Quaternion::Concatenate(
				InverseTotalRootRotation,
				DeltaCorrection
			);
		}
		return SimpleMath::Quaternion::Identity;
	}

	void SetInverseTotalRootRotation(const SimpleMath::Quaternion& Rotation)
	{
		InverseTotalRootRotation = Rotation;
	}

	void UpdateCapsuleRotation(Capsule& capsule)
	{
		frame++;

		static bool bInit = false;
		if (!bInit)
		{
			bInit = true;
			AnimSet.insert("JumpFromWall");
			AnimSet.insert("Climb_Look_Idle_L");
			AnimSet.insert("Climb_Look_Idle_R");
			AnimSet.insert("hangingIdle");
			AnimSet.insert("Jump_To_Hang");

			ApplyInverseRotationByAnimChange.insert(std::pair<char*, char*>("hangingIdle", "Climb_Look_Idle_L"));
			ApplyInverseRotationByAnimChange.insert(std::pair<char*, char*>("hangingIdle", "Climb_Look_Idle_R"));

			ApplyInverseRotationByAnimChange.insert(std::pair<char*, char*>("hangingIdle", "BallisticFly"));
			ApplyInverseRotationByAnimChange.insert(std::pair<char*, char*>("Jump_To_Hang", "BallisticFly"));
		}

		if (CurrentAnimationName != EveAnimationGraph->getAnimationName())// || (!EveAnimationGraph->IsBlendActivated() && SignalResetRotations))
		{
			PrevRootRotation = EveAnimationGraph->getPlayingAnimation()->CurrentJoints[64][1];
			CurrentAnimationName = EveAnimationGraph->getAnimationName();

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

			InverseTotalRootRotation = SimpleMath::Quaternion::Identity;
		}

		if (AnimSet.find(EveAnimationGraph->getAnimationName()) != AnimSet.end())
		{
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

				auto V1 = SimpleMath::Matrix::CreateFromQuaternion(PrevRootRotation).Forward(); V1.y = 0; V1.Normalize();
				auto V2 = SimpleMath::Matrix::CreateFromQuaternion(RootRotation).Forward(); V2.y = 0; V2.Normalize();
				PrevRootRotation = RootRotation;

				auto RotationAxis = V1.Cross(V2);
				auto RotationAngle = atan2(RotationAxis.Length(), V1.Dot(V2));

				if (!XMVector3Equal(RotationAxis, XMVectorZero()))
				{
					capsule.orientation = SimpleMath::Quaternion::Concatenate(
						SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, RotationAngle),
						capsule.orientation
					);

					InverseTotalRootRotation = SimpleMath::Quaternion::Concatenate(
						SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, -RotationAngle),
						InverseTotalRootRotation
					);
				}

				RootJoint[1] = SimpleMath::Quaternion::Concatenate(InverseTotalRootRotation, RootJoint[1]);
				(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();

				if (abs(RotationAngle) > 0.000001f)
				{
					//sprintf(DebugBuffer, "(%d)Simulation::UpdateCapsuleRotation RotationAngle=%f RotationAxis=(%f,%f,%f)\n", frame, RotationAngle, RotationAxis.x, RotationAxis.y, RotationAxis.z); Debug();
				}
				//	CurrentFrame.Right().x, CurrentFrame.Right().y, CurrentFrame.Right().z,
				//	CurrentFrame.Up().x, CurrentFrame.Up().y, CurrentFrame.Up().z,
				//	CurrentFrame.Backward().x, CurrentFrame.Backward().y, CurrentFrame.Backward().z); Debug();
			}

			return;
			//Quat Q;
			//JointSQT& RootJoint = GetRootJoint();
			//Q.decompose(0.f, PrevRootRotation, RootJoint[1]);
			//PrevRootRotation = RootJoint[1];

			//capsule.orientation = SimpleMath::Quaternion::Concatenate(Q.DeltaRotation, capsule.orientation);
			//RootJoint[1] = Q.RootRotation;

			//(*GetSkeletonMatrix(Eve->skelet, 64)) = RootJoint.matrix();
			//sprintf(DebugBuffer, "UpdateCapsuleRotation %f %f %f [Anim1 Yaw %f] [BlendK %f]\n", Q.yaw0, Q.yaw1, Q.yaw1 - Q.yaw0, Quat().decompose0(GetAnimation1RootRotation()).yaw0, __GetBlendK()); Debug();
		}
	}
}