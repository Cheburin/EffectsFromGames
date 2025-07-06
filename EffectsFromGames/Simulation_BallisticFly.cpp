#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

void SetBallisticTrajectoryParams(AnimationBase * BallisticAnimation, SimpleMath::Vector3 & ballisticG, SimpleMath::Vector3 & ballisticInitialVelocity);
SimpleMath::Vector3 GetBallisticAnimationHandLocation(AnimationBase * BallisticAnimation, int CatherHandIndex);
void SetBallisticAnimationCatherHand(AnimationBase * Animation, int CatherHandIndex);

extern bool state_hanging;

extern bool state_ballistic_fly_to_target;

extern bool state_BallisticFly_to_HangingIdle;
extern bool state_BallisticFly_to_HangingIdleWithOutLeg;

//extern SimpleMath::Vector3 state_ballistic_fly_Start_Location;
extern SimpleMath::Vector3 state_ballistic_fly_Start_World_Location;
//extern SimpleMath::Vector3 state_ballistic_fly_Finish_Location;
extern SimpleMath::Vector3 state_ballistic_fly_Finish_World_Location;

extern Box state_hanging_Ledge_Box;
extern std::string state_hanging_Ledge_Name;
extern std::string start_hanging_Ledge_Name;
extern int start_hanging_Ledge_BoxIndex;
extern int state_hanging_Ledge_BoxIndex;

#undef min // use __min instead
#undef max // use __max instead

namespace
{
	SimpleMath::Vector3 normalize(SimpleMath::Vector3 v)
	{
		v.Normalize();

		return v;
	}

	int GetHandIndexByLedgeAndCharactersForwards(SimpleMath::Vector3 LedgeForward, const SimpleMath::Vector3& CharacterForward)
	{
		if (CharacterForward.Dot(LedgeForward) < 0.f)
		{
			LedgeForward *= -1.f;
		};
		return CharacterForward.Cross(LedgeForward).y > 0.f ? 0 : 1;
	}
}

void GetBallisticTrajectoryParams(const float Velocity, const SimpleMath::Vector3& forward, const SimpleMath::Vector3& Start, const SimpleMath::Vector3& Finish, SimpleMath::Vector3 & ballisticG, SimpleMath::Vector3 & ballisticInitialVelocity)
{
	SimpleMath::Vector3 ProjectOnGround(const SimpleMath::Vector3 & a);

	//auto InvertEveSkinnedModel = FromModelSpaceToWorld.Invert();

	float y0 = Start.y;
	float yPeak = Start.y + 2.5f;
	float yEnd = Finish.y;

	auto D = (Finish - Start);
	auto t = ProjectOnGround(D).Length() / Velocity;

	ballisticG = ( 4.0f*(y0 - 2.0f*yPeak + yEnd) / (t*t) ) * SimpleMath::Vector3(0, 1, 0);
	ballisticInitialVelocity = Velocity * forward + (-1.0f*(3.0f*y0 - 4.0f*yPeak + yEnd) / t) * SimpleMath::Vector3(0, 1, 0);

	//ballisticG = SimpleMath::Vector3::TransformNormal(ballisticG, InvertEveSkinnedModel);
	//ballisticInitialVelocity = SimpleMath::Vector3::TransformNormal(ballisticInitialVelocity, InvertEveSkinnedModel);
}

namespace Simulation
{
	void EnableBallisticFlyIfNeeded(const SimpleMath::Matrix& FromModelSpaceToWorld, const SimpleMath::Vector3& origin, const SimpleMath::Vector3& forward)
	{
		auto D = std::numeric_limits<float>::max();
		auto N = SimpleMath::Vector3(0, 1, 0).Cross(forward);
		auto L = vec4(N, -origin.Dot(N));

		SimpleMath::Vector3 S, V;

		{
			state_BallisticFly_to_HangingIdle = false;
			state_BallisticFly_to_HangingIdleWithOutLeg = false;

			if (!state_hanging)
			{
				start_hanging_Ledge_Name = "";
				start_hanging_Ledge_BoxIndex - 1;
			}
		}

		if (origin.y > 1)
		{
			for (auto Ledge_Iter = GWorld.Ledges.begin(); Ledge_Iter != GWorld.Ledges.end(); Ledge_Iter++)
			{
				const auto & Ledge_Boxes = Ledge_Iter->second.Boxes;
				for (auto Ledge_Box_Iter = Ledge_Boxes.begin(); Ledge_Box_Iter != Ledge_Boxes.end(); Ledge_Box_Iter++)
				{
					SimpleMath::Vector3 _S = Ledge_Box_Iter->origin;
					SimpleMath::Vector3 _V = Ledge_Box_Iter->worldForward;
					SimpleMath::Vector3 _N = Ledge_Box_Iter->worldBackSide;

					auto t = -(L.Dot(vec4(_S, 1)) / L.Dot(vec4(_V, 0)));

					if (-0.48f < t && t < 0.48f && (origin - _S).Dot(_N) > 0.0f && (forward).Dot(_N) < 0.0f)
					{
						const auto _D = forward.Dot(_S + t*_V - origin);

						if (_D < D)
						{
							D = _D;
							S = _S;
							V = _V;
						}
					}
				}
			}
		}

		if (D != std::numeric_limits<float>::max())
		{
			if (state_ballistic_fly_to_target = S.y > origin.y)
			{
				AnimationBase * BallisticAnimation = EveAnimationGraph->getAnimation("BallisticFly");

				SetBallisticAnimationCatherHand(BallisticAnimation, GetHandIndexByLedgeAndCharactersForwards(V, forward));

				state_ballistic_fly_Start_World_Location = SimpleMath::Vector3::Transform(GetBallisticAnimationHandLocation(BallisticAnimation, GetHandIndexByLedgeAndCharactersForwards(V, forward)), FromModelSpaceToWorld);
				{
					auto L2 = vec4(N, -state_ballistic_fly_Start_World_Location.Dot(N));
					auto t2 = -(L2.Dot(vec4(S, 1)) / L2.Dot(vec4(V, 0)));
					state_ballistic_fly_Finish_World_Location = S + t2*V;
				}

				SimpleMath::Vector3 ballisticG;
				SimpleMath::Vector3 ballisticInitialVelocity;
				GetBallisticTrajectoryParams(15.f, forward, state_ballistic_fly_Start_World_Location, state_ballistic_fly_Finish_World_Location, ballisticG, ballisticInitialVelocity);
				SetBallisticTrajectoryParams(BallisticAnimation, ballisticG, ballisticInitialVelocity);

				//state_ballistic_fly_Start_Location = SimpleMath::Vector3::Transform(state_ballistic_fly_Start_Location, FromModelSpaceToWorld.Invert());
				//state_ballistic_fly_Finish_Location = SimpleMath::Vector3::Transform(state_ballistic_fly_Finish_Location, FromModelSpaceToWorld.Invert());
			}
		}
	}

	void StartBallisticFly(const SimpleMath::Matrix& FromModelSpaceToWorld, const SimpleMath::Vector3& CapsuleForward)
	{
		////sprintf(DebugBuffer, "capsuleDirection %f %f %f\n", capsuleDirection.x, capsuleDirection.y, capsuleDirection.z); Debug();
		static bool deferred_state_ballistic_fly_to_target = false;
		if (EveAnimationGraph->getAnimationName() == "jump_forward")
		{
			if (EveAnimationGraph->getAnimation<Animation>()->getLocTime() == 0.0)
			{
				const auto EveOrigin = FromModelSpaceToWorld.Translation();

				//SimpleMath::Vector3 sc, tr;
				//SimpleMath::Quaternion orientation;
				//SimpleMath::Matrix(FromModelSpaceToWorld).Decompose(sc, orientation, tr);
				
				const auto EveDirection = CapsuleForward;// SimpleMath::Vector3::Transform(SimpleMath::Vector3(1, 0, 0), orientation);

				EnableBallisticFlyIfNeeded(FromModelSpaceToWorld, EveOrigin, EveDirection);

				deferred_state_ballistic_fly_to_target = state_ballistic_fly_to_target;
				state_ballistic_fly_to_target = false;
				//sprintf(DebugBuffer, "evalBallisticPathStartVelocity %f %f %f %f %f %f %f %f %f\n", ballisticStart.x, ballisticStart.y, ballisticStart.z, ballisticFinish.x, ballisticFinish.y, ballisticFinish.z, capsuleDirection.x, capsuleDirection.y, capsuleDirection.z); Debug();
			}
			if (EveAnimationGraph->getAnimation<Animation>()->getLocTime() > 0.07)
			{
				state_ballistic_fly_to_target = deferred_state_ballistic_fly_to_target;
			}
		}
	}

	void FinishBallisticFly(const SimpleMath::Matrix& CapsuleSystem, const float CapsuleAB, const float CapsuleR)
	{
		if (state_ballistic_fly_to_target && state_hanging && (start_hanging_Ledge_Name != state_hanging_Ledge_Name || start_hanging_Ledge_BoxIndex != state_hanging_Ledge_BoxIndex))
		{
			state_BallisticFly_to_HangingIdle = false;
			state_BallisticFly_to_HangingIdleWithOutLeg = true;

			const auto MiddleCapsuleRayOrigin = (CapsuleR + 0.5f*CapsuleAB) * SimpleMath::Vector3(0, 1, 0);
			const auto MiddleCapsuleRaySystem = SimpleMath::Matrix::CreateTranslation(MiddleCapsuleRayOrigin) * CapsuleSystem;

			const auto TrS = SimpleMath::Vector3::Transform(SimpleMath::Vector3(0,0,0), MiddleCapsuleRaySystem);
			const auto TrV = -5.f*CapsuleR*normalize(state_hanging_Ledge_Box.worldBackSide);

			sprintf(DebugBuffer, "FinishBallisticFly Check(0)\n"); Debug();
			for (const auto& LedgeOwner : GWorld.Ledges[state_hanging_Ledge_Name].Owners)
			{
				if (LedgeOwner.Intersect(TrS, TrV))
				{
					std::swap(state_BallisticFly_to_HangingIdle, state_BallisticFly_to_HangingIdleWithOutLeg);
					sprintf(DebugBuffer, "FinishBallisticFly LedgeOwner.Intersect\n"); Debug();
					break;
				}
			}
			if (state_BallisticFly_to_HangingIdle || state_BallisticFly_to_HangingIdleWithOutLeg)
			{
				auto V1 = GWorld.Capsules["eve"].getMatrix().Right();
				auto V2 = -state_hanging_Ledge_Box.worldBackSide;
				TDeltaRotation DeltaRotation(V1, V2);
				Simulation::UpdateCapsuleRotation_SetParams(DeltaRotation.Delta, state_hanging_Ledge_Box.GetPawnOrientation());
				sprintf(DebugBuffer, "FinishBallisticFly Simulation::UpdateCapsuleRotation_SetParams TargetRootDeltaRotation Angle %f \n", (DeltaRotation.RotationAngle/XM_PI)*180.f); Debug();
			}
			sprintf(DebugBuffer, "FinishBallisticFly Check(1)\n"); Debug();
			////перпендикуляр к уступу(на который прыгаем,уступ пока один) по направлению прышка(пока можем пригать с одного направления)
			////desiredDirectionToTarget = SimpleMath::Vector3(-1.f, 0.f, 0.f);
			state_ballistic_fly_to_target = false;
		}
		//sprintf(DebugBuffer, "set hangOnObject %s %s\n", state_hang_on_hand_name, state_hang_on_object_name); Debug();
		//break;
	}
}