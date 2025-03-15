#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern float eveCurrentRotation;
extern bool state_ballistic_fly_to_target;
extern bool state_falling;
extern bool state_falling_and_hang_on;
extern bool	state_jump_from_wall;
extern bool	state_jump;
extern bool state_climbing;
extern bool state_into_water;
extern bool state_hanging;
extern bool state_walking_on_your_feet;
extern bool state_idle;
extern SimpleMath::Vector3 gravitation;
extern bool state_movement_is_obstructed;
extern SimpleMath::Vector3 state_hanging_Hand_Location;

void physCollision(const Capsule& ca, const SimpleMath::Vector3 & worldDistance, float & T, SimpleMath::Vector3& impactPoint, SimpleMath::Vector3& impactNormal, std::string& objectName, int& objectType, std::vector<HitInfo2>* HitInfos);
SimpleMath::Vector3 physGetSlidingVector(const SimpleMath::Vector3 & worldDistance, float min_t, SimpleMath::Vector3& impactNormal);

int CollisionObjectType = 0;
std::string CollisionObjectName = "";
SimpleMath::Vector3 CollisionImpactNormal = SimpleMath::Vector3::Zero;
SimpleMath::Vector3 CollisionImpactPoint = SimpleMath::Vector3::Zero;

//!state_into_water &&
//!state_hanging &&
//!state_falling_and_hang_on &&
//!state_jump &&
//!state_climbing &&
//!state_ballistic_fly_to_target

namespace Simulation
{
	std::vector<HitInfo2> ExtraHitInfos;
	bool init = false;
	void EnsureInits()
	{
		if (!init)
		{
			init = true;
			ExtraHitInfos.reserve(100);
		}
	}
	float Move(Capsule& capsule, SimpleMath::Vector3 SlideDeltaTranslation)
	{
		capsule.origin += SlideDeltaTranslation;

		return 1.f;
	}
	float BlockingMove(Capsule& capsule, SimpleMath::Vector3 SlideDeltaTranslation, std::vector<HitInfo2>* HitInfos = nullptr)
	{
		float t = 0.f;

		physCollision(capsule, SlideDeltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, HitInfos);
		capsule.origin += t*SlideDeltaTranslation;

		return t;
	}
	float Slide(Capsule& capsule, SimpleMath::Vector3 SlideDeltaTranslation, bool needlog = false)
	{
		float t = 0.f, s = 0.f;

		physCollision(capsule, SlideDeltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);

		//if (needlog && CollisionImpactNormal.z ==-1.f) return 0.f;
		//if (0.f < t)
		{
			capsule.origin += t*SlideDeltaTranslation;

			if(needlog)
			{
				sprintf(DebugBuffer, "T collision zero %f hitNormal=(%f, %f, %f)\n", t, CollisionImpactNormal.x, CollisionImpactNormal.y, CollisionImpactNormal.z); Debug();
			}

			if (t < 1.f)
			{
				SlideDeltaTranslation = physGetSlidingVector(SlideDeltaTranslation, 1.0f - t, CollisionImpactNormal);

				physCollision(capsule, SlideDeltaTranslation, s, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);
				if (needlog)
				{
					sprintf(DebugBuffer, "S collision zero %f SlideDeltaTranslation=(%f, %f, %f)\n", s, SlideDeltaTranslation.x, SlideDeltaTranslation.y, SlideDeltaTranslation.z); Debug();
				}
				//sprintf(DebugBuffer, "collision zero %f\n", s); Debug();
				capsule.origin += s*SlideDeltaTranslation;
			}
		}

		return t + s*(1.0f - t);
	}
	/*float Slide2(Capsule& capsule, SimpleMath::Vector3 SlideDeltaTranslation)
	{
		float t = 0.f, s = 0.f;

		physCollision(capsule, SlideDeltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);
		capsule.origin += t*SlideDeltaTranslation;
		//sprintf(DebugBuffer, "physCollision(1) %f (%f, %f, %f) %s\n", t, CollisionImpactNormal.x, CollisionImpactNormal.y, CollisionImpactNormal.z, CollisionObjectName.data()); Debug();

		if (0.f < t && t < 1.f)
		{
			SlideDeltaTranslation = physGetSlidingVector(SlideDeltaTranslation, 1.0f - t, CollisionImpactNormal);

			physCollision(capsule, SlideDeltaTranslation, s, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);
			capsule.origin += s*SlideDeltaTranslation;
			//sprintf(DebugBuffer, "physCollision(2) %f (%f, %f, %f) %s\n", t, CollisionImpactNormal.x, CollisionImpactNormal.y, CollisionImpactNormal.z, CollisionObjectName.data()); Debug();

			return t + s*(1.0f - t);
		}

		return t;
	}*/
	void Collision(Capsule& capsule, SimpleMath::Vector3 deltaTranslation, SimpleMath::Quaternion deltaRotation)
	{
		EnsureInits();

		capsule.orientation = SimpleMath::Quaternion::Concatenate(deltaRotation, capsule.orientation);

		if (state_hanging)
		{
			Move(capsule, deltaTranslation);
		}
		else
		{
			if (state_walking_on_your_feet || state_idle)
			{
				ExtraHitInfos.clear();

				auto t = BlockingMove(capsule, gravitation, &ExtraHitInfos);
				
				state_falling = t == 1.f;

				if (!state_falling)
				{
					auto t = 1.0f;
					if (ExtraHitInfos.size() == 2){ sprintf(DebugBuffer, "-----------------\n"); Debug(); }
					for (int i = 0; i < ExtraHitInfos.size(); i++)
					{
						if (ExtraHitInfos.size() == 2){
							sprintf(DebugBuffer, "physCollision(0) N %d[%d] t=%f hitNormal=(%f, %f, %f) hitPoint=(%f, %f, %f) objectName=%s\n", i, ExtraHitInfos.size(),
								ExtraHitInfos[i].t,
								ExtraHitInfos[i].hitNormal.x, ExtraHitInfos[i].hitNormal.y, ExtraHitInfos[i].hitNormal.z,
								ExtraHitInfos[i].hitPoint.x, ExtraHitInfos[i].hitPoint.y, ExtraHitInfos[i].hitPoint.z,
								ExtraHitInfos[i].objectName.data()); Debug();
						}
						CollisionImpactPoint = ExtraHitInfos[i].hitPoint;
						CollisionImpactNormal = ExtraHitInfos[i].hitNormal;
						CollisionObjectName = ExtraHitInfos[i].objectName;
						CollisionObjectType = ExtraHitInfos[i].objectType;

						auto SlideDeltaTranslation = physGetSlidingVector(deltaTranslation, t, CollisionImpactNormal);
						t -= t*Slide(capsule, SlideDeltaTranslation, ExtraHitInfos.size() == 2);
						t = min(1.f, max(0.f, t));

						if (ExtraHitInfos.size() == 2){
							sprintf(DebugBuffer, "collision %f\n", t); Debug();
						}
						if (t==0.f)
						{
							break;
						}
					}
					if (ExtraHitInfos.size() == 2){ sprintf(DebugBuffer, "-----------------\n"); Debug(); }
				}
			}
			else if (state_into_water)
			{
				Slide(capsule, deltaTranslation);
			}
			else if (state_falling)
			{
				auto t = BlockingMove(capsule, gravitation);

				if (EveAnimationGraph->getAnimationBlend()->isPlaying() && EveAnimationGraph->getAnimationName() == "ActionPose_Release_And_Go_Down")
				{
					state_hanging_Hand_Location += t*gravitation;
				}

				state_falling = state_falling ? t == 1.f : state_falling;
			}
			else if (state_falling_and_hang_on || state_jump || state_jump_from_wall || state_ballistic_fly_to_target)
			{
				auto t = BlockingMove(capsule, deltaTranslation);

				state_jump_from_wall = state_jump_from_wall ? t == 1.f : state_jump_from_wall;
				state_falling_and_hang_on = state_falling_and_hang_on ? t == 1.f : state_falling_and_hang_on;
			}
			else if (state_climbing)
			{
				Move(capsule, deltaTranslation);
			}
			else
			{
				sprintf(DebugBuffer, "collision unknown state\n"); Debug();
			}
		}
		/*
		else if ((!state_hanging && state_falling_and_hang_on) || state_into_water || state_falling || state_jump || state_jump_from_wall || state_ballistic_fly_to_target)
		{
			if (state_falling)
			{
				deltaTranslation = gravitation;
			}

			physCollision(capsule, deltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);

			capsule.origin += t*deltaTranslation;

			state_falling_and_hang_on = state_falling_and_hang_on ? t == 1.f : state_falling_and_hang_on;

			state_falling = state_falling ? t == 1.f : state_falling;

			state_jump_from_wall = state_jump_from_wall ? t == 1.f : state_jump_from_wall;
		}
		else if (state_hanging || state_climbing)
		{
			capsule.origin += deltaTranslation;
		}
		else
		{
			sprintf(DebugBuffer, "collision unknown state\n"); Debug();
		}*/

		state_movement_is_obstructed = false;

		const auto CapsuleSystem = capsule.getMiddleCapsuleRaySystem();

		const auto TrS = capsule.getMiddleCapsuleRaySystem().Translation();
		const auto TrV = SimpleMath::Vector3::TransformNormal(capsule.getProb(), capsule.getMiddleCapsuleRaySystem());

		for (const auto& namedBoxPair : GWorld.Boxes)
		{
			if (namedBoxPair.second.IntersectOrContain(TrS, TrV))
			{
				state_movement_is_obstructed = true;
				break;
			}
		}

	}
}