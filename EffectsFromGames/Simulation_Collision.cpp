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
	void Collision(Capsule& capsule, SimpleMath::Vector3 deltaTranslation, SimpleMath::Quaternion deltaRotation)
	{
		EnsureInits();
		ExtraHitInfos.clear();

		capsule.orientation = SimpleMath::Quaternion::Concatenate(deltaRotation, capsule.orientation);

		float t = 0.f;

		if (state_walking_on_your_feet || state_idle)
		{
			physCollision(capsule, gravitation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, &ExtraHitInfos);

			capsule.origin += t*gravitation;
			//sprintf(DebugBuffer, "Collision N %f\n", t); Debug();

			state_falling = t == 1.f;

			for (int i = 0; !state_falling && i < ExtraHitInfos.size(); i++)
			{
				//sprintf(DebugBuffer, "physCollision(0) N %d[%d] t=%f hitNormal=(%f, %f, %f) hitPoint=(%f, %f, %f) objectName=%s\n", i, ExtraHitInfos.size(), 
				//	ExtraHitInfos[i].t,
				//	ExtraHitInfos[i].hitNormal.x, ExtraHitInfos[i].hitNormal.y, ExtraHitInfos[i].hitNormal.z,
				//	ExtraHitInfos[i].hitPoint.x, ExtraHitInfos[i].hitPoint.y, ExtraHitInfos[i].hitPoint.z,
				//	ExtraHitInfos[i].objectName.data()); Debug();
				CollisionImpactPoint = ExtraHitInfos[i].hitPoint;
				CollisionImpactNormal = ExtraHitInfos[i].hitNormal;
				CollisionObjectName = ExtraHitInfos[i].objectName;
				CollisionObjectType = ExtraHitInfos[i].objectType;

				auto SlideDeltaTranslation = physGetSlidingVector(deltaTranslation, 1.0f, CollisionImpactNormal);
				physCollision(capsule, SlideDeltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);

				if (0.f < t)
				{
					capsule.origin += t*SlideDeltaTranslation;
					//sprintf(DebugBuffer, "physCollision(1) %f (%f, %f, %f) %s\n", t, CollisionImpactNormal.x, CollisionImpactNormal.y, CollisionImpactNormal.z, CollisionObjectName.data()); Debug();

					if (t < 1.0)
					{
						SlideDeltaTranslation = physGetSlidingVector(SlideDeltaTranslation, 1.0f - t, CollisionImpactNormal);
						physCollision(capsule, SlideDeltaTranslation, t, CollisionImpactPoint, CollisionImpactNormal, CollisionObjectName, CollisionObjectType, nullptr);

						capsule.origin += t*SlideDeltaTranslation;
						//sprintf(DebugBuffer, "physCollision(2) %f (%f, %f, %f) %s\n", t, CollisionImpactNormal.x, CollisionImpactNormal.y, CollisionImpactNormal.z, CollisionObjectName.data()); Debug();
					}

					break;
				}
			}
		}
		else if ((!state_hanging && state_falling_and_hang_on) || state_falling || state_jump || state_jump_from_wall || state_ballistic_fly_to_target)
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
		}

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