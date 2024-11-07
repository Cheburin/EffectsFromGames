#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern bool state_climbing;
extern bool state_force_kneeling;

extern SimpleMath::Vector3 EveHeadLocation;

SimpleMath::Vector3 AutoKneeling_Debug_Location;

void physCollision(const Capsule& ca, const SimpleMath::Vector3 & worldDistance, float & T, SimpleMath::Vector3& impactPoint, SimpleMath::Vector3& impactNormal, std::string& objectName, int& objectType, std::vector<HitInfo2>* HitInfos);

extern std::vector<Capsule> DebugCapsule;

extern bool simulation_state_manual_control;

extern bool state_falling;

namespace Simulation
{
	void AutoKneeling(const SimpleMath::Matrix& FromModelSpaceToWorld)
	{
		SimpleMath::Vector3 prev_HeadLocation = EveHeadLocation;
		SimpleMath::Vector3 current_HeadLocation = AutoKneeling_Debug_Location = GetHeadLocation(FromModelSpaceToWorld);

		auto TargetHeadUp = 4.69f + 2.f;

		float t;

		Capsule capsule;
		capsule.ab = 0.f;
		capsule.orientation = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::Identity);

		SimpleMath::Vector3 impactPoint, impactNormal;

		if (!state_force_kneeling)
		{
			capsule.r = 0.25f;
			capsule.origin = prev_HeadLocation - SimpleMath::Vector3(0.0f, capsule.r, 0.0f);

			int objectType;
			std::string objectName;
			auto Distance = current_HeadLocation - prev_HeadLocation;
			if (Distance.Length() > 0.0f)
			{
				physCollision(capsule, Distance, t, impactPoint, impactNormal, objectName, objectType, nullptr);
				if (t != 1.0f)// && impactNormal.y < -0.6f)
				{
					sprintf(DebugBuffer, "state_force_kneeling true %f %s [%d]\n", t, &objectName[0], state_falling); Debug();
					state_force_kneeling = true;
				}
			}
		}
		else
		{
			SimpleMath::Vector3 Origin = FromModelSpaceToWorld.Translation();
			auto CurHeadUp = (current_HeadLocation - Origin).y;

			capsule.r = 1.f;
			capsule.origin = (CurHeadUp - 2.f*capsule.r)*SimpleMath::Vector3(0.0f, 1.f, 0.0f) + Origin;

			int objectType;
			std::string objectName;
			auto Distance = (TargetHeadUp - CurHeadUp);
			if (Distance > 0.0f)
			{
				physCollision(capsule, Distance*SimpleMath::Vector3(0.0f, 1.f, 0.0f), t, impactPoint, impactNormal, objectName, objectType, nullptr);
				if (t == 1.0f)
				{
					sprintf(DebugBuffer, "state_force_kneeling false %f [%d]\n", t, state_falling); Debug();
					state_force_kneeling = false;
				}
			}
		}

		EveHeadLocation = current_HeadLocation;
	}
}