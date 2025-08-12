#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern bool state_movement_on_ladder;
extern bool state_ballistic_fly_to_target;
extern bool state_falling;
extern bool state_falling_and_hang_on;
extern bool	state_jump;
extern bool	state_jump_from_wall;
extern bool state_climbing;
extern bool state_into_water;
extern bool state_hanging;

extern SimpleMath::Vector3 cameraForward;
extern SimpleMath::Vector3 cameraRight;
extern SimpleMath::Vector3 cameraPos;
extern SimpleMath::Vector3 cameraTarget;
extern SimpleMath::Vector3 desiredDirectionToTarget;
extern SimpleMath::Vector3 desiredTarget;
extern float eveCurrentRotation;
extern float eveTargetRotation;
extern float eveDeltaRotation;
extern float eveDeltaRotationSpeed;
SimpleMath::Vector3 ProjectOnGround(const SimpleMath::Vector3 & a);
extern SimpleMath::Vector3 gForward;
extern SimpleMath::Vector3 gravitation;
float sign(const float& arg);

namespace Simulation
{
	float eveDeltaRotationSpeed = 0.f;
	float DegFromDir(const SimpleMath::Vector3& Dir)
	{
		return (-atan2(Dir.z, Dir.x) * 180.f) / PI;
	}
	float DegToRad(const float Deg)
	{
		return (Deg * PI) / 180.f;
	}
}

namespace Simulation
{
	void CharacterLerpRotation(float simulationTime, char* CapsuleName, char* ModelTransformName)
	{
		eveCurrentRotation = DegFromDir(GWorld.Capsules[CapsuleName].getMatrix().Right());

		//eveTargetRotation = (-sign(dir.Cross(gForward).y)*acos(dir.Dot(gForward))) * 180.0 / PI;
		//&& (ThirdPersonCameraMode || (FreeCameraMode && input_move.LengthSquared() != 0.0f))//&&
		if (!state_falling && !state_jump && !state_jump_from_wall && !state_hanging && !state_climbing && !state_ballistic_fly_to_target && !state_movement_on_ladder)
		{
			eveTargetRotation = DegFromDir(ProjectOnGround(desiredDirectionToTarget));
			eveDeltaRotation = eveTargetRotation - eveCurrentRotation;

			//find shortest length
			if (fabs(eveDeltaRotation) > 180.f){
				if (eveDeltaRotation > 0.0f)
					eveDeltaRotation -= 360.f;
				else
					eveDeltaRotation += 360.f;
			}
			eveDeltaRotationSpeed = eveDeltaRotation*4.0f;
			// *(fabs(eveDeltaRotation) / 5.0f);
			//if (eveDeltaRotation > 90)eveDeltaRotationSpeed *= 4;
			//if (eveDeltaRotation > 45)eveDeltaRotationSpeed *= 3;
			//if (eveDeltaRotation > 25)eveDeltaRotationSpeed *= 2;

			//sprintf(DebugBuffer, "CharacterLerpRotation %f %f %f %f\n", eveCurrentRotation, eveTargetRotation); Debug();
		}
		{
			if (eveDeltaRotationSpeed != .0f){
				float delta = eveDeltaRotationSpeed * simulationTime;
				if (fabs(eveDeltaRotation) >= fabs(delta))
				{
					eveDeltaRotation -= delta;
					eveCurrentRotation += delta;
					GWorld.Capsules[CapsuleName].orientation = SimpleMath::Quaternion::CreateFromYawPitchRoll(DegToRad(eveCurrentRotation), .0, .0);
				}
				else{
					eveDeltaRotationSpeed = .0f;
					GWorld.Capsules[CapsuleName].orientation = SimpleMath::Quaternion::CreateFromYawPitchRoll(DegToRad(eveCurrentRotation), .0, .0);
				}
			}
		}
	}
}