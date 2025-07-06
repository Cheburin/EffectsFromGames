#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern World GWorld;
extern bool state_force_kneeling;
extern bool state_walking_on_your_feet;
extern bool state_movement_on_ladder;
namespace
{
	SimpleMath::Vector3 CurrentCharacterLocation;
	SimpleMath::Vector3 PrevCharacterLocation;
}

namespace Simulation
{
	void _LadderDetection()
	{
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		const auto LadderOffset = GWorld.Capsules["eve"].r * 1.0f;// (1.f + 1.f / 3.f);
		const auto LadderEnterAlfa = 30.f;
		const auto LadderEnterBound = GWorld.Capsules["eve"].r*1.f / 3.f;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		auto LadderForwardVector = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Forward(); LadderForwardVector.Normalize();
		const auto LadderLocation = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Translation();
		const auto LadderFrontierLocation = LadderLocation + -LadderOffset * LadderForwardVector;
		auto LadderFrontierPlane = vec4(LadderForwardVector, -LadderForwardVector.Dot(LadderFrontierLocation));

		const auto CharacterForwardVector = SimpleMath::Vector3::Transform(SimpleMath::Vector3(1, 0, 0), GWorld.Capsules["eve"].orientation);
		SimpleMath::Vector3 CharacterDeltaTranslation = CurrentCharacterLocation - PrevCharacterLocation;

		float Alfa = acos(CharacterForwardVector.Dot(LadderForwardVector)) * 180.f / PI;

		//sprintf(DebugBuffer, "LadderDetection Alfa %f [%f %f %f]\n", Alfa, CharacterForwardVector.x, CharacterForwardVector.y, CharacterForwardVector.z); Debug();

		if (Alfa < LadderEnterAlfa)
		{
			auto t = -(LadderFrontierPlane.Dot(vec4(PrevCharacterLocation, 1)) / LadderFrontierPlane.Dot(vec4(CharacterDeltaTranslation, 0)));
			//sprintf(DebugBuffer, "LadderDetection t %f\n", t); Debug();
			if (0.0f <= t && t <= 1.f)
			{
				const auto NewCharacterLocation = PrevCharacterLocation + t * CharacterDeltaTranslation;
				auto LadderUpVector = SimpleMath::Matrix(GWorld.WorldTransforms["Ladder"]).Up(); LadderUpVector.Normalize();
				if (LadderForwardVector.Cross(SimpleMath::Vector3(LadderUpVector)).Dot(NewCharacterLocation - LadderFrontierLocation) < LadderEnterBound)
				{
					sprintf(DebugBuffer, "LadderDetection ready to ladder\n", t); Debug();

					state_movement_on_ladder = true;

					GWorld.Capsules["eve"].origin = NewCharacterLocation;
				}
			}
		}
	}

	void LadderDetection()
	{
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		CurrentCharacterLocation = GWorld.Capsules["eve"].origin;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (state_walking_on_your_feet)
		{
			_LadderDetection();
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PrevCharacterLocation = CurrentCharacterLocation;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	}
}
