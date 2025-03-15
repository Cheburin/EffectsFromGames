#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern bool state_falling;
extern bool state_hanging;
extern bool state_falling_and_hang_on;
extern bool state_into_water;

extern char* state_hanging_Hand_Name;
extern SimpleMath::Vector3 state_hanging_Hand_Location;
extern std::string state_hanging_Ledge_Name;

//extern bool state_prevent_hanging;

namespace Simulation
{
	void EnterIntoWater(char* CapsuleName, char* ModelTransformName)
	{
		if (state_hanging && state_hanging_Ledge_Name == "Water")
		{
			if (!state_into_water)
			{
				state_into_water = true;
				//state_prevent_hanging = true;
			}
			else if (EveAnimationGraph->getAnimationBlend()->isPlaying())
			{
				auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms[ModelTransformName]) * GWorld.Capsules[CapsuleName].getMatrix();
				auto Offset = state_hanging_Hand_Location - GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name);
				GWorld.Capsules[CapsuleName].origin += SimpleMath::Vector3(0, Offset.y, 0);
			}
			else
			{
				state_falling = false;
				state_hanging = false;
			}
		}
	}
}