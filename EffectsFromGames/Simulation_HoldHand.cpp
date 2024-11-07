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
	std::set<char*> HoldWhilePlaying;
}

namespace Simulation
{
	void HoldHand(char* CapsuleName, char* ModelTransformName)
	{
		static bool bInit = false;
		if (!bInit)
		{
			bInit = true;
			HoldWhilePlaying.insert("hangingIdle");
			HoldWhilePlaying.insert("Jump_To_Hang");
			HoldWhilePlaying.insert("Hanging_Idle_With_OutLeg");
			HoldWhilePlaying.insert("Climb_Look_Idle_L");
			HoldWhilePlaying.insert("Climb_Look_Idle_R");
		}

		char* CurrentAnimName = EveAnimationGraph->getAnimationName();

		const auto ptrHoldWhilePlaying = HoldWhilePlaying.find(CurrentAnimName);
		if (ptrHoldWhilePlaying != HoldWhilePlaying.end())
		{
			auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms[ModelTransformName]) * GWorld.Capsules[CapsuleName].getMatrix();
			auto Offset = state_hanging_Hand_Location - GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name);
			GWorld.Capsules[CapsuleName].origin += Offset;
		}
	}
}