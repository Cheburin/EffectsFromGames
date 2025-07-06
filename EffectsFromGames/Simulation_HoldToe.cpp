#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern bool state_hanging_Toe_activated;
extern char* state_hanging_Toe_Name;
extern SimpleMath::Vector3 state_hanging_Toe_Location;

namespace
{
	std::set<char*> HoldWhilePlaying;
}

namespace Simulation
{
	void HoldToe(char* CapsuleName, char* ModelTransformName)
	{
		static bool bInit = false;
		if (!bInit)
		{
			bInit = true;
			HoldWhilePlaying.insert("Left_Edge_Horizontal_Jump");
			HoldWhilePlaying.insert("Right_Edge_Horizontal_Jump");
		}

		if (state_hanging_Toe_activated)
		{
			//char* CurrentAnimName = EveAnimationGraph->getAnimationName();
			//
			//const auto ptrHoldWhilePlaying = HoldWhilePlaying.find(CurrentAnimName);
			//if (ptrHoldWhilePlaying != HoldWhilePlaying.end())
			//{
			//	auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms[ModelTransformName]) * GWorld.Capsules[CapsuleName].getMatrix();
			//	auto Offset = state_hanging_Toe_Location - GetHandLocation(FromModelSpaceToWorld, state_hanging_Toe_Name);
			//	GWorld.Capsules[CapsuleName].origin += Offset;
			//}
		}
	}
}