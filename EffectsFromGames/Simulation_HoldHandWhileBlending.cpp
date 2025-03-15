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
	std::map<char*, float> HoldWhileBlendPlaying;
}

namespace Simulation
{
	void HoldHandWhileBlending(char* CapsuleName, char* ModelTransformName)
	{
		static bool bInit = false;
		if (!bInit)
		{
			bInit = true;
			HoldWhileBlendPlaying.insert(std::pair<char*, float>("LeftShimmy", -1.f));
			HoldWhileBlendPlaying.insert(std::pair<char*, float>("RightShimmy", -1.f));

			HoldWhileBlendPlaying.insert(std::pair<char*, float>("climbing", 0.3f));
			HoldWhileBlendPlaying.insert(std::pair<char*, float>("Freehang_Climb", 0.3f));

			HoldWhileBlendPlaying.insert(std::pair<char*, float>("ActionPose_Release_And_Go_Down", -1.f));
			HoldWhileBlendPlaying.insert(std::pair<char*, float>("Climb_Fold_Hands", -1.f));
		}

		char* CurrentAnimName = EveAnimationGraph->getAnimationName();
		const auto ptrHoldWhileBlendPlaying = HoldWhileBlendPlaying.find(CurrentAnimName);
		if (ptrHoldWhileBlendPlaying != HoldWhileBlendPlaying.end())
		{
			bool  CurrentAnimBlending = EveAnimationGraph->getAnimationBlend()->isPlaying();
			float CurrentAnimTime = EveAnimationGraph->getAnimation<Animation>()->getLocTime();
			if (CurrentAnimBlending || CurrentAnimTime < ptrHoldWhileBlendPlaying->second)
			{
				auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms[ModelTransformName]) * GWorld.Capsules[CapsuleName].getMatrix();
				auto Offset = state_hanging_Hand_Location - GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name);
				GWorld.Capsules[CapsuleName].origin += Offset;
			}
		}
	}
}