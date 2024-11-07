#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern SimpleMath::Vector3 CollisionImpactPoint;

extern bool state_falling;
extern bool state_falling_and_hang_on;

extern std::string state_hanging_Ledge_Name;
extern Box state_hanging_Ledge_Box;
extern int state_hanging_Ledge_BoxIndex;

namespace Simulation
{
	void FallingAndHangOn(const SimpleMath::Vector3& CapsuleOrigin, const SimpleMath::Vector3& CapsuleA)
	{
		if (state_falling_and_hang_on)
		{
			return;
		}
		for (auto Ledge_Iter = GWorld.Ledges.begin(); Ledge_Iter != GWorld.Ledges.end(); Ledge_Iter++)
		{
			const auto & Ledge_Boxes = Ledge_Iter->second.Boxes;
			for (auto Ledge_Box_Iter = Ledge_Boxes.begin(); Ledge_Box_Iter != Ledge_Boxes.end(); Ledge_Box_Iter++)
			{
				if (Ledge_Box_Iter->Containe(CapsuleOrigin))
				{
					auto dir = (CollisionImpactPoint - CapsuleA);
					dir.Normalize();
					if (dir.y < 0.0)
					{
						dir.y *= -1.0f;
						float Alfa = acos(dir.y) * 180 / PI;
						if (Alfa>5.f)
						{
							//state_hanging_Ledge_Name = Ledge_Iter->first;
							//state_hanging_Ledge_Box = *Ledge_Box_Iter;
							//state_hanging_Ledge_BoxIndex = Ledge_Box_Iter - Ledge_Boxes.begin();

							state_falling_and_hang_on = true;

							state_hanging_Ledge_Name = Ledge_Iter->first;
							state_hanging_Ledge_Box = *Ledge_Box_Iter;
							state_hanging_Ledge_BoxIndex = Ledge_Box_Iter - Ledge_Boxes.begin();

							sprintf(DebugBuffer, "Simulation::FallingAndHangOn %f\n", Alfa); Debug();
							break;
						}
					}
				}

			}
		}
	}
}