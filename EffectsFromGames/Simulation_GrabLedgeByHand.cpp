#include "main.h"
#include "DXUTgui.h"
#include "SDKmisc.h"
#include <algorithm>

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern bool state_jump;
extern bool state_climbing;
extern bool state_ballistic_fly_to_target;
extern bool state_falling_and_hang_on;
extern bool state_falling;

extern bool state_hanging;
//extern bool state_prevent_hanging;

extern char* state_hanging_Hand_Name;
extern SimpleMath::Vector3 state_hanging_Hand_Location;

extern std::string state_hanging_Ledge_Name;
extern Box state_hanging_Ledge_Box;
extern int state_hanging_Ledge_BoxIndex;

namespace 
{
	int GrabLedgesCount = 0;
	struct FGrabLedges{
		bool Grab;
		std::string Ledge_Name;
		Box Ledge_Box;
		int Ledge_BoxIndex;
		FGrabLedges() : Grab(){}
		FGrabLedges(const std::string& The_Ledge_Name, const Box& The_Ledge_Box, const int The_Ledge_BoxIndex) :
			Grab(true), 
			Ledge_Name(The_Ledge_Name),
			Ledge_Box(The_Ledge_Box),
			Ledge_BoxIndex(The_Ledge_BoxIndex)
		{}
	} GrabLedges[10];
}

namespace Simulation
{
	void AddLedge(const std::string& The_Ledge_Name, const Box& The_Ledge_Box, const int The_Ledge_BoxIndex)
	{
		for (int i = 0; i < 10; i++)
		{
			if (GrabLedges[i].Grab && GrabLedges[i].Ledge_Name == The_Ledge_Name && GrabLedges[i].Ledge_BoxIndex == The_Ledge_BoxIndex)
			{
				return;
			}
		}
		for (int i = 0; i < 10; i++)
		{
			if (!GrabLedges[i].Grab)
			{
				GrabLedgesCount += 1;
				state_hanging = true;

				sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Grab Ledge Hand_Name(%s) Ledge_Name(%s) Ledge_BoxIndex(%d) state_falling(%d)\n", 
					//FromModelSpaceToWorld._41, FromModelSpaceToWorld._42, FromModelSpaceToWorld._43, 
					state_hanging_Hand_Name, 
					&The_Ledge_Name[0],
					The_Ledge_BoxIndex, state_falling); Debug();

				GrabLedges[i] = FGrabLedges(The_Ledge_Name, The_Ledge_Box, The_Ledge_BoxIndex);
				return;
			}
		}
		assert(false);
	}
	void FreeLedgeByIndex(int index)
	{
		GrabLedgesCount -= GrabLedges[index].Grab ? 1 : 0;
		state_hanging = GrabLedgesCount != 0;

		//sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Release Ledge %d [%f %f %f] %s %s %d state_falling %d\n",
		sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Release Ledge Hand_Name(%s) state_hanging(%d) Ledge_Name(%s) Ledge_BoxIndex(%d) state_falling(%d)\n",
			state_hanging_Hand_Name,
			state_hanging,
			//FromModelSpaceToWorld._41, FromModelSpaceToWorld._42, FromModelSpaceToWorld._43, 
			//state_hanging_Hand_Name, 
			&GrabLedges[index].Ledge_Name[0],
			GrabLedges[index].Ledge_BoxIndex,
			state_falling); Debug();
		//state_prevent_hanging = false;

		GrabLedges[index] = FGrabLedges();
	}

	void GrabLedgeByHand(const SimpleMath::Matrix& FromModelSpaceToWorld)
	{
		//if (state_prevent_hanging)
		//{
			if (state_hanging)
			{
				for (int i = 0; i < 10; i++)
				{
					if (GrabLedges[i].Grab
						&&
						!IsJointOrNestedJointsIntoBox(Eve, FromModelSpaceToWorld, "LeftHand", 0, GrabLedges[i].Ledge_Box)
						&&
						!IsJointOrNestedJointsIntoBox(Eve, FromModelSpaceToWorld, "RightHand", 0, GrabLedges[i].Ledge_Box))
					{
						FreeLedgeByIndex(i);
					}
				}
			}
		//}
		else if (state_falling || state_jump || state_ballistic_fly_to_target)
		{
			for (auto Ledge_Iter = GWorld.Ledges.begin(); Ledge_Iter != GWorld.Ledges.end(); Ledge_Iter++){
				const auto & Ledge_Boxes = Ledge_Iter->second.Boxes;
				for (auto Ledge_Box_Iter = Ledge_Boxes.begin(); Ledge_Box_Iter != Ledge_Boxes.end(); Ledge_Box_Iter++){
					for (int i = 0; i < 2; i++){
						if (IsJointOrNestedJointsIntoBox(Eve, FromModelSpaceToWorld, Hands[i], 0, *Ledge_Box_Iter))
						{
							state_hanging_Hand_Name = Hands[i];
							state_hanging_Hand_Location = GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name);

							state_hanging_Ledge_Name = Ledge_Iter->first;
							state_hanging_Ledge_Box = *Ledge_Box_Iter;
							state_hanging_Ledge_BoxIndex = Ledge_Box_Iter - Ledge_Boxes.begin();

							AddLedge(
								state_hanging_Ledge_Name,
								state_hanging_Ledge_Box, 
								state_hanging_Ledge_BoxIndex
							);

							//sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Grab Ledge [%f %f %f] %s %s %d state_falling %d\n", FromModelSpaceToWorld._41, FromModelSpaceToWorld._42, FromModelSpaceToWorld._43, state_hanging_Hand_Name, &state_hanging_Ledge_Name[0], state_hanging_Ledge_BoxIndex, state_falling); Debug();
							
							break;
						}
					}
				}
			}
		}
	}
}