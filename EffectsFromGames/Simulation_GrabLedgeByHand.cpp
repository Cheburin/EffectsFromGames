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
extern SimpleMath::Vector3 EveHandLocation[2];

extern EvePath g_EvePath;

extern std::string start_hanging_Ledge_Name;
extern int start_hanging_Ledge_BoxIndex;

SimpleMath::Vector4 vec4(SimpleMath::Vector3 v3, float w);

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

				sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Grab Ledge Hand_Name(%s) Ledge_Name(%s) Ledge_BoxIndex(%d) state_falling(%d) Hand_Location(%f %f %f)\n", 
					//FromModelSpaceToWorld._41, FromModelSpaceToWorld._42, FromModelSpaceToWorld._43, 
					state_hanging_Hand_Name, 
					&The_Ledge_Name[0],
					The_Ledge_BoxIndex, state_falling, state_hanging_Hand_Location.x, state_hanging_Hand_Location.y, state_hanging_Hand_Location.z); Debug();

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

	bool CheckHandIntoBox(const SimpleMath::Matrix& FromModelSpaceToWorld, char* HandName, const Box2& Ledge_Box, const SimpleMath::Vector3& A, const SimpleMath::Vector3& B, SimpleMath::Vector3& Hand_Location)
	{
		if (IsJointOrNestedJointsIntoBox(Eve, FromModelSpaceToWorld, HandName, 0, Ledge_Box))
		{
			Hand_Location = GetHandLocation(FromModelSpaceToWorld, HandName);

			return true;
		}
		else
		{
			float t;

			auto V = B - A;
			if (Ledge_Box.Intersect(A, V, &t))
			{
				const auto a = A + t*V;
				
				Ledge_Box.Intersect(B, -V, &t);
				const auto b = B - t*V;

				Hand_Location = 0.5f*a + 0.5f*b;//A + t*V + SimpleMath::Vector3(0, -(1.f / 3.f * Ledge_Box.worldSize.y), 0);

				sprintf(DebugBuffer, "|Simulation::GrabLedgeByHand::CheckHandIntoBox by Trace\n"); Debug();

				g_EvePath.Count = 5;
				g_EvePath.Pivots[0] = vec4(A, 1);
				g_EvePath.Pivots[1] = vec4(A + t*V, 1);
				g_EvePath.Pivots[2] = vec4(Hand_Location, 1);
				g_EvePath.Pivots[3] = vec4(A + t*V, 1);
				g_EvePath.Pivots[4] = vec4(B, 1);

				return true;
			};
		}
		return false;
	}

	void GrabLedgeByHand(SimpleMath::Matrix& FromModelSpaceToWorld)
	{
		//EveHandLocation[]
		SimpleMath::Vector3 prev_EveHandLocation[2];
		std::memcpy(prev_EveHandLocation, EveHandLocation, sizeof(EveHandLocation));
		SimpleMath::Vector3 cur_EveHandLocation[2] = { GetHandLocation(FromModelSpaceToWorld, Hands[0]), GetHandLocation(FromModelSpaceToWorld, Hands[1]) };
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
						if (CheckHandIntoBox(FromModelSpaceToWorld, Hands[i], *Ledge_Box_Iter, prev_EveHandLocation[i], cur_EveHandLocation[i], state_hanging_Hand_Location))
						{
							state_hanging_Hand_Name = Hands[i];

							state_hanging_Ledge_Name = Ledge_Iter->first;
							state_hanging_Ledge_Box = *Ledge_Box_Iter;
							state_hanging_Ledge_BoxIndex = Ledge_Box_Iter - Ledge_Boxes.begin();

							AddLedge(
								state_hanging_Ledge_Name,
								state_hanging_Ledge_Box, 
								state_hanging_Ledge_BoxIndex
							);

							//sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Grab Ledge HandLocation == [%f %f %f]\n", state_hanging_Hand_Location.x, state_hanging_Hand_Location.y, state_hanging_Hand_Location.z); Debug();
							//sprintf(DebugBuffer, "Simulation::GrabLedgeByHand Grab Ledge [%f %f %f] %s %s %d state_falling %d\n", FromModelSpaceToWorld._41, FromModelSpaceToWorld._42, FromModelSpaceToWorld._43, state_hanging_Hand_Name, &state_hanging_Ledge_Name[0], state_hanging_Ledge_BoxIndex, state_falling); Debug();
							
							//char* CurrentAnimName = EveAnimationGraph->getAnimationName();
							//if (CurrentAnimName == "Left_Edge_Horizontal_Jump" || CurrentAnimName == "Right_Edge_Horizontal_Jump")
							//{
							if (start_hanging_Ledge_Name != state_hanging_Ledge_Name || start_hanging_Ledge_BoxIndex != state_hanging_Ledge_BoxIndex)
							{
								sprintf(DebugBuffer, "******************Simulation::GrabLedgeByHand******************\n"); Debug();
								GWorld.Capsules["eve"].origin += state_hanging_Hand_Location - GetHandLocation(FromModelSpaceToWorld, state_hanging_Hand_Name);
								FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
							}

							break;
						}
					}
				}
			}
		}
		std::memcpy(EveHandLocation, cur_EveHandLocation, sizeof(EveHandLocation));
	}
}