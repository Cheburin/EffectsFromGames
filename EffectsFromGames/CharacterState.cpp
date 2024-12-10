#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

SimpleMath::Vector3 gForward = SimpleMath::Vector3(1, 0, 0);

SimpleMath::Vector3 gravitation = SimpleMath::Vector3(0, -0.10, 0);

float eveCurrentRotation = 0;
float eveTargetRotation = 0;
float eveDeltaRotation = 0.0f;
float eveDeltaRotationSpeed = 0.0f;

SimpleMath::Vector3 eveTargetPos;

SimpleMath::Vector2 input_move;
bool input_jump = false;

bool state_jump = false;
bool state_jump_from_wall = false;
bool state_climbing = false;
bool state_falling = false;
bool state_into_water = false;
bool state_take_it = false;
bool state_kneeling = false;
bool state_falling_and_hang_on = false;
bool state_walking_on_your_feet = false;
bool state_idle = false;

//1 ballistic fly /**/
bool state_ballistic_fly_to_target = false;
SimpleMath::Vector3 state_ballistic_fly_Start_Location;
SimpleMath::Vector3 state_ballistic_fly_Start_World_Location;

SimpleMath::Vector3 state_ballistic_fly_Finish_Location;
SimpleMath::Vector3 state_ballistic_fly_Finish_World_Location;

bool state_BallisticFly_to_HangingIdle = false;
bool state_BallisticFly_to_HangingIdleWithOutLeg = false;
//2 ballistic fly /**/

//1 hanging /**/
bool state_hanging = false;
//bool state_prevent_hanging = false;

char* state_hanging_Hand_Name = nullptr;
SimpleMath::Vector3 state_hanging_Hand_Location;

std::string state_hanging_Ledge_Name;
Box state_hanging_Ledge_Box;
int state_hanging_Ledge_BoxIndex;
//2 hanging /**/

bool state_play_debug_animation = false;

char* LeftHandName = "LeftHand";
char* RightHandName = "RightHand";
char* Hands[2] = { LeftHandName, RightHandName };

std::vector<Capsule> DebugCapsule;

bool state_movement_is_obstructed = false;

float state_CapsulAlfa_WS;

float state_HipsAlfa_WS;