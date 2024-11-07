#include "main.h"

#include "DXUTgui.h"
#include "SDKmisc.h"

#include <algorithm>

extern SceneState scene_state;

extern PostProccessState post_proccess_state;

extern World GWorld;

extern IAnimationGraph2 * EveAnimationGraph;

extern IKSolverInterface * EveIKSolver;

extern IClimbingPathHelper* ClimbingPathHelper;

extern JointsRefsChainCollection jointsRefsChains;

extern Character* Eve;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool simulation_state_manual_control = false;
bool simulation_state_next_step = false;

bool ScanEnable = false;
bool ThirdPersonCameraMode = true;
bool FreeCameraMode = false;

SimpleMath::Vector3 gUp = SimpleMath::Vector3(0,1,0);

SimpleMath::Vector3 cameraForward;
SimpleMath::Vector3 cameraRight;
SimpleMath::Vector3 cameraPos;
SimpleMath::Vector3 cameraTarget;
SimpleMath::Vector3 desiredDirectionToTarget;
SimpleMath::Vector3 desiredTarget;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern bool ShowCapsule;
extern bool ShowJoints;

extern SimpleMath::Vector3 gForward;

extern SimpleMath::Vector3 gravitation;

extern float eveCurrentRotation;
extern float eveTargetRotation;
extern float eveDeltaRotation;
extern float eveDeltaRotationSpeed;

extern SimpleMath::Vector2 input_move;
extern bool input_jump;

extern bool state_jump;
extern bool state_hanging;
extern bool state_climbing;
extern bool state_falling;
extern bool state_into_water;
extern bool state_take_it;
extern bool state_ballistic_fly_to_target;
extern bool state_kneeling;
extern bool state_falling_and_hang_on;

extern bool state_ortho_proj;
extern bool state_pers_proj;

extern bool state_channel1_for_blend;
extern SimpleMath::Vector3 state_origin_for_blend;
extern SimpleMath::Vector3 state_origin_offset_for_blend;

extern bool state_play_debug_animation;

extern EvePath g_EvePath;

SimpleMath::Vector3 ProjectOnGround(const SimpleMath::Vector3 & a);

extern char DebugBuffer[1024];

void Debug();

float sign(const float& arg);

float CameraBoom = 0.f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateOrientation(SimpleMath::Vector3 cameraDirection, SimpleMath::Vector3& _cameraPos, SimpleMath::Vector3& _cameraTarget){
	cameraForward = cameraDirection;
	cameraRight = SimpleMath::Vector3(0, 1, 0).Cross(cameraDirection);

	if (FreeCameraMode)
	{
		cameraTarget = cameraPos + cameraDirection;
	}
	else if (ThirdPersonCameraMode)
	{
		SimpleMath::Vector3 A;
		SimpleMath::Vector3 B;

		GWorld.Capsules["eve"].getAB(A, B);

		cameraTarget = 0.5f*(A + B);
		cameraPos = cameraTarget - (20.0f - CameraBoom)*cameraDirection;
	}

	_cameraPos = cameraPos;
	_cameraTarget = cameraTarget;
}

SimpleMath::Vector2 prevMove;

void updatePosition(float fElapsedTime, SimpleMath::Vector2 move){
	if (FreeCameraMode)
	{
		cameraPos += fElapsedTime * 1750 * (move.x*cameraForward + move.y*cameraRight);
	}
	else if (ThirdPersonCameraMode)
	{
		SimpleMath::Vector3 delta;

		input_move = move;

		if (move.Length() == 0.0f)
		{
			auto Q = SimpleMath::Quaternion::CreateFromYawPitchRoll(eveCurrentRotation * PI / 180.0, .0, .0);
			auto M = SimpleMath::Matrix::CreateFromQuaternion(Q);
			delta = M.Right();
		}
		else
		{
			auto Forward = ProjectOnGround(cameraForward);
			Forward.Normalize();
			auto Right = ProjectOnGround(cameraRight);
			Right.Normalize();
			delta = move.x*Forward + move.y*Right;
		}
		prevMove = move;

		desiredDirectionToTarget = delta;
	}
}

SimpleMath::Vector3 MouseRayOrigin;
SimpleMath::Vector3 MouseRayDirection;

void reñeiveMouseInput(DirectX::Mouse::State ms){
	float x, y;
	SimpleMath::Vector3 _v, _s;
	{
		x = float(ms.x) / scene_state.vFrustumParams.x;
		y = float(ms.y) / scene_state.vFrustumParams.y;

		x = x * 2 - 1;
		y = -y * 2 + 1;

		_v = SimpleMath::Vector3(x / scene_state.vFrustumParams.z, y / scene_state.vFrustumParams.w, 1.0);
		_s = SimpleMath::Vector3(scene_state.mInvView._14, scene_state.mInvView._24, scene_state.mInvView._34);

		_v = SimpleMath::Vector3::TransformNormal(_v, SimpleMath::Matrix(scene_state.mInvView).Transpose());

		MouseRayOrigin = _s;
		MouseRayDirection = _v;
	}
	//if (FreeCameraMode && ms.rightButton && ms.positionMode == Mouse::MODE_ABSOLUTE)
	//{
	//	float t = -(SimpleMath::Plane(0, 1, 0, 0).DotCoordinate(_s) / SimpleMath::Plane(0, 1, 0, 0).DotNormal(_v));

	//	desiredTarget = _s + t * _v;
	//	desiredDirectionToTarget = desiredTarget - ProjectOnGround(GWorld.Capsules["eve"].origin);

	//	input_move = SimpleMath::Vector2(1, 0);
	//}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
struct QKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::Q;
};
struct EKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::E;
};
struct PageUpKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::PageUp;
};
struct PageDownKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::PageDown;
};
struct F1Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F1;
};
struct F3Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F3;
};
struct F4Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F4;
};
struct F5Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F5;
};
struct F6Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F6;
};
struct F9Key
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F9;
};
struct SpaceKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::Space;
};
struct XKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::X;
};
struct ZKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::Z;
};
struct CKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::C;
};
struct FKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::F;
};
struct GKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::G;
};
struct LeftShiftKey
{
	static const DirectX::Keyboard::Keys Value = DirectX::Keyboard::Keys::LeftShift;
};

template<class Key>
void IsKeyPressed(DirectX::Keyboard::State kb, std::function<void __cdecl()> OnPressed)
{
	static bool IsDown = false;
	if (kb.IsKeyDown(Key::Value)){
		IsDown = true;
	}
	if (IsDown && kb.IsKeyUp(Key::Value)){
		IsDown = false;
		OnPressed();
	}
}

template<class Key>
void IsKeyDown(DirectX::Keyboard::State kb, std::function<void __cdecl()> OnDown)
{
	if (kb.IsKeyDown(Key::Value)){
		OnDown();
	}
}

template<class Key>
void IsKeyUp(DirectX::Keyboard::State kb, std::function<void __cdecl()> OnUp)
{
	if (kb.IsKeyUp(Key::Value)){
		OnUp();
	}
}

bool state_force_kneeling = false;

void reñeiveKeyBoardInput(DirectX::Keyboard::State kb){
	IsKeyPressed<QKey>(kb, []{
		ScanEnable = !ScanEnable;
	});
	IsKeyPressed<EKey>(kb, []{
		ScanEnable = false;
		post_proccess_state.ScanDistance = 0.0f;
	});
	IsKeyDown<PageUpKey>(kb, []{
		EveAnimationGraph->getAnimation<Animation>()->setRate(EveAnimationGraph->getAnimation<Animation>()->getRate() + 0.01);
	});
	IsKeyDown<PageDownKey>(kb, []{
		EveAnimationGraph->getAnimation<Animation>()->setRate(EveAnimationGraph->getAnimation<Animation>()->getRate() - 0.01);
	});
	IsKeyPressed<SpaceKey>(kb, []{
		input_jump = true;
	});
	IsKeyPressed<F3Key>(kb, []{
		std::swap(FreeCameraMode, ThirdPersonCameraMode);
	});
	IsKeyPressed<F4Key>(kb, []{
		ShowCapsule = !ShowCapsule;
	});
	IsKeyPressed<F5Key>(kb, []{
		ShowJoints = !ShowJoints;
	});
	IsKeyPressed<F6Key>(kb, []{
		state_play_debug_animation = true;
	});
	IsKeyDown<F9Key>(kb, []{
		CameraBoom += 0.1f;
		extern std::vector<Capsule> DebugCapsule;
		DebugCapsule.clear();
	});
	IsKeyDown<ZKey>(kb, []{
		simulation_state_next_step = true;
	});
	IsKeyPressed<XKey>(kb, []{
		simulation_state_manual_control = !simulation_state_manual_control;
	});
	//IsKeyPressed<FKey>(kb, []{
	//state_take_it = true;
	IsKeyDown<FKey>(kb, []{
		void UpdateJumpFromWallTrajectoryHandler(Animation *Anim);
		UpdateJumpFromWallTrajectoryHandler(EveAnimationGraph->getAnimation<Animation>());
	});
	IsKeyPressed<GKey>(kb, []{
		void SelectedJumpFromWallTrajectoryHandler(Animation *Anim);
		SelectedJumpFromWallTrajectoryHandler(EveAnimationGraph->getAnimation<Animation>());
	});
	IsKeyPressed<F1Key>(kb, []{
		if (state_ortho_proj){
			state_ortho_proj = false;
			state_pers_proj = true;
		}
		else if (state_pers_proj){
			state_pers_proj = false;
			state_ortho_proj = true;
		}
	});
	IsKeyDown<LeftShiftKey>(kb, []{
		state_kneeling = true;
	});
	IsKeyUp<LeftShiftKey>(kb, []{
		state_kneeling = state_force_kneeling;
	});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

//void physCollision(const Capsule& ca, const SimpleMath::Vector3 & worldDistance, float & T, SimpleMath::Vector3& impactPoint, SimpleMath::Vector3& impactNormal, std::string& objectName, int& objectType);
SimpleMath::Vector3 physGetSlidingVector(const SimpleMath::Vector3 & worldDistance, float min_t, SimpleMath::Vector3& impactNormal);
SimpleMath::Vector3 GetCharacterJointTranslation(CharacterSkelet * characterSkelet, int Index);
SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);

//SimpleMath::Vector3 CollisionImpactNormal;
//SimpleMath::Vector3 CollisionImpactPoint;
//std::string CollisionObjectName;
//int CollisionObjectType;
SimpleMath::Vector3 EveHeadLocation = SimpleMath::Vector3::Zero;
//void physCollision2(const Capsule& ca, const SimpleMath::Vector3 & worldDistance, float & T, SimpleMath::Vector3& impactPoint, SimpleMath::Vector3& impactNormal)
//{
//	std::string objectName;
//	int objectType;
//	physCollision(ca, worldDistance, T, impactPoint, impactNormal, objectName, objectType);
//}

void SceneSimulation(double fTime, float fElapsedTime, void* pUserContext)
{
	using namespace SimpleMath;

	///transition helpers
	static bool transition_Into_Water = false;

	////initialization
	static bool doOnce = false;
	if (!doOnce){
		doOnce = true;
	}

	////other think simulation
	{
		if (ScanEnable)
		{
			post_proccess_state.ScanDistance += fElapsedTime * 15;
		}

		scene_state.vTime.z += 0.001f;
		if (scene_state.vTime.z > 1.0f)
			scene_state.vTime.z = -1.0f;

		scene_state.vTime.x = fTime*0.3f;

		auto View = SimpleMath::Matrix(XMMatrixTranspose(XMLoadFloat4x4(&scene_state.mView)));

		post_proccess_state.ScanerPosition = SimpleMath::Vector4(SimpleMath::Vector3::Transform(SimpleMath::Vector3(-5, 0, -5), View));
	}

	//if (FreeCameraMode)
	//{
	//	desiredDirectionToTarget = desiredTarget - ProjectOnGround(GWorld.Capsules["eve"].origin);
	//	if (desiredDirectionToTarget.Length() < 1.0f)
	//		input_move = SimpleMath::Vector2(0, 0);
	//}

	////camera move
	Camera::OnFrameMove(fTime, fElapsedTime, pUserContext);

	////Simulation mode(manual/auto)
	auto simulationTime = fElapsedTime;
	{
		if (simulation_state_manual_control)
		{
			input_move.y = -1.f;
			if (simulation_state_next_step)
			{
				simulationTime = 1.0 / 600.0;
			}
			else
			{
				simulationTime = 0.0;
			}
			simulation_state_next_step = false;
		}
	}

	////IK Sample
	if (state_take_it)
	{
		typedef SimpleMath::Vector3 Vec3;

		auto currentPos = EveAnimationGraph->getAnimation("take_it_pose");

		copyAnimationPose(EveAnimationGraph->getAnimation<AnimationBase>(), currentPos);

		////////////////////////////////////////////////////////////////////////////////////////////////////
		auto modelTransform = Matrix::CreateScale(0.01f) * Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

		SimpleMath::Vector3 target = Vec3::Transform(Vec3::Transform(Vec3(0.f, 0.5f, 0.f), GWorld.WorldTransforms["platform2"]), modelTransform.Invert());

		////////////////////////////////////////////////////////////////////////////////////////////////////
		JointsRefsChainCollection & Chains = jointsRefsChains;

		JointHelpers::AnimToChain(Chains.RightShoulderRightHand, currentPos, EveIKSolver->chainRef(0));
		JointHelpers::localToModel(Chains.RightShoulderRightHand.size(), EveIKSolver->chainRef(0));

		EveIKSolver->solve(Chains.RightShoulderRightHand.size(), JointHelpers::transformFromFirstToLastJointFrame(Chains.HipsSpine2, currentPos, target));

		JointHelpers::modelToLocal(Chains.RightShoulderRightHand.size(), EveIKSolver->chainRef(0));
		JointHelpers::ChainToAnim(Chains.RightShoulderRightHand, EveIKSolver->chainRef(0), currentPos);
		////////////////////////////////////////////////////////////////////////////////////////////////////
	}

	////Eve Advanse Animation
	SimpleMath::Vector3 deltaTranslation;
	SimpleMath::Quaternion deltaRotation;
	SimpleMath::Matrix FromModelSpaceToWorld;

	EveAnimationGraph->advanse(simulationTime, deltaTranslation, deltaRotation);
	Simulation::UpdateCapsuleRotation(GWorld.Capsules["eve"]);
	fillSkeletonTransformFromJoint(EveAnimationGraph->getPlayingAnimation(), Eve->skelet);
	calculateFramesTransformations(Eve->frame, Matrix::Identity);
	
	Simulation::Collision(GWorld.Capsules["eve"], deltaTranslation, deltaRotation);

	Simulation::CharacterLerpRotation(simulationTime, "eve", "eveSkinnedModel");/* need update FromModelSpaceToWorld*/
	Simulation::HoldHandWhileBlending("eve", "eveSkinnedModel");/* need update FromModelSpaceToWorld*/
	Simulation::HoldHand("eve", "eveSkinnedModel");/* need update FromModelSpaceToWorld*/
	Simulation::EnterIntoWater("eve", "eveSkinnedModel");/* need update FromModelSpaceToWorld*/

	FromModelSpaceToWorld = Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
	SimpleMath::Vector3 capsuleA, capsuleB;
	GWorld.Capsules["eve"].getAB(capsuleA, capsuleB);

	Simulation::GrabLedgeByHand(FromModelSpaceToWorld);
	Simulation::FallingAndHangOn(GWorld.Capsules["eve"].origin, capsuleA);
	Simulation::StartBallisticFly(FromModelSpaceToWorld, SimpleMath::Vector3::Transform(SimpleMath::Vector3(1, 0, 0), GWorld.Capsules["eve"].orientation));
	Simulation::FinishBallisticFly(GWorld.Capsules["eve"].getMatrix(), GWorld.Capsules["eve"].ab, GWorld.Capsules["eve"].r);
	Simulation::AutoKneeling(FromModelSpaceToWorld);
	UpdateEveCapsuleHeight(FromModelSpaceToWorld);
	////Eve Combine Animation Transaction 
	{
		{
			////////////////////////////////////////////////
			///auto orientation = Matrix::CreateFromQuaternion(GWorld.Capsules["eve"].orientation);

			/*
			if (EveAnimationGraph->getAnimationName() == "hangingIdle")
			{
			state_origin_offset_for_blend = SimpleMath::Vector3();
			if (std::string(state_hang_on_object_name) == "sub_box0")
			state_origin_offset_for_blend = -0.6f*orientation.Right();
			if (std::string(state_hang_on_object_name) == "sub_box1")
			state_origin_offset_for_blend = +0.3f*orientation.Right();
			}
			*/

			////auto offset0 = 0.01f*EveAnimationGraph->getAnimationBlend()->getCurrentJointTranslationOffset(64);
			//auto offset0 = EveAnimationGraph->getAnimationBlend()->getCurrentMeta(0);

			////offset0.x = offset0.z = 0.0f;
			//sprintf(DebugBuffer, "offset0 %f\n", offset0.y); Debug();
			//offset1.x = offset1.z = 0.0f;

			///offset0 = SimpleMath::Vector3::Transform(offset0, GWorld.WorldTransforms["eveSkinnedModel"]);
			///offset0 = SimpleMath::Vector3::Transform(offset0, Matrix::CreateFromQuaternion(GWorld.Capsules["eve"].orientation));
			// -(EveAnimationGraph->getAnimationName() == "treading_water" ? offset0 : SimpleMath::Vector3::Zero);
			// +EveAnimationGraph->getAnimationBlend()->getTimeRatio()*state_origin_offset_for_blend;
			////////////////////////////////////////////////
		}

		/*
		if (EveAnimationGraph->getAnimationName() == "swimming")
		{
		if (EveAnimationGraph->getAnimationBlend()->getPlaying())
		{
		if (EveAnimationGraph->getAnimation()->getFrame() == 0)
		{
		state_origin_for_blend = GWorld.Capsules["eve"].origin;
		}
		}
		}

		auto _gravitation = gravitation;

		if (!state_falling && EveAnimationGraph->getAnimationName() == "treading_water")
		{
		{
		if (EveAnimationGraph->getAnimationBlend()->getPlaying())
		{
		if (EveAnimationGraph->getAnimation()->getFrame() == 0)
		{
		state_origin_offset_for_blend = state_origin_for_blend - GWorld.Capsules["eve"].origin;
		state_origin_for_blend = GWorld.Capsules["eve"].origin;
		}
		GWorld.Capsules["eve"].origin.y = state_origin_for_blend.y + state_origin_offset_for_blend.y * EveAnimationGraph->getAnimationBlend()->getTimeRatio();
		}

		auto hand0 = EveAnimationGraph->getAnimationBlend()->getPlaying()
		? EveAnimationGraph->getAnimation()->CurrentMetaChannels[1]
		: EveAnimationGraph->getAnimationBlend()->getCurrentMeta(1);

		hand0 = SimpleMath::Vector3::Transform(hand0, GWorld.WorldTransforms["eveSkinnedModel"]);
		hand0 = SimpleMath::Vector3::Transform(hand0, GWorld.Capsules["eve"].getMatrix());

		auto p1 = vec4(SimpleMath::Vector3(0, 1, 0), -(SimpleMath::Matrix(GWorld.WorldTransforms["pools_water"]).Translation().y));
		auto d1 = vec4(hand0, 1).Dot(p1);

		if (d1 < -0.2f || 0.2f < d1)
		{
		_gravitation += (d1>0.0f ? +1 * 0.25f * gravitation : -1 * 0.25f * gravitation);
		sprintf(DebugBuffer, "Correction treading_water via gravitation %f\n", d1); Debug();
		}
		}
		}
		*/
	}
}