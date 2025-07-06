#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern JointsRefsChainCollection jointsRefsChains;
extern SimpleMath::Vector3 gravitation;
extern EvePath g_EvePath;

extern SimpleMath::Vector3 MouseRayOrigin;
extern SimpleMath::Vector3 MouseRayDirection;
extern Box state_hanging_Ledge_Box;

Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);
std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);

extern bool state_ballistic_fly_to_target;
extern bool state_BallisticFly_to_HangingIdle;
extern bool state_BallisticFly_to_HangingIdleWithOutLeg;

namespace Curve
{
	SimpleMath::Vector3 bezier(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float t);
	float arcLength(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float t);
	float getTByS(const SimpleMath::Vector3& a, const SimpleMath::Vector3& b, const SimpleMath::Vector3& c, const SimpleMath::Vector3& d, float s);
}

namespace Simulation
{
	void EnableBallisticFlyIfNeeded(const SimpleMath::Matrix& FromModelSpaceToWorld, const SimpleMath::Vector3& origin, const SimpleMath::Vector3& forward);
}

struct JumpFromWallAnimation : public Animation
{
	Animation* Anim_Current;

	Animation* Anim_JumpForward;

	AnimationRep* Impl;
	
	std::function<void __cdecl(bool state)> onPlayingChanged;

	SimpleMath::Matrix FrameOfReference;
	SimpleMath::Matrix StartEvaTransform;

	SimpleMath::Quaternion OppositeForward;
	SimpleMath::Vector4 HipsLocation;

	SimpleMath::Vector3 A;
	SimpleMath::Vector3 B;
	SimpleMath::Vector3 C;
	SimpleMath::Vector3 D;
	SimpleMath::Vector3* SelectedHandler;

	SimpleMath::Vector3 PrevLocation;

	float CurrentDistanceOnCurve;
	float ConstVelocityOnCurve;

	//float StartYaw;

	JumpFromWallAnimation(std::map<std::string, unsigned int> & FramesNamesIndex,
	std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
	{
		Impl = new AnimationRep();

		Anim_JumpForward = loadAnimation("Media\\Animations\\JumpForward.dae", FramesNamesIndex);
		Anim_JumpForward->setRate(1);
		Anim_JumpForward->setLooping(false);
		extractAnimationMeta(Anim_JumpForward, true, .45f, getSkeletMatrix, calculateFramesTrans);

		CurrentJoints = ::__AnimGetJointsByTime(Anim_JumpForward, 0.f);
	}

	~JumpFromWallAnimation()
	{
		delete Anim_JumpForward;

		delete Impl;
	}

	std::vector<JointSQT>& __AnimGetJointsByTime(float Time)
	{
		return ::__AnimGetJointsByTime(Anim_Current, Time);
	};

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		Impl->prev_frameNo = Impl->frameNo;
		Anim_Current->advanse(elapsedTime, DeltaTranslation, DeltaRotation);

		CurrentJoints = Anim_Current->CurrentJoints;
		CurrentMetaChannels = Anim_Current->CurrentMetaChannels;

		RootSampledRotation = SimpleMath::Quaternion(CurrentJoints[64][1]);
		CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(
			RootDeltaRotation,
			RootSampledRotation
		);
		//CurrentJoints[64][2] = HipsLocation;

		//sprintf(DebugBuffer, "JumpFromWallAnim yaw %f Time %f\n", Quat().decompose0(CurrentJoints[64][1]).yaw0, Impl->global_time); Debug();

		if (!state_ballistic_fly_to_target)
		{
			CurrentDistanceOnCurve += elapsedTime*ConstVelocityOnCurve;
			const auto CurLocation = Curve::bezier(A, B, C, D, Curve::getTByS(A, B, C, D, CurrentDistanceOnCurve));
			DeltaTranslation = CurLocation - PrevLocation;
			DeltaRotation = SimpleMath::Quaternion::Identity;
			PrevLocation = CurLocation;
		}

		Impl->frameNo += 1;
		Impl->global_time += elapsedTime;
	}

	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
	{
		onPlayingChanged = callback;
	}
	bool getPlaying()
	{
		return Impl->playing;
	}
	void setPlaying(bool value)
	{
		Anim_Current->setPlaying(value);
		if (Impl->playing != value)
		{
			Impl->playing = value;
			if (onPlayingChanged)
				onPlayingChanged(Impl->playing);
		}
	}
	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}
	void reset()
	{
		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->prev_local_time = Impl->local_time = .0;

		Impl->global_time = .0;

		Impl->playing = false;

		Impl->Rate = 1.f;

		StartEvaTransform = FrameOfReference = GWorld.Capsules["eve"].getMatrix();
		SimpleMath::Vector3 x = state_hanging_Ledge_Box.worldBackSide; x.Normalize();
		SimpleMath::Vector3 z = state_hanging_Ledge_Box.worldForward; z.Normalize();
		FrameOfReference = SimpleMath::Matrix(
			SimpleMath::Vector4(-x),
			SimpleMath::Vector4(0,1,0,0),
			SimpleMath::Vector4(z),
			SimpleMath::Vector4(FrameOfReference.Translation().x, FrameOfReference.Translation().y, FrameOfReference.Translation().z, 1)
		);

		A = SimpleMath::Vector3(0, 0, 0);
		B = SimpleMath::Vector3(-2.22f, 8.18f, 0);
		C = SimpleMath::Vector3(-6.84, 7.033, 0);
		D = SimpleMath::Vector3(-11.366, -14.3688, 0);

		A = SimpleMath::Vector3::Transform(A, FrameOfReference);
		B = SimpleMath::Vector3::Transform(B, FrameOfReference);
		C = SimpleMath::Vector3::Transform(C, FrameOfReference);
		D = SimpleMath::Vector3::Transform(D, FrameOfReference);

		//auto ToModel = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		//ToModel = ToModel.Invert();
		auto V1 = GWorld.Capsules["eve"].getMatrix().Right();
		auto V2 = x;
		TDeltaRotation DeltaRotation(V1, V2);
		//auto Alfa = atan2(V1.Cross(V2).Length(), V1.Dot(V2));
		//OppositeForward = SimpleMath::Quaternion::CreateFromAxisAngle(V1.Cross(V2), Alfa);
		//sprintf(DebugBuffer, "JumpFromWallAnim Alfa %f\n", Alfa); Debug();
		sprintf(DebugBuffer, "FrameOfReference.Right() %f %f %f\n", -FrameOfReference.Right().x, -FrameOfReference.Right().y, -FrameOfReference.Right().z); Debug();
		//std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index);
		//HipsLocation = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 1)[64][2];
		Simulation::UpdateCapsuleRotation_SetParams(DeltaRotation.Delta, SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(-FrameOfReference.Right(), FrameOfReference.Up(), -FrameOfReference.Backward())));

		Anim_Current = Anim_JumpForward;
		
		{
			const auto Frame = SimpleMath::Matrix(
				SimpleMath::Vector4(x),
				SimpleMath::Vector4(0, 1, 0, 0),
				SimpleMath::Vector4(-z),
				SimpleMath::Vector4(GWorld.Capsules["eve"].getMatrix().Translation().x, GWorld.Capsules["eve"].getMatrix().Translation().y, GWorld.Capsules["eve"].getMatrix().Translation().z, 1)
			);

			Simulation::EnableBallisticFlyIfNeeded(SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * Frame, GWorld.Capsules["eve"].origin, x);

			if (state_ballistic_fly_to_target)
			{
				Anim_Current = static_cast<Animation*>(EveAnimationGraph->getAnimation("BallisticFly"));
			}
		}

		Anim_Current->reset();

		ConstVelocityOnCurve = Curve::arcLength(A, B, C, D, 1.f)/2.f;

		CurrentDistanceOnCurve = 0.f;

		PrevLocation = A;

		//Anim->sample(0.f);
		//StartYaw = Quat().decompose0(Anim->CurrentJoints[64][1]).yaw0;
	}

	double getRate()
	{
		return 1;
	}

	void setRate(double speed)
	{
	};

	int getFrame()
	{
		return Impl->prev_frameNo;
	}

	double getLocTime(){
		return Impl->prev_local_time;
	};

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}

	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
	}

	void setLooping(bool loop)
	{
	}

	bool IsLoop(){
		return false;
	};
};

Animation * CreateJumpFromWallAnimation(std::map<std::string, unsigned int> & FramesNamesIndex, std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
{
	return new JumpFromWallAnimation(FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
}

SimpleMath::Matrix GetJumpFromWallStartTransform(AnimationBase *Anim)
{
	JumpFromWallAnimation* JumpFromWallAnim = (JumpFromWallAnimation*)Anim;
	return JumpFromWallAnim->StartEvaTransform;
}

void FillJumpFromWallTrajectory(Animation *Anim)
{
	JumpFromWallAnimation* JumpFromWallAnim = (JumpFromWallAnimation*)Anim;
	auto FrameOfReference = SimpleMath::Matrix::Identity; //JumpFromWallAnim->FrameOfReference;//
	g_EvePath.Count = 32;
	for (int t = 0; t < 32; t++)
	{
		const auto Vector = Curve::bezier(
			SimpleMath::Vector3::Transform(JumpFromWallAnim->A, FrameOfReference),
			SimpleMath::Vector3::Transform(JumpFromWallAnim->B, FrameOfReference),
			SimpleMath::Vector3::Transform(JumpFromWallAnim->C, FrameOfReference),
			SimpleMath::Vector3::Transform(JumpFromWallAnim->D, FrameOfReference),
			t / 31.f
		);
		g_EvePath.Pivots[t] = SimpleMath::Vector4(Vector.x, Vector.y, Vector.z, 1.f);
	}
}

float CatchJumpFromWallTrajectoryHandler(SimpleMath::Matrix& FrameOfReference, SimpleMath::Vector3 a)
{
	const auto r = 0.125f;
	a = SimpleMath::Vector3::Transform(a, FrameOfReference);
	float t = (a - MouseRayOrigin).Dot(MouseRayDirection) / MouseRayDirection.Dot(MouseRayDirection);
	return ((MouseRayOrigin - a) + t*MouseRayDirection).LengthSquared() < r*r;
}

SimpleMath::Vector3 GetJumpFromWallTrajectoryHandlerLocation(Animation *Anim, int index)
{
	//FillJumpFromWallTrajectory(Anim);//для простоты, но лучше так не надо

	JumpFromWallAnimation* JumpFromWallAnim = (JumpFromWallAnimation*)Anim;
	auto FrameOfReference = SimpleMath::Matrix::Identity; //JumpFromWallAnim->FrameOfReference;//
	if (index == 0)
		return SimpleMath::Vector3::Transform(JumpFromWallAnim->A, FrameOfReference);
	else if (index == 1)
		return SimpleMath::Vector3::Transform(JumpFromWallAnim->B, FrameOfReference);
	else if (index == 2)
		return SimpleMath::Vector3::Transform(JumpFromWallAnim->C, FrameOfReference);
	else if (index == 3)
		return SimpleMath::Vector3::Transform(JumpFromWallAnim->D, FrameOfReference);
}

void SelectedJumpFromWallTrajectoryHandler(Animation *Anim)
{
	JumpFromWallAnimation* JumpFromWallAnim = (JumpFromWallAnimation*)Anim;
	JumpFromWallAnim->SelectedHandler = nullptr;
	if (CatchJumpFromWallTrajectoryHandler(JumpFromWallAnim->FrameOfReference, JumpFromWallAnim->A))
	{
		JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->A;
		sprintf(DebugBuffer, "JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->A\n"); Debug();
	}
	if (CatchJumpFromWallTrajectoryHandler(JumpFromWallAnim->FrameOfReference, JumpFromWallAnim->B))
	{
		JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->B;
		sprintf(DebugBuffer, "JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->B\n"); Debug();
	}
	if (CatchJumpFromWallTrajectoryHandler(JumpFromWallAnim->FrameOfReference, JumpFromWallAnim->C))
	{
		JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->C;
		sprintf(DebugBuffer, "JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->C\n"); Debug();
	}
	if (CatchJumpFromWallTrajectoryHandler(JumpFromWallAnim->FrameOfReference, JumpFromWallAnim->D))
	{
		JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->D;
		sprintf(DebugBuffer, "JumpFromWallAnim->SelectedHandler = &JumpFromWallAnim->D\n"); Debug();
	}
}

void UpdateJumpFromWallTrajectoryHandler(Animation *Anim)
{
	JumpFromWallAnimation* JumpFromWallAnim = (JumpFromWallAnimation*)Anim;
	
	if (JumpFromWallAnim->SelectedHandler != nullptr)
	{
		const auto Pl = SimpleMath::Plane(JumpFromWallAnim->FrameOfReference.Translation(), JumpFromWallAnim->FrameOfReference.Backward());
		float t = -(Pl.DotCoordinate(MouseRayOrigin) / Pl.DotNormal(MouseRayDirection));
		auto PlPoint = MouseRayOrigin + t * MouseRayDirection;

		sprintf(DebugBuffer, "JumpFromWallAnim UpdateJumpFromWallTrajectoryHandler %f %f %f\n", PlPoint.x, PlPoint.y, PlPoint.z); Debug();

		PlPoint = SimpleMath::Vector3::Transform(PlPoint, JumpFromWallAnim->FrameOfReference.Invert());

		*JumpFromWallAnim->SelectedHandler = PlPoint;

		FillJumpFromWallTrajectory(Anim);
	}
}