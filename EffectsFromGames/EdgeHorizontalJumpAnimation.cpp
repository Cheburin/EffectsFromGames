#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#undef min // use __min instead
#undef max // use __max instead

extern Character* Eve;
extern IAnimationGraph2 * EveAnimationGraph;
extern char DebugBuffer[1024];
extern World GWorld;
void Debug();

extern JointsRefsChainCollection jointsRefsChains;
extern SimpleMath::Vector3 gravitation;
extern SimpleMath::Vector3 state_ballistic_fly_Start_Location;

Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
void AnimationSetJointT(Animation * anim, int JointNum, SimpleMath::Vector3 Translation);
void rotateHips(Animation *, SimpleMath::Quaternion);
std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);
std::vector<SimpleMath::Vector3>& __AnimGetMetaByTime(AnimationBase* Anim, float Time);
void GetBallisticTrajectoryParams(const float Velocity, const SimpleMath::Vector3& forward, const SimpleMath::Vector3& Start, const SimpleMath::Vector3& Finish, SimpleMath::Vector3 & ballisticG, SimpleMath::Vector3 & ballisticInitialVelocity);
std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index, float Time = 0);

void GetEdgeHorizontalJumpAnimation_ProcessBallisticTrajectory(Animation* __Animation, bool bLeft);

void FixAnimJointsOrientation(Animation* Anim, Animation* RefAnim);

extern bool state_hanging_Toe_activated;
extern std::string state_hanging_Ledge_Name;
extern Box state_hanging_Ledge_Box;

extern char* state_hanging_Toe_Name;
extern SimpleMath::Vector3 state_hanging_Toe_Location;

SimpleMath::Vector3 EdgeHorizontal_DebugLocation;

SimpleMath::Quaternion Inverse(SimpleMath::Quaternion PoseAbsoluteYaw)
{
	SimpleMath::Quaternion Ret;
	PoseAbsoluteYaw.Inverse(Ret);
	return Ret;
}

SimpleMath::Vector3 GetEdgeHorizontalJumpAnimation_vec4ToVec3(SimpleMath::Vector4 v4)
{
	return SimpleMath::Vector3(v4.x, v4.y, v4.z);
}

float GetEdgeHorizontalJumpAnimation_sign(const float& arg){
	return  arg < 0.f ? -1.f : 1.f;
}

SimpleMath::Vector3 GetEdgeHorizontalJumpAnimation_GetHandLocation(bool IsLeft){
	std::vector<JointSQT> ChainsJoints;
	ChainsJoints.resize(256);

	auto HandChain = jointsRefsChains[IsLeft ? 0 : 1];
	JointHelpers::AnimToChain(HandChain, EveAnimationGraph->getPlayingAnimation(), ChainsJoints);
	JointHelpers::localToModel(HandChain.size(), ChainsJoints);

	auto BaseHandLocation = SimpleMath::Vector3::Transform(0.01f*GetEdgeHorizontalJumpAnimation_vec4ToVec3(ChainsJoints[HandChain.size() - 1][2]), SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix());
	
	sprintf(DebugBuffer, "GetEdgeHorizontalJumpAnimation_GetHandLocation %f %f %f\n", BaseHandLocation.x, BaseHandLocation.y, BaseHandLocation.z); Debug();

	return BaseHandLocation;
}

struct EdgeHorizontalJumpAnimation : public Animation
{
	AnimationRep* Impl;

	static const int PoseCount = 2;

	double CurrentTime;
	int CurrentPoseIndex;
	Animation* CurrentPose;

	std::vector<JointSQT> CurrentPoseJoints;
	std::vector<SimpleMath::Vector3> CurrentPoseMetaChannels;

	SimpleMath::Quaternion InitialPoseYaw;

	int HandChain_Count;
	int HandChain_ArmIndex;
	int HandChain_HandIndex;
	std::vector<int> HandChain;
	bool bLeft;

	Animation* Poses[PoseCount];
	SimpleMath::Quaternion PosesYaws[PoseCount];
	float PosesBlendsTimes[PoseCount];

	SimpleMath::Vector3 ballisticOrigin;
	SimpleMath::Vector3 ballisticG;
	SimpleMath::Vector3 ballisticInitialVelocity;
	SimpleMath::Vector3 ballisticPrevLoc;
	SimpleMath::Vector3 ballisticNextLoc;

	std::function<void __cdecl(bool state)> onPlayingChanged;

	SimpleMath::Quaternion BaseCapsuleOrientation;
	SimpleMath::Quaternion DeltaHipsTargetRotation;
	SimpleMath::Vector3    BaseHandLocation;
	SimpleMath::Vector3    BaseToeLocation;

	bool BallisticState_Activated;
	SimpleMath::Vector3    BallisticState_start_capsule_location;
	double				   BallisticState_start_time;

	EdgeHorizontalJumpAnimation()
	{
		Impl = new AnimationRep();

		for (int i = 0; i < PoseCount; i++){ Poses[i] = nullptr; }
	}

	~EdgeHorizontalJumpAnimation()
	{
		for (int i = 0; i < PoseCount; i++){ delete Poses[i]; }

		delete Impl;
	}

	SimpleMath::Vector3 SimulatePath(float Time)
	{
		return Time * ballisticInitialVelocity + 0.5f * (Time * Time) * ballisticG;
	}

	void PostActions()
	{ 
		auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
		auto worldBackSide = state_hanging_Ledge_Box.worldBackSide;
		worldBackSide.Normalize();

		if (!BallisticState_Activated)
		{
			if (CurrentTime < 0.05f) //0.07
			{
				GWorld.Capsules["eve"].origin += (BaseToeLocation - GetHandLocation(FromModelSpaceToWorld, state_hanging_Toe_Name));

				BallisticState_start_capsule_location = GWorld.Capsules["eve"].origin;

			}
			else
			{
				GWorld.Capsules["eve"].origin += (BaseToeLocation - GetHandLocation(FromModelSpaceToWorld, state_hanging_Toe_Name));

				GWorld.Capsules["eve"].origin += worldBackSide.Dot(BallisticState_start_capsule_location - GWorld.Capsules["eve"].origin) * worldBackSide;
			}
		}

		if (!BallisticState_Activated)
		{
			//auto FromModelSpaceToWorld = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();
			//auto Toe = GetHandLocation(FromModelSpaceToWorld, state_hanging_Toe_Name);
			//auto worldBackSide = state_hanging_Ledge_Box.worldBackSide;
			//worldBackSide.Normalize();
			//sprintf(DebugBuffer, "PostBlendActions (%d) Capsule==(%f %f %f)\n", CurrentTime < 0.25f, GWorld.Capsules["eve"].origin.x, GWorld.Capsules["eve"].origin.y, GWorld.Capsules["eve"].origin.z); Debug();// , Toe.x, Toe.y, Toe.z, worldBackSide.x, worldBackSide.y, worldBackSide.z);;  Toe==(%f %f %f) worldBackSide ==(%f %f %f)
		}

	};

	SimpleMath::Vector3 evalBallisticPath()
	{
		if (BallisticState_Activated)
		{
			//GetEdgeHorizontalJumpAnimation_GetHandLocation(bLeft);

			//sprintf(DebugBuffer, "evalBallisticPath(2) %f %f %f\n", GWorld.Capsules["eve"].origin.x, GWorld.Capsules["eve"].origin.y, GWorld.Capsules["eve"].origin.z); Debug();

			ballisticPrevLoc = ballisticNextLoc;

			auto evalBallisticState_time = (CurrentTime - BallisticState_start_time);

			ballisticNextLoc = evalBallisticState_time * ballisticInitialVelocity + 0.5f * (evalBallisticState_time * evalBallisticState_time) * ballisticG;

			return ballisticNextLoc - ballisticPrevLoc;
		}
		//else if (state_hanging_Toe_activated && CurrentPoseIndex == 0 && CurrentTime < 0.25f) //0.07
		//{
		//}
		//else if ((state_hanging_Toe_activated && CurrentPoseIndex == 0 && 0.25f < CurrentTime) || (state_hanging_Toe_activated && CurrentPoseIndex == 1 && CurrentTime < 0.45f)) //0.07
		//{
		//}
		else if (0.25f < CurrentTime)
		{
			BallisticState_start_time = CurrentTime;

			BallisticState_Activated = true;

			ballisticNextLoc = SimpleMath::Vector3::Zero;

			state_hanging_Toe_activated = false;

			GetEdgeHorizontalJumpAnimation_ProcessBallisticTrajectory(this, bLeft);
		}
		return SimpleMath::Vector3::Zero;
	}

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		Impl->prev_frameNo = Impl->frameNo;

		DeltaTranslation = evalBallisticPath();
		DeltaRotation = SimpleMath::Quaternion::Identity;

		//CurrentPose->advanse(elapsedTime, SimpleMath::Vector3(), SimpleMath::Quaternion());
		CurrentJoints = CurrentPoseJoints; //CurrentPose->CurrentJoints;
		CurrentMetaChannels = CurrentPoseMetaChannels; //CurrentPose->CurrentMetaChannels;

		RootSampledRotation = SimpleMath::Quaternion(CurrentJoints[HipsJointIndex][1]);
		CurrentJoints[HipsJointIndex][1] = SimpleMath::Quaternion::Concatenate(
			RootDeltaRotation,
			RootSampledRotation
		);

		Impl->frameNo += 1;
		Impl->global_time += elapsedTime;

		CurrentTime += elapsedTime;
	}

	std::vector<JointSQT>& __AnimGetJointsByTime(float Time)
	{
		return ::__AnimGetJointsByTime(CurrentPose, Time);
	};

	void SetBlendTime(double& blendTime, bool& blendRoot){
		blendRoot = false;
		blendTime = PosesBlendsTimes[CurrentPoseIndex];
	};

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
		CurrentPose->setPlaying(value);
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

		CurrentPose = Poses[CurrentPoseIndex];
		CurrentPose->reset();

		CurrentJoints = CurrentPoseJoints = ::__AnimGetJointsByTime(CurrentPose, .0f);
		CurrentMetaChannels = CurrentPoseMetaChannels = ::__AnimGetMetaByTime(CurrentPose, .0f);
	}

	void resetPosesStates()
	{
		sprintf(DebugBuffer, "EdgeHorizontalJumpAnimation::resetPosesStates"); Debug();

		CurrentTime = 0.0;
		CurrentPoseIndex = 0;

		BallisticState_Activated = false;
		
		DeltaHipsTargetRotation = InitialPoseYaw;
		for (int i = 0; i < EdgeHorizontalJumpAnimation::PoseCount; i++)
		{
			DeltaHipsTargetRotation = SimpleMath::Quaternion::Concatenate(PosesYaws[i], DeltaHipsTargetRotation);
		}

		BaseCapsuleOrientation = GWorld.Capsules["eve"].orientation;
		BaseHandLocation = GetEdgeHorizontalJumpAnimation_GetHandLocation(bLeft);
		BaseToeLocation = state_hanging_Toe_Location;

		//auto h0 = GetAnimationJointsSet(EveAnimationGraph->getAnimationBlend(), 1)[64][2];
		//auto h1 = ::__AnimGetJointsByTime(Poses[0], .0f)[64][2];
		//auto h2 = ::__AnimGetJointsByTime(Poses[1], .0f)[64][2];
		//sprintf(DebugBuffer, "EdgeHorizontalJumpAnimation::resetPosesStates h0 %f %f %f\n", h0.x, h0.y, h0.z); Debug();
		//sprintf(DebugBuffer, "EdgeHorizontalJumpAnimation::resetPosesStates h1 %f %f %f\n", h1.x, h1.y, h1.z); Debug();
		//sprintf(DebugBuffer, "EdgeHorizontalJumpAnimation::resetPosesStates h2 %f %f %f\n", h2.x, h2.y, h2.z); Debug();
	}

	void NextPose()
	{
		CurrentPoseIndex++;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TDeltaRotation GetHipsDelta(Animation* Anim1, Animation* Anim2)
{
	auto V1 = SimpleMath::Matrix::CreateFromQuaternion(__AnimGetJointsByTime(Anim1, 0.f)[HipsJointIndex][1]).Forward();
	auto V2 = SimpleMath::Matrix::CreateFromQuaternion(__AnimGetJointsByTime(Anim2, 0.f)[HipsJointIndex][1]).Forward();
	return TDeltaRotation(V1, V2);
}

void SetRelativeToBase(Animation* Pose, const TDeltaRotation& PoseYaw, SimpleMath::Quaternion& BasePoseYaw)
{
	//rotateHips(NextPose, InverseAbsolutePoseYaw); // вычитаем из hips NextPose поворот yaw Pose, те задаем поворот NextPose относительно Pose
	
	rotateHips(Pose, SimpleMath::Quaternion::Concatenate(PoseYaw.InverseDelta, Inverse(BasePoseYaw))); //теперь yaw NextPose равен нулю
	BasePoseYaw = SimpleMath::Quaternion::Concatenate(PoseYaw.Delta, BasePoseYaw);

	extractAnimationMeta(Pose, true, 1.0f);
	//return DeltaHipsYaw;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Animation* CreateEdgeHorizontalJumpAnimation(bool IsLeft, Animation* Pose, SimpleMath::Quaternion PoseYaw, std::map<std::string, unsigned int> & FramesNamesIndex)
{
	//нам нужно переводить орентацию Hips в идентити, вычислим корректный Hips делта ротэйшин, потом его будет использовать аспект System_UpdateCupsuleRotation
	char* PosePath[EdgeHorizontalJumpAnimation::PoseCount] =
	{
		//"Media\\Animations\\Edge\\Edge_Left_Horizontal_Jump_Pose_1.dae",
		IsLeft ? "Media\\Animations\\Edge\\Edge_Left_Horizontal_Jump_Pose_1.dae" : "Media\\Animations\\Edge\\Edge_Right_Horizontal_Jump_Pose_1.dae",
		IsLeft ? "Media\\Animations\\Edge\\Edge_Left_Horizontal_Jump_Pose_2.dae" : "Media\\Animations\\Edge\\Edge_Right_Horizontal_Jump_Pose_2.dae"
	};
	float BlendsTimes[EdgeHorizontalJumpAnimation::PoseCount] = {
		0.2f,//0.2f,
		0.2f//0.1f//,
		//0.1f 
	};
	Animation* Poses[EdgeHorizontalJumpAnimation::PoseCount] =
	{
		nullptr,
		nullptr
	};
	TDeltaRotation PosesYaws[EdgeHorizontalJumpAnimation::PoseCount];

	{
		Animation* BasePose = Pose;
		for (int i = 0; i < EdgeHorizontalJumpAnimation::PoseCount; i++)
		{
			auto CurrentPose = loadAnimationFromBlender(PosePath[i], FramesNamesIndex);
			if (!IsLeft)
			{
				FixAnimJointsOrientation(CurrentPose, (Animation*)EveAnimationGraph->getAnimation("TPose"));
			}
			extractAnimationMeta(CurrentPose, false, 1.0f);

			rotateHips(CurrentPose, SimpleMath::Quaternion::Concatenate(Inverse(PoseYaw), SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY((IsLeft?-90.f:90.f)*PI / 180.0))));

			Poses[i] = CurrentPose;
			PosesYaws[i] = GetHipsDelta(BasePose, CurrentPose); // поворот hips по yaw от Pose до NextPose 

			BasePose = CurrentPose;
		}
	}

	{
		Animation* BasePose = Pose;
		SimpleMath::Quaternion BasePoseYaw = SimpleMath::Quaternion::Identity;
		for (int i = 0; i < EdgeHorizontalJumpAnimation::PoseCount; i++)
		{
			auto CurrentPose = Poses[i];

			SetRelativeToBase(CurrentPose, PosesYaws[i], BasePoseYaw);

			BasePose = CurrentPose;
		}
	}

	auto Anim = new EdgeHorizontalJumpAnimation();

	Anim->InitialPoseYaw = PoseYaw;

	Anim->bLeft = IsLeft;
	Anim->HandChain = jointsRefsChains[IsLeft ? 0 : 1];
	Anim->HandChain_Count = Anim->HandChain.size();
	Anim->HandChain_ArmIndex = Anim->HandChain_Count - 3;
	Anim->HandChain_HandIndex = Anim->HandChain_Count - 1;

	for (int i = 0; i < EdgeHorizontalJumpAnimation::PoseCount; i++)
	{
		Anim->Poses[i] = Poses[i];
		Anim->PosesYaws[i] = PosesYaws[i].Delta;
		Anim->PosesBlendsTimes[i] = BlendsTimes[i];
	}
	//auto Pose_2 = loadAnimationFromBlender(, FramesNamesIndex);
	//rotateHips(Pose_2, SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationY(-90.0*PI / 180.0)));
	//auto Pose_1_HipsYaw = SetRelativeToBase(Pose_1, Climb_Look_Idle_InvertDelta_Rotation, getSkeletMatrix, calculateFramesTrans);
	//auto Pose_2_HipsYaw = SetRelativeToBase(Pose_2, SimpleMath::Quaternion::Concatenate(Pose_1_HipsYaw.InverseDelta, Climb_Look_Idle_InvertDelta_Rotation), getSkeletMatrix, calculateFramesTrans);
	//EdgeHorizontalJumpAnimationInstance->Pose_1 = Pose_1;
	//EdgeHorizontalJumpAnimationInstance->Pose_1_HipsYaw = Pose_1_HipsYaw;
	//EdgeHorizontalJumpAnimationInstance->Pose_2 = Pose_2;
	//EdgeHorizontalJumpAnimationInstance->Pose_2_HipsYaw = Pose_2_HipsYaw;

	return Anim;
}

Animation* CreateRightEdgeHorizontalJumpAnimation()
{
	return nullptr;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool GetEdgeHorizontalJumpAnimation_FindEdgeFinishLocation(SimpleMath::Vector3 Direction, SimpleMath::Vector3 Origin, SimpleMath::Vector3& EdgeFinishLocation)
{
	Box2 NearestEdge;
	std::string NearestEdge_Name;

	bool Found = false;

	float t = std::numeric_limits<float>::max();

	for (const auto& Ledge : GWorld.Ledges)
	{
		float This_t;

		if (state_hanging_Ledge_Name != Ledge.first && Ledge.second.Boxes[0].Intersect(Origin, 1000.f*Direction, &This_t))
 		{
			if (0.f < This_t && This_t < t)
			{
				t = This_t;
				
				Found = true;

				NearestEdge_Name = Ledge.first;

				NearestEdge = Ledge.second.Boxes[0];
			}
		}
	}
	
	if (Found)
	{
		auto Normal = NearestEdge.worldForward;
		Normal.Normalize();
		Normal = (Normal.Dot(Origin - NearestEdge.origin) < 0.f ? -1.f : 1.f) * Normal;

		EdgeFinishLocation = NearestEdge.origin + (0.5f*NearestEdge.worldForward.Length() - GWorld.Capsules["eve"].r) * Normal;

		sprintf(DebugBuffer, "GetEdgeHorizontalJumpAnimation_FindEdgeFinishLocation (%s)\n", &NearestEdge_Name[0]); Debug();

		EdgeHorizontal_DebugLocation = EdgeFinishLocation;
	}

	return Found;
}

void GetEdgeHorizontalJumpAnimation_RotateChain(std::vector<JointSQT>& ChainJoints, SimpleMath::Quaternion DeltaRot, int ChainJoints_OriginIndex, int ChainJoints_Count)
{
	int i = ChainJoints_OriginIndex;
	const auto Origin = GetEdgeHorizontalJumpAnimation_vec4ToVec3(ChainJoints[i][2]);
	for (; i < ChainJoints_Count; i++)
	{
		auto & ModelJoint = ChainJoints[i];

		auto Loc = SimpleMath::Vector3::Transform(GetEdgeHorizontalJumpAnimation_vec4ToVec3(ModelJoint[2]) - Origin, DeltaRot) + Origin;
		auto Rot = SimpleMath::Quaternion::Concatenate(DeltaRot, ModelJoint[1]);

		ModelJoint[1] = Rot;
		ModelJoint[2] = vec4(Loc, 1);
	}
}

SimpleMath::Quaternion GetEdgeHorizontalJumpAnimation_GetRotation(SimpleMath::Vector3 RotationAxis, SimpleMath::Vector3 Target, std::vector<JointSQT>& ChainJoints, int ChainJoints_OriginIndex, int ChainJoints_HandIndex)
{
	const auto Hand = GetEdgeHorizontalJumpAnimation_vec4ToVec3(ChainJoints[ChainJoints_HandIndex][2]);
	const auto Origin = GetEdgeHorizontalJumpAnimation_vec4ToVec3(ChainJoints[ChainJoints_OriginIndex][2]);
	auto Normal = RotationAxis;
	Normal.Normalize();
	const auto Plane = SimpleMath::Plane(Origin, Normal);

	auto From = Hand - Plane.DotCoordinate(Hand)*Normal - Origin;
	auto To = Target - Plane.DotCoordinate(Target)*Normal - Origin;
	
	if (-0.00001 < From.x && From.x < 0.00001) // y z
	{
		auto UpSquared = From.LengthSquared() - To.z * To.z;
		assert(UpSquared > 0.f && GetEdgeHorizontalJumpAnimation_sign(From.z) == GetEdgeHorizontalJumpAnimation_sign(To.z));
		To = SimpleMath::Vector3(0.f, GetEdgeHorizontalJumpAnimation_sign(From.y)*sqrt(UpSquared), To.z);
		From.x = 0.f;
	}
	else if (-0.00001 < From.z && From.z < 0.00001)  // y x
	{
		auto UpSquared = From.LengthSquared() - To.x * To.x;
		assert(UpSquared > 0.f && GetEdgeHorizontalJumpAnimation_sign(From.x) == GetEdgeHorizontalJumpAnimation_sign(To.x));
		To = SimpleMath::Vector3(To.x, GetEdgeHorizontalJumpAnimation_sign(From.y)*sqrt(UpSquared), 0.f);
		From.z = 0.f;
	}

	float DeltaAngle = 0.f;
	SimpleMath::Quaternion DeltaRotation = SimpleMath::Quaternion::Identity;
	{
		From.Normalize();
		To.Normalize();
		DeltaAngle = atan2(From.Cross(To).Length(), From.Dot(To));
		DeltaRotation = SimpleMath::Quaternion::CreateFromAxisAngle(From.Cross(To), DeltaAngle);
	}
	sprintf(DebugBuffer, "GetEdgeHorizontalJumpAnimation_GetRotation %f %f(%f %f %f  %f %f %f)\n", DeltaAngle, (DeltaAngle / PI)*180.f, From.x, From.y, From.z, To.x, To.y, To.z); Debug(); // 0.066138 3.789412(0.000000 24.868416 -25.826628  0.000000 23.107176 -27.413706)
	return DeltaRotation;
	//return SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, (46.f / 180.f)*PI);
}

SimpleMath::Vector3 GetEdgeHorizontalJumpAnimation_ExtractStartLocation(EdgeHorizontalJumpAnimation* Anim, Capsule TheCapsule, SimpleMath::Vector3 RotationAxis)
{
	const auto FromLSToWS = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * SimpleMath::Matrix::CreateFromQuaternion(Anim->BaseCapsuleOrientation) * SimpleMath::Matrix::CreateTranslation(GWorld.Capsules["eve"].origin);

 	std::vector<JointSQT> ChainsJoints;
 	ChainsJoints.resize(256);

 	Anim->Poses[1]->reset();
 	Anim->Poses[1]->CurrentJoints = ::__AnimGetJointsByTime(Anim->Poses[1], .0f);
	Anim->Poses[1]->CurrentJoints[HipsJointIndex][1] = SimpleMath::Quaternion::Concatenate(Anim->DeltaHipsTargetRotation, Anim->Poses[1]->CurrentJoints[HipsJointIndex][1]);

	JointHelpers::AnimToChain(Anim->HandChain, Anim->Poses[1], ChainsJoints);
	JointHelpers::localToModel(Anim->HandChain_Count, ChainsJoints);

	const auto RotationLS = GetEdgeHorizontalJumpAnimation_GetRotation(
		(1.f / 0.01f)*SimpleMath::Vector3::TransformNormal(RotationAxis, FromLSToWS.Invert()),
		(1.f / 0.01f)*SimpleMath::Vector3::Transform(Anim->BaseHandLocation, FromLSToWS.Invert()),
		ChainsJoints,
		Anim->HandChain_ArmIndex,
		Anim->HandChain_HandIndex
	);

	GetEdgeHorizontalJumpAnimation_RotateChain(ChainsJoints, RotationLS, Anim->HandChain_ArmIndex, Anim->HandChain_Count);
	auto HandLS = 0.01f*GetEdgeHorizontalJumpAnimation_vec4ToVec3(ChainsJoints[Anim->HandChain_HandIndex][2]);

	JointHelpers::modelToLocal(Anim->HandChain_Count, ChainsJoints);

	const auto NewRot = ChainsJoints[Anim->HandChain_ArmIndex][1];
	Anim->Poses[1]->TransformJointSamples(
		Anim->HandChain[Anim->HandChain_ArmIndex],
		"rotation",
		[NewRot](SimpleMath::Vector4 v){
			return NewRot;
		}
	);

	if (Anim->CurrentPoseIndex == 1)
	{
		Anim->CurrentJoints = Anim->CurrentPoseJoints = ::__AnimGetJointsByTime(Anim->CurrentPose, .0f);
		Anim->CurrentMetaChannels = Anim->CurrentPoseMetaChannels = ::__AnimGetMetaByTime(Anim->CurrentPose, .0f);
	}

	return SimpleMath::Vector3::Transform(HandLS, FromLSToWS);
	//return GetHandLocation(FromModelSpaceToWorld, "Hips");
}

SimpleMath::Vector3 GetEdgeHorizontalJumpAnimation_ProjectFinishLocation(SimpleMath::Vector3 EdgeFinishLocation, SimpleMath::Vector3 TheOrigin)
{
	auto TheNormal = state_hanging_Ledge_Box.worldBackSide;
	TheNormal.Normalize();

	auto plane = SimpleMath::Plane(TheOrigin, TheNormal);
 
	return EdgeFinishLocation - plane.DotCoordinate(EdgeFinishLocation)*TheNormal;
}

void GetEdgeHorizontalJumpAnimation_ProcessBallisticTrajectory(Animation* __Animation, bool bLeft)
{
	sprintf(DebugBuffer, "GetEdgeHorizontalJumpAnimation_ProcessBallisticTrajectory\n"); Debug();

	auto Anim = ((EdgeHorizontalJumpAnimation*)__Animation);

	auto LedgeOrigin = state_hanging_Ledge_Box.origin;

	auto BallisticForward = state_hanging_Ledge_Box.worldForward;
	BallisticForward.Normalize();
	BallisticForward = (bLeft ? 1.f : -1.f)*BallisticForward;

	SimpleMath::Vector3 EdgeFinishLocation;

	GetEdgeHorizontalJumpAnimation_FindEdgeFinishLocation(BallisticForward, LedgeOrigin, EdgeFinishLocation);
 
	auto BallisticStart = Anim->ballisticOrigin = GetEdgeHorizontalJumpAnimation_ExtractStartLocation(Anim, GWorld.Capsules["eve"], BallisticForward);
	 
	auto BallisticFinish = GetEdgeHorizontalJumpAnimation_ProjectFinishLocation(EdgeFinishLocation, BallisticStart);
	 
	GetBallisticTrajectoryParams(7.5f, BallisticForward, BallisticStart, BallisticFinish, Anim->ballisticG, Anim->ballisticInitialVelocity);
}

bool GetEdgeHorizontalJumpAnimation_NextPose(IAnimationGraph2 * Graph, Animation* __Animation, bool bStarting)
{
	auto Anim = ((EdgeHorizontalJumpAnimation*)__Animation);

	auto UpdateCapsuleRotation_SetParams = [](EdgeHorizontalJumpAnimation* Anim)
	{
		auto Delta = Anim->PosesYaws[Anim->CurrentPoseIndex];
		Simulation::UpdateCapsuleRotation_SetParams(Delta, SimpleMath::Quaternion::Concatenate(Delta, GWorld.Capsules["eve"].orientation));

		sprintf(DebugBuffer, "GetEdgeHorizontalJumpAnimation_NextPose %d\n", Anim->CurrentPoseIndex); Debug();
	};

	if (bStarting)
	{
		Anim->resetPosesStates();

		Graph->getPlayingAnimation()->CurrentJoints[HipsJointIndex][1] = SimpleMath::Quaternion::Concatenate(Inverse(Anim->InitialPoseYaw), Graph->getPlayingAnimation()->CurrentJoints[HipsJointIndex][1]);
		GWorld.Capsules["eve"].orientation = SimpleMath::Quaternion::Concatenate(Anim->InitialPoseYaw, GWorld.Capsules["eve"].orientation);

		UpdateCapsuleRotation_SetParams(Anim);

		return true;
	}
	else if (!Graph->getAnimationBlend()->isPlaying() && (Anim->CurrentPoseIndex+1) < EdgeHorizontalJumpAnimation::PoseCount)
	{
		Anim->NextPose();

		UpdateCapsuleRotation_SetParams(Anim);

		return true;
	}
	else
	{
		return false;
	}
}

SimpleMath::Vector3 BallisticAnimation_SimulatePath(AnimationBase * Animation, float Time)
{
	auto BallisticAnimation = (EdgeHorizontalJumpAnimation*)Animation;

	return BallisticAnimation->ballisticOrigin + BallisticAnimation->SimulatePath(Time);
}

AnimationBase* EdgeHorizontalJumpAnimation_TestPoses(AnimationBase * Animation, float DeltaTime, int PoseIndex)
{
	SimpleMath::Vector3 deltaTranslation;
	SimpleMath::Quaternion deltaRotation;

	auto Anim = (EdgeHorizontalJumpAnimation*)Animation;

	Anim->Poses[PoseIndex]->advanse(.0f, deltaTranslation, deltaRotation);

	return Anim->Poses[PoseIndex];
}