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
extern SimpleMath::Vector3 state_ballistic_fly_Start_Location;

struct BallisticFlyAnimation : public Animation
{
	AnimationRep* Impl;

	std::function<void __cdecl(bool state)> onPlayingChanged;

	SimpleMath::Vector3 ballisticG;
	SimpleMath::Vector3 ballisticInitialVelocity;

	SimpleMath::Vector3 ballisticPrevLoc;
	SimpleMath::Vector3 ballisticNextLoc;

	std::vector<JointSQT> JointsByPose[2];
	std::vector<SimpleMath::Vector3> MetaChannelsByPose[2];

	SimpleMath::Vector3 SimulatePath(float Time)
	{
		return Time * ballisticInitialVelocity + 0.5f * (Time * Time) * ballisticG;
	}

	SimpleMath::Vector3 evalBallisticPath()
	{
		ballisticPrevLoc = ballisticNextLoc;

		ballisticNextLoc = Impl->local_time * ballisticInitialVelocity + 0.5f * (Impl->local_time * Impl->local_time) * ballisticG;

		return ballisticNextLoc - ballisticPrevLoc;
	}

	BallisticFlyAnimation(std::map<std::string, unsigned int> & FramesNamesIndex,
		std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
	{
		Animation* makeAnimationTransition(Animation* _animation1, Animation* _animation2, double local_duration, std::function<double __cdecl(double, double)> _BlendFunction, std::function<std::pair<SimpleMath::Vector3, SimpleMath::Vector3> __cdecl(double, Animation*, Animation*)> _AdvanseFunction);
		Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr);
		Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex);
		void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations);

		Impl = new AnimationRep();

		float signs[] = { -1.f, 1.f };
		char* ActionPoseFilePathes[] = { "Media\\Animations\\ActionPose_L.dae", "Media\\Animations\\ActionPose_R.dae" };

		for (int i = 0; i < 2; i++)
		{
			Animation* ActionPose = loadAnimation(ActionPoseFilePathes[i], FramesNamesIndex);
			ActionPose->TransformJointSamples(
				64,
				"rotation",
				[&signs, i](SimpleMath::Vector4 v){
					auto RootRotation = SimpleMath::Quaternion(v.x, v.y, v.z, v.w);
					auto RootFrame = SimpleMath::Matrix::CreateFromQuaternion(RootRotation);
					auto DeltaRotation3 = SimpleMath::Quaternion::CreateFromAxisAngle(SimpleMath::Vector3(1, 0, 0), (PI * 30) / 180.0);
					auto V1 = RootFrame.Forward(); V1.y = 0; V1.Normalize();
					auto V2 = SimpleMath::Vector3(0, 0, -1); V2.y = 0; V2.Normalize();
					auto DeltaRotation2 = SimpleMath::Quaternion::CreateFromAxisAngle(V1.Cross(V2), atan2(V1.Cross(V2).Length(), V1.Dot(V2)));
					auto DeltaRotation1 = SimpleMath::Quaternion::CreateFromAxisAngle(RootFrame.Forward(), signs[i]*(PI * 35) / 180.0);

					return SimpleMath::Quaternion::Concatenate(DeltaRotation3,
						   SimpleMath::Quaternion::Concatenate(DeltaRotation2, 
						   SimpleMath::Quaternion::Concatenate(DeltaRotation1, 
						   RootRotation)));
			});
			ActionPose->TransformJointSamples(
				64,
				"translation",
				[](SimpleMath::Vector4 v){
					v = SimpleMath::Vector4(v.x, v.y + 40.0f, v.z, 0);
					return v;
			});
			extractAnimationMeta(ActionPose, true, 1.0f, getSkeletMatrix, calculateFramesTrans);

			ActionPose->advanse(0, SimpleMath::Vector3(), SimpleMath::Quaternion());
			JointsByPose[i] = ActionPose->CurrentJoints;
			MetaChannelsByPose[i] = ActionPose->CurrentMetaChannels;

			delete ActionPose;
		}

		CurrentJoints = JointsByPose[0];
		CurrentMetaChannels = MetaChannelsByPose[0];
	}

	~BallisticFlyAnimation()
	{
		delete Impl;
	}

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		const auto modelTransform = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix();

		Impl->prev_frameNo = Impl->frameNo;
		Impl->prev_local_time = Impl->local_time;

		DeltaRotation = SimpleMath::Quaternion::Identity;
		DeltaTranslation = SimpleMath::Vector3::TransformNormal(evalBallisticPath(), modelTransform);

		Impl->global_time += elapsedTime;
		Impl->frameNo += 1;
		Impl->local_time = Impl->Rate * Impl->global_time;
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

		ballisticPrevLoc = SimpleMath::Vector3::Zero;
			
		ballisticNextLoc = SimpleMath::Vector3::Zero;
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

Animation * CreateBallisticFlyAnimation(std::map<std::string, unsigned int> & FramesNamesIndex,	std::function<SimpleMath::Matrix* __cdecl(unsigned int)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTrans)
{
	return new BallisticFlyAnimation(FramesNamesIndex, getSkeletMatrix, calculateFramesTrans);
}

void SetBallisticTrajectoryParams(AnimationBase * Animation, SimpleMath::Vector3 & ballisticG, SimpleMath::Vector3 & ballisticInitialVelocity)
{
	auto BallisticAnimation = (BallisticFlyAnimation*)Animation;

	BallisticAnimation->ballisticG = ballisticG;
	BallisticAnimation->ballisticInitialVelocity = ballisticInitialVelocity;
}

SimpleMath::Vector3 GetBallisticAnimationHandLocation(AnimationBase * Animation, int CatherHandIndex)
{
	auto BallisticAnimation = (BallisticFlyAnimation*)Animation;

	return BallisticAnimation->MetaChannelsByPose[CatherHandIndex][CatherHandIndex + 1];
}

void SetBallisticAnimationCatherHand(AnimationBase * Animation, int CatherHandIndex)
{
	auto BallisticAnimation = (BallisticFlyAnimation*)Animation;

	BallisticAnimation->CurrentJoints = BallisticAnimation->JointsByPose[CatherHandIndex];
	BallisticAnimation->CurrentMetaChannels = BallisticAnimation->MetaChannelsByPose[CatherHandIndex];
}

//SimpleMath::Vector3 BallisticAnimation_SimulatePath(AnimationBase * Animation, float Time)
//{
	//auto BallisticAnimation = (BallisticFlyAnimation*)Animation;
//
//	//return BallisticAnimation->SimulatePath(Time);
//}
