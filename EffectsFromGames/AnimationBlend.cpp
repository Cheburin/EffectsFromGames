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

extern char DebugBuffer[1024];
extern World GWorld;
void Debug();
std::vector<JointSQT>& __AnimGetLastJoints(AnimationBase* Anim);
std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time);

AnimationLinearBlend::~AnimationLinearBlend()
{
}

struct Animation3 : public AnimationLinearBlend
{
	AnimationRep3* Impl;
	
	double BlendK;

	Animation3()
	{
		Impl = new AnimationRep3();
	}

	~Animation3()
	{
		delete Impl;
	};

	bool getPlaying()
	{
		return Impl->playing;
	}

	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}

	void setPlaying(bool value)
	{
		if (Impl->playing != value)
		{
			Impl->playing = value;
		}
	}

	void reset()
	{
		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->prev_local_time = Impl->local_time = .0;

		Impl->playing = false;
	}

	SimpleMath::Vector3 getCurrentMeta(int channelId)
	{
		return CurrentMetaChannels[channelId];
	}

	SimpleMath::Vector3 getCurrentMetaOffset(int channelId)
	{
		return CurrentMetaChannels[channelId] - Impl->animation1->CurrentMetaChannels[channelId];
	}

	SimpleMath::Vector3 getCurrentJointTranslationOffset(int jointId)
	{
		auto d = SimpleMath::Vector4(CurrentJoints[jointId][2]) - SimpleMath::Vector4(Impl->animation1->CurrentJoints[jointId][2]);
		SimpleMath::Vector3 r = d;
		return r;
	}

	float getTimeRatio()
	{
		return Impl->BlendFunction(Impl->prev_local_time, Impl->local_duration);
	}

	void initAndSetPlaying(AnimationBase* _animation1, AnimationBase* _animation2, double local_duration, 
		std::function<double __cdecl(double, double)> _BlendFunction, 
		std::function<SimpleMath::Vector3 __cdecl(double, double, AnimationBase*, AnimationBase*)> _AdvanseFunction
	)
	{
		Impl->playing = true;

		Impl->prev_frameNo = 0;

		Impl->frameNo = 0;

		Impl->prev_local_time = .0;

		Impl->local_time = .0;

		Impl->local_duration = local_duration;

		Impl->animation1 = _animation1;

		Impl->animation2 = _animation2;

		Impl->BlendFunction = _BlendFunction;

		Impl->AdvanseFunction = _AdvanseFunction;

		CurrentJoints.resize(Impl->animation1->CurrentJoints.size());

		CurrentMetaChannels.resize(Impl->animation1->CurrentMetaChannels.size());

		for (int i = 0; i < CurrentJoints.size(); i++)
			CurrentJoints[i] = Impl->animation1->CurrentJoints[i];

		for (int i = 0; i < CurrentMetaChannels.size(); i++)
			CurrentMetaChannels[i] = Impl->animation1->CurrentMetaChannels[i];
	}

	void PostActions()
	{ 
		Impl->animation2->PostActions();
	};

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		Impl->prev_frameNo = Impl->frameNo;
		Impl->prev_local_time = Impl->local_time;

		double t = Impl->BlendFunction(Impl->local_time, Impl->local_duration);
		BlendK = t;
		DeltaTranslation = Impl->AdvanseFunction(elapsedTime, t, Impl->animation1, Impl->animation2);
		DeltaRotation = SimpleMath::Quaternion::Identity;

		auto size = CurrentJoints.size();

		for (int i = 0; i < size; i++)
		{
			auto & joint1 = Impl->animation1->CurrentJoints[i];
			auto & joint2 = Impl->animation2->CurrentJoints[i];

			auto & joint = CurrentJoints[i];

			joint[0] = SimpleMath::Vector4::Lerp(joint1[0], joint2[0], t);
			joint[1] = SimpleMath::Quaternion::Lerp(joint1[1], joint2[1], t);
			joint[2] = SimpleMath::Vector4::Lerp(joint1[2], joint2[2], t);
		}

		for (int i = 0; i < 3; i++)
		{
			auto & meta1 = Impl->animation1->CurrentMetaChannels[i];
			auto & meta2 = Impl->animation2->CurrentMetaChannels[i];
			CurrentMetaChannels[i] = SimpleMath::Vector3::Lerp(meta1, meta2, t);
		}

		if (Impl->playing){
			double local_time = Impl->local_time + elapsedTime;

			if (Impl->prev_local_time > Impl->local_duration)
			{
				Impl->playing = false;
			}
			//else
			//{
			Impl->frameNo += 1;
			Impl->local_time = local_time;
			//}
		}
	}
};

//Animation* makeAnimationTransition(Animation* _animation1, Animation* _animation2, double local_duration, std::function<double __cdecl(double, double)> _BlendFunction, std::function<std::pair<SimpleMath::Vector3, SimpleMath::Vector3> __cdecl(double, Animation*, Animation*)> _AdvanseFunction)
//{
//	auto ret = new AnimationLinearTransition();

//	ret->init(_animation1, _animation2, local_duration, _BlendFunction, _AdvanseFunction);

//	return ret;
//}

AnimationLinearBlend* makeBlend()
{
	auto ret = new Animation3();
	return ret;
}

void initBlend(AnimationLinearBlend* blend, AnimationBase* _animation1, AnimationBase* _animation2, double local_duration,
	std::function<double __cdecl(double, double)> _BlendFunction, 
	std::function<SimpleMath::Vector3  __cdecl(double, double, AnimationBase*, AnimationBase*)> _AdvanseFunction
)
{
	auto ret = (Animation3*)blend;

	//reset target animation to set isPlaying true an call subscribers for that events

	ret->initAndSetPlaying(_animation1, _animation2, local_duration, _BlendFunction, _AdvanseFunction);

	_animation1->setPlaying(false);

	_animation2->reset();

	_animation2->setPlaying(true);
}

struct AnimationPose: AnimationBase
{
	AnimationPose()
	{
	}

	~AnimationPose()
	{
	};

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
	}

	bool getPlaying()
	{
		return false;
	}

	void setPlaying(bool value)
	{
	}

	bool isPlaying()
	{
		return false;
	}

	void reset()
	{
	}

};

AnimationBase* makeAnimationPose()
{
	auto ret = new AnimationPose();
	return ret;
}

void copyAnimationPose(AnimationBase* src, AnimationBase* dst)
{
	dst->CurrentJoints = src->CurrentJoints;
	dst->CurrentMetaChannels = src->CurrentMetaChannels;
}

void copyBlendPoseToAnimation1AndInitBlend(AnimationLinearBlend* blend, AnimationBase* _animation1, AnimationBase* _animation2, double local_duration,
	std::function<double __cdecl(double, double)> _BlendFunction,
	std::function<SimpleMath::Vector3 __cdecl(double, double, AnimationBase*, AnimationBase*)> _AdvanseFunction
)
{
	auto ret = (Animation3*)blend;

	_animation1->CurrentJoints = blend->CurrentJoints;
	_animation1->CurrentMetaChannels = blend->CurrentMetaChannels;

	ret->initAndSetPlaying(_animation1, _animation2, local_duration, _BlendFunction, _AdvanseFunction);

	_animation2->reset();

	_animation2->setPlaying(true);
}

void copyCurrentPoseFromAnimation1ToAnimation2(AnimationBase* _animation1, AnimationBase* _animation2)
{
	_animation1->setPlaying(false);

	_animation2->CurrentJoints = _animation1->CurrentJoints;
	_animation2->CurrentMetaChannels = _animation1->CurrentMetaChannels;

	_animation2->reset();
	_animation2->setPlaying(true);
}

double GetBlendK(AnimationBase* blend)
{
	auto ret = (Animation3*)blend;
	return ret->BlendK;
}

std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index, float Time = 0)
{
	static std::vector<JointSQT> Dummy;
	auto ret = (Animation3*)blend;
	if (index == 1)
	{
		return ret->Impl->animation1->CurrentJoints;
	}
	else if (index == 2)
	{
		return __AnimGetJointsByTime(ret->Impl->animation2, Time);
	}
	else
	{
		return Dummy;
	}
}

void __BlendSetAnimation2RootDeltaRotation(AnimationBase* blend, SimpleMath::Quaternion InverseRotation)
{
	void __AnimSetRootDeltaRotation(AnimationBase* Anim, SimpleMath::Quaternion RootRotation);
	auto ret = (Animation3*)blend;
	__AnimSetRootDeltaRotation(ret->Impl->animation2, InverseRotation);
}