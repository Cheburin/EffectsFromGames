#include "main.h"
#include "AnimationImpl.h"

AnimationWithImpl::AnimationWithImpl()
{
	Impl = new AnimationRep();
}

AnimationWithImpl::~AnimationWithImpl()
{
	delete Impl;
}

void AnimationWithImpl::TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)	{}

void AnimationWithImpl::TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f) {}

void AnimationWithImpl::setRate(double speed) {};

void AnimationWithImpl::setLooping(bool loop) {}

double AnimationWithImpl::getRate()
{
	return 1;
}

int AnimationWithImpl::getFrame()
{
	return Impl->prev_frameNo;
}

double AnimationWithImpl::getLocTime(){
	return Impl->prev_local_time;
};


bool AnimationWithImpl::IsLoop(){
	return false;
};

void AnimationWithImpl::subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
{
	onPlayingChanged = callback;
}

bool AnimationWithImpl::getPlaying()
{
	return Impl->playing;
}

void AnimationWithImpl::setPlaying(bool value)
{
	if (Impl->playing != value)
	{
		Impl->playing = value;
		if (onPlayingChanged)
			onPlayingChanged(Impl->playing);
	}
}

bool AnimationWithImpl::isPlaying()
{
	return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
}

void AnimationWithImpl::reset()
{
	Impl->prev_frameNo = Impl->frameNo = 0;

	Impl->prev_local_time = Impl->local_time = .0;

	Impl->global_time = .0;

	Impl->playing = false;

	Impl->Rate = 1.f;
}
