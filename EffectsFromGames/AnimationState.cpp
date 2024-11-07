#include "main.h"
#include "AnimationImpl.h"
#include <fstream>

// #include <assimp/Importer.hpp>      // C++ importer interface
// #include <assimp/scene.h>           // Output data structure
// #include <assimp/postprocess.h>     // Post processing flags

#include <locale>
#include <codecvt>
#include <string>
#include <array>
#include <map>
#include <locale> 

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);

extern SimpleMath::Vector3 gravitation;

IAnimationGraph::~IAnimationGraph()
{

}

struct AnimationGraph : IAnimationGraph
{
	SimpleMath::Vector3 saved_gravitation;
	AnimationState state;
	AnimationState next_state;
	std::map<AnimationState, Animation *> animations;
	CharacterSkelet * skelet;
	bool isPlaying;

	~AnimationGraph(){
		for (auto p = animations.begin(); p != animations.end(); p++)
			delete (*p).second;
	}
	AnimationGraph(std::map<AnimationState, Animation *>  newAnimations, CharacterSkelet * newSkelet){
		saved_gravitation = gravitation;
		state = AnimationState::idle;
		animations = newAnimations;
		skelet = newSkelet;
		isPlaying = true;
	}
	void advanse(double newElipsedTime, SimpleMath::Vector3 & deltaTranslation){
		auto & animation = animations[state];

		//animation->advanse(isPlaying ? newElipsedTime : 0.0f, deltaTranslation);

		for (int i = 0; i < animation->CurrentJoints.size(); i++)
		{
			auto & cj = animation->CurrentJoints[i];

			(*GetSkeletonMatrix(skelet, i)) = cj.matrix();
		}

		if (isPlaying){
			checkStatus();
		}
	}
	void checkStatus(){
		auto & animation = animations[state];

		//if (!animation->isPlaying())
		{
			if (state == AnimationState::walking_to_jump && next_state == AnimationState::jump)
			{
				state = AnimationState::jump;
				animations[next_state = AnimationState::jump_to_walking]->reset();
				return;
			}
			if (state == AnimationState::jump && next_state == AnimationState::jump_to_walking)
			{
				state = AnimationState::jump_to_walking;
				next_state = AnimationState::walking;
				return;
			}
			if (state == AnimationState::jump_to_walking && next_state == AnimationState::walking)
			{
				gravitation += saved_gravitation;
				state = AnimationState::walking;
				return;
			}

			if (state == AnimationState::idle_to_jump && next_state == AnimationState::jump)
			{
				state = AnimationState::jump;
				animations[next_state = AnimationState::jump_to_idle]->reset();
				return;
			}
			if (state == AnimationState::jump && next_state == AnimationState::jump_to_idle)
			{
				state = AnimationState::jump_to_idle;
				next_state = AnimationState::idle;
				return;
			}
			if (state == AnimationState::jump_to_idle && next_state == AnimationState::idle)
			{
				gravitation += saved_gravitation;
				state = AnimationState::idle;
				return;
			}

			if (state == AnimationState::idle_to_walking && next_state == AnimationState::walking)
			{
				state = AnimationState::walking;
				return;
			}

			if (state == AnimationState::walking_to_idle && next_state == AnimationState::idle)
			{
				state = AnimationState::idle;
				return;
			}

			if (state == AnimationState::idle_to_jumpUp && next_state == AnimationState::jumpUp)
			{
				state = AnimationState::jumpUp;
				animations[next_state = AnimationState::jumpUp_to_idle]->reset();
				return;
			}
			if (state == AnimationState::jumpUp && next_state == AnimationState::jumpUp_to_idle)
			{
				state = AnimationState::jumpUp_to_idle;
				animations[next_state = AnimationState::idle]->reset();
				return;
			}
			if (state == AnimationState::jumpUp_to_idle && next_state == AnimationState::idle)
			{
				gravitation += saved_gravitation;
				state = AnimationState::idle;
				return;
			}
			if (state == AnimationState::climbing && next_state == AnimationState::idle)
			{
				gravitation += saved_gravitation;
				state = AnimationState::idle;
				return;
			}
		}
	}
	void setState(AnimationState newState){
		if (state == AnimationState::idle && newState == AnimationState::walking)
		{
			animations[state = AnimationState::idle_to_walking]->reset();
			animations[next_state = AnimationState::walking]->reset();
		}
		if (state == AnimationState::walking && newState == AnimationState::idle)
		{
			animations[state = AnimationState::walking_to_idle]->reset();
			animations[next_state = AnimationState::idle]->reset();
		}
		if (state == AnimationState::walking && newState == AnimationState::jump)
		{
			gravitation -= saved_gravitation;
			animations[state = AnimationState::walking_to_jump]->reset();
			animations[next_state = AnimationState::jump]->reset();
		}
		if (state == AnimationState::jump && newState == AnimationState::walking)
		{
			gravitation += saved_gravitation;
			animations[state = AnimationState::jump_to_walking]->reset();
			animations[next_state = AnimationState::walking]->reset();
		}
		if (state == AnimationState::idle && newState == AnimationState::jump)
		{
			gravitation -= saved_gravitation;
			animations[state = AnimationState::idle_to_jump]->reset();
			animations[next_state = AnimationState::jump]->reset();
		}
		if (state == AnimationState::jump && newState == AnimationState::idle)
		{
			gravitation += saved_gravitation;
			animations[state = AnimationState::jump_to_idle]->reset();
			animations[next_state = AnimationState::idle]->reset();
		}

		if (state == AnimationState::idle && newState == AnimationState::jumpUp)
		{
			gravitation -= saved_gravitation;
			animations[state = AnimationState::idle_to_jumpUp]->reset();
			animations[next_state = AnimationState::jumpUp]->reset();
		}

		if (state == AnimationState::jumpUp && newState == AnimationState::jumpUp)
		{
			isPlaying = true;
			animations[state = AnimationState::jumpUp]->reset();
			animations[next_state = AnimationState::jumpUp_to_idle]->reset();
		}

		if (state == AnimationState::jumpUp && newState == AnimationState::climbing)
		{
			isPlaying = true;
			animations[state = AnimationState::climbing]->reset();
			animations[next_state = AnimationState::idle]->reset();
		}
	}
	AnimationState getState(){
		return state;
	}
	Animation* gatAnimationByState(AnimationState state){
		return animations[state];
	}

	void setPlaying(bool value){
		isPlaying = value;
	}

	bool IsPlaying(){
		return isPlaying;
	}
};

IAnimationGraph * __makeAnimationGraph(std::map<AnimationState, Animation *> newAnimations, CharacterSkelet * skelet)
{
	return new AnimationGraph(newAnimations, skelet);
}

void disposeAnimationGraph(IAnimationGraph2* gr)
{
	delete gr;
}