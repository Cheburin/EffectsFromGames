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
//void blendAnimationInitBlend(Animation* currentAnimation, Animation* _animation1, Animation* _animation2, double local_duration, std::function<double __cdecl(double, double)> _BlendFunction, std::function<std::pair<SimpleMath::Vector3, SimpleMath::Vector3> __cdecl(double, Animation*, Animation*)> _AdvanseFunction);
void initBlend(AnimationLinearBlend* blend, AnimationBase* _animation1, AnimationBase* _animation2, double local_duration,
	std::function<double __cdecl(double, double)> _BlendFunction,
	std::function<SimpleMath::Vector3
	__cdecl(double, double, AnimationBase*, AnimationBase*)> _AdvanseFunction
);
void copyBlendPoseToAnimation1AndInitBlend(AnimationLinearBlend* blend, AnimationBase* _animation1, AnimationBase* _animation2, double local_duration,
	std::function<double __cdecl(double, double)> _BlendFunction,
	std::function<
	SimpleMath::Vector3 __cdecl(double, double, AnimationBase*, AnimationBase*)> _AdvanseFunction
);
void copyCurrentPoseFromAnimation1ToAnimation2(AnimationBase* _animation1, AnimationBase* _animation2);

struct AnimationGraph2;

struct AnimationGraphNode;

struct AnimationGraphNodeCondition
{
	AnimationGraphNode* node;
	std::function<bool __cdecl()> condition; //Animation * src, Animation * dst
};

struct AnimationGraphNode
{
	char * name;
	AnimationBase * animation;
	std::vector<AnimationGraphNodeCondition> nodes;
};

IAnimationGraph2::~IAnimationGraph2()
{

}

namespace Simulation
{
	SimpleMath::Quaternion UpdateCapsuleRotation_GetSubstructedRootRotation(char* _animation1, char* _animation2, float BlendTime);
};
void __BlendSetRoot2Rotation(AnimationBase* blend, SimpleMath::Quaternion RootRotation);

struct AnimationGraph2 : IAnimationGraph2
{
	CharacterSkelet * skelet;

	bool bAnimationBlendActivated;
	AnimationGraphNode * currentNode;
	AnimationGraphNode * prevNode;

	AnimationBase * currentAnimation;
	AnimationBase * animationPoseForBlend;
	AnimationLinearBlend* animationBlend;

	AnimationBase * blendedAnimation2;

	std::vector<AnimationGraphNode> nodes;
	std::map<AnimationBase *, AnimationGraphNode*> animationToNode;

	void registerAnimation(char * name, AnimationBase * animation)
	{
		AnimationGraphNode node = { name, animation };
		nodes.push_back(node);
		animationToNode.insert(std::pair<AnimationBase *, AnimationGraphNode*>(animation, &nodes[nodes.size() - 1]));
	};

	AnimationGraph2Link createLink(AnimationBase * fromAnimation)
	{
		AnimationGraph2Link link = { this, fromAnimation, nullptr };
		return link;
	}

	void registerLink(AnimationBase * fromAnimation, AnimationBase * toAnimation, std::function<bool __cdecl()> forwardCond)
	{
		auto fromAnim = animationToNode.find(fromAnimation);
		auto toAnim = animationToNode.find(toAnimation);

		if (fromAnim == animationToNode.end() || toAnim == animationToNode.end())
			throw "registerLink failed";

		if (forwardCond != nullptr)
		{
			AnimationGraphNodeCondition condition = { toAnim->second, forwardCond };
			fromAnim->second->nodes.push_back(condition);
		}

		//if (reverseCond != nullptr)
		//{
		//	AnimationGraphNodeCondition condition = { fromAnim->second, reverseCond };
		//	toAnim->second->nodes.push_back(condition);
		//}
	};

	void start(AnimationBase * anim)
	{
		auto node = animationToNode.find(anim);
		if (node == animationToNode.end())
			throw "start failed";
		currentAnimation = (prevNode = currentNode = node->second)->animation;

		currentAnimation->setPlaying(true);
	}

	void advanse(double newElipsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		resolveNode();

		auto & animation = currentAnimation;

		animation->advanse(newElipsedTime, DeltaTranslation, DeltaRotation);
	};

	~AnimationGraph2(){
		for (auto p = animationToNode.begin(); p != animationToNode.end(); p++)
			delete (*p).first;

		delete animationPoseForBlend;

		delete animationBlend;
	}

	AnimationGraph2(CharacterSkelet * newSkelet, AnimationLinearBlend * blend, AnimationBase* animationPose)
	{
		nodes.reserve(1024); // very impotent!!! due to link on this memory via create map

		skelet = newSkelet;
		animationBlend = blend;
		animationPoseForBlend = animationPose;

		bAnimationBlendActivated = false;

		currentNode = nullptr;
	}

	char * getAnimationName()
	{
		//return (targetNode != nullptr ? targetNode : currentNode)->name;
		return currentNode->name;
	}

	char * getPrevAnimationName()
	{
		//return (targetNode != nullptr ? targetNode : currentNode)->name;
		return prevNode->name;
	}

	AnimationBase* _getAnimation()
	{
		//return (targetNode != nullptr ? targetNode : currentNode)->animation;
		return currentNode->animation;
	}

	AnimationBase * getAnimation(char* name)
	{
		for (int i = 0; i < nodes.size(); i++)
		{
			if(std::string(name) == nodes[i].name)
			{
				return nodes[i].animation;
			}
		}

		return 0;
	}

	AnimationLinearBlend* getAnimationBlend()
	{
		return animationBlend;
	}

	AnimationBase* getPlayingAnimation()
	{
		if (animationBlend->isPlaying())
		{
			return animationBlend;
		}
		else if (currentNode)
		{
			return currentNode->animation;
		}
		else
		{
			return nullptr;
		}
	}
	
	bool IsBlendActivated()
	{
		return bAnimationBlendActivated;
	}
protected:
	//void generateTransitionBetweenAnimations()
	//{
	//
	//}

	void resolveNode()
	{

		if (bAnimationBlendActivated)
		{
			//transition completed ?
			if (!animationBlend->isPlaying())
			{
				bAnimationBlendActivated = false;

				//now blend is done
				//continue play current after transition completed
				currentAnimation = currentNode->animation;
			}
		}

		for (int i = 0; i < currentNode->nodes.size(); i++)
		{
			auto & conditionNode = currentNode->nodes[i];

			if (conditionNode.condition())//currentNode->animation, currentNode->nodes[i].node->animation))
			{
				prevNode = currentNode;

				currentNode = conditionNode.node;

				double transition_time = 0.5;
				transition_time = std::string(currentNode->name) == "BallisticFly" ? 0.25 : transition_time;
				
				transition_time = std::string(currentNode->name) == "Jump_To_Hang_With_Leg" ? 0.125 : transition_time;
				transition_time = std::string(currentNode->name) == "Jump_To_Hang_WithOut_Leg" ? 0.125 : transition_time;
				
				transition_time = std::string(currentNode->name) == "Freehang_Climb" ? 0.025 : transition_time;
				transition_time = std::string(currentNode->name) == "RightShimmy" ? 0.125 : transition_time;
				transition_time = std::string(currentNode->name) == "LeftShimmy" ? 0.125 : transition_time;
				transition_time = std::string(currentNode->name) == "JumpFromWall" ? 0.4 : transition_time;

				bool transition_deltaTranslation = true;
				transition_deltaTranslation = std::string(currentNode->name) == "BallisticFly" ? false : transition_deltaTranslation;
				transition_deltaTranslation = std::string(currentNode->name) == "RightShimmy" ? false : transition_deltaTranslation;
				transition_deltaTranslation = std::string(currentNode->name) == "LeftShimmy" ? false : transition_deltaTranslation;
				transition_deltaTranslation = std::string(currentNode->name) == "JumpFromWall" ? false : transition_deltaTranslation;

				if (transition_time == 0.0)
				{
					currentAnimation = currentNode->animation;

					currentAnimation->reset();

					currentAnimation->setPlaying(true);
				}
				else if (std::string(currentNode->name) == "FallingAndHangOn")
				{
					copyCurrentPoseFromAnimation1ToAnimation2(currentAnimation, currentNode->animation);

					currentAnimation = currentNode->animation;

					char buffer[1024]; sprintf(buffer, "AnimationGraph resolveNode ----- %s %s\n", prevNode->name, currentNode->name); OutputDebugStringA(buffer);
				}
				else if (!bAnimationBlendActivated)
				{
					bAnimationBlendActivated = true;

					currentAnimation = animationBlend;

					char buffer[1024]; sprintf(buffer, "AnimationGraph resolveNode ----- %s %s\n", prevNode->name, currentNode->name); OutputDebugStringA(buffer);
					
					initBlend(animationBlend, prevNode->animation, blendedAnimation2 = currentNode->animation, transition_time,
						[](double t, double d){
							//if (std::string(conditionNode.node->name) == "hangingIdle")
							//	return 0.0;
							auto k = t / d;
							return k>1.0 ? 1.0 : k;
						},
						[transition_deltaTranslation](double elapsedTime, double k, AnimationBase *a1, AnimationBase *a2){
							SimpleMath::Vector3 deltaTranslation;

							a2->advanse(elapsedTime, deltaTranslation, SimpleMath::Quaternion());

							return (transition_deltaTranslation ? (float)k : 1.f)*deltaTranslation;
						}
					);
					Simulation::UpdateCapsuleRotation_ApplyTargetRootDeltaRotation(currentNode->name, prevNode->name, transition_time);
				}
				else if (bAnimationBlendActivated)
				{
					// sorry, wait until transaction completed
					// but now we try it don't wait until transaction completed
					
					char buffer[1024]; sprintf(buffer, "AnimationGraph resolveNode(*) ----- %s %s\n", prevNode->name, currentNode->name); OutputDebugStringA(buffer);

					blendedAnimation2->setPlaying(false);

					copyBlendPoseToAnimation1AndInitBlend(animationBlend, animationPoseForBlend, blendedAnimation2 = currentNode->animation, transition_time,
						[](double t, double d){
							auto k = t / d;
							return k>1.0 ? 1.0 : k;
						},
						[transition_deltaTranslation](double elapsedTime, double k, AnimationBase *a1, AnimationBase *a2){
							SimpleMath::Vector3 deltaTranslation;

							a2->advanse(elapsedTime, deltaTranslation, SimpleMath::Quaternion());
							
							return (transition_deltaTranslation ? (float)k : 1.f)*deltaTranslation;
						}
					);
					Simulation::UpdateCapsuleRotation_ApplyTargetRootDeltaRotation(currentNode->name, prevNode->name, transition_time);
				};
				//only one travel must be posible so break
				break;
			}
		}
	}

};

IAnimationGraph2 * makeAnimationGraph(CharacterSkelet * skelet, AnimationLinearBlend * transitionAnim, AnimationBase* animationPose)
{
	return new AnimationGraph2(skelet, transitionAnim, animationPose);
}