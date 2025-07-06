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

template<class T> bool XorSign(T a, T b)
{
	return (a < .0 && b > .0) || (b < .0 && a > .0);
}

AnimationBase::~AnimationBase(){

}

void Debug();
extern World GWorld;
extern char DebugBuffer[1024];

struct Animation2 : public Animation
{
	AnimationRep2* Impl;

	Animation2()
	{
		Impl = new AnimationRep2();
	}

	~Animation2()
	{
		delete Impl;
	};

	void init(unsigned int size, double rate, double local_duration)
	{
		Impl->Rate = rate;

		Impl->local_duration = local_duration;

		Impl->global_duration = fabs(1.0 / rate) * local_duration;

		Impl->setJointsCount(size);

		Impl->setMetaChannelsCount(3);

		CurrentJoints.resize(size);

		CurrentMetaChannels.resize(3);
	}

	void reset()
	{
		Impl->prev_frameNo = Impl->frameNo = 0;

		Impl->prev_local_time = Impl->local_time = .0;

		Impl->global_time = .0;

		Impl->loop_counter = 0;

		Impl->prev_loop_counter = 0;

		Impl->playing = false;

		Impl->prev_translation = Impl->Rate<0.f ? Impl->end_translation : Impl->begin_translation;

		Impl->local_time = Impl->Rate < 0.f ? Impl->local_duration : 0.f;

		for (int i = 0; i < Impl->jointsCount; i++)
		{
			Impl->getJointSQT(i, Impl->local_time, CurrentJoints[i]);
		}
	}

	//////subscribe
	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback)
	{
		Impl->onPlayingChanged = callback;
	}

	void setPlaying(bool value)
	{
		if (Impl->playing != value)
		{
			Impl->playing = value;
			if (Impl->onPlayingChanged)
				Impl->onPlayingChanged(value);
		}
	}

	void setLooping(bool loop){
		Impl->looped = loop;
	}

	void setRate(double speed)
	{
		Impl->Rate = speed;

		Impl->global_duration = Impl->local_duration / fabs(Impl->Rate);

		Impl->global_time = Impl->local_time / Impl->Rate;
	}

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
		Impl->TransformMetaSamples(channelId, f);
	}

	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f)
	{
		if (std::string(sampleKind) == "translation")
			Impl->TransformJointTranslationSamples(jointId, f);
		else if (std::string(sampleKind) == "rotation")
			Impl->TransformJointRotationSamples(jointId, f);
	}

	//////get
	SimpleMath::Vector3 GetOffset()
	{
		return Impl->offset;
	}

	int getFrame()
	{
		return Impl->prev_frameNo;
	}

	bool getPlaying()
	{
		return Impl->playing;
	}

	bool IsLoop()
	{
		return Impl->prev_local_time == 0.0 || Impl->prev_loop_counter > 0;
	}

	bool isPlaying()
	{
		return Impl->playing || Impl->frameNo != Impl->prev_frameNo;
	}

	double getRate()
	{
		return Impl->Rate;
	}

	double getLocTime()
	{
		return Impl->prev_local_time;
	}

	///// inner implementation
	void computeLocalTime()
	{
		Impl->frameNo += 1;
		Impl->local_time = Impl->Rate * Impl->global_time;
		if (Impl->local_time < .0){
			Impl->local_time = Impl->local_duration + Impl->local_time;
		}
	}
	
	void preComputeRange(double local_range)
	{
		if (Impl->looped){
			if (local_range < .0)
			{
				Impl->a_translation = Impl->end_translation;
				Impl->b_translation = Impl->begin_translation;
				Impl->ab_translation = Impl->b_translation - Impl->a_translation;
				if (Impl->prev_translation == Impl->begin_translation)
				{
					Impl->prev_translation = Impl->end_translation;
				}
			}
			else
			{
				Impl->a_translation = Impl->begin_translation;
				Impl->b_translation = Impl->end_translation;
				Impl->ab_translation = Impl->b_translation - Impl->a_translation;
				if (Impl->prev_translation == Impl->end_translation)
				{
					Impl->prev_translation = Impl->begin_translation;
				}
			}
			for (int i = 0; i < Impl->jointsCount; i++)
			{
				Impl->resetSampleIndex(i, local_range);
			}
			for (int i = 0; i < 3; i++)
			{
				Impl->resetMetaSampleIndex(i, local_range);
			}
		}
	}

	void sample(double elapsedTime)
	{
		for (int i = 0; i < Impl->jointsCount; i++)
		{
			Impl->getJointSQT(i, elapsedTime, CurrentJoints[i]);
		}
	}

	SimpleMath::Vector3 ModelSpaceDeltaTranslation;

	void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation)
	{
		Impl->prev_frameNo = Impl->frameNo;
		Impl->prev_local_time = Impl->local_time;
		Impl->prev_loop_counter = Impl->loop_counter;

		for (int i = 0; i < Impl->jointsCount; i++)
		{
			Impl->getJointSQT(i, Impl->local_time, CurrentJoints[i]);
		}
		RootSampledRotation = CurrentJoints[64][1];
		CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(
			RootDeltaRotation,
			RootSampledRotation
		);
		for (int i = 0; i < 3; i++)
		{
			Impl->getMetaSample(i, Impl->local_time, CurrentMetaChannels[i]);
		}
		SimpleMath::Vector3 current_translation = CurrentMetaChannels[0];

		{
			SimpleMath::Vector3 delta = SimpleMath::Vector3::Zero;
			if (Impl->frameNo > 0)
			{
				if (Impl->loop_counter){
					delta = Impl->b_translation - Impl->prev_translation;
					Impl->loop_counter--;
					if (Impl->loop_counter)
						delta += Impl->loop_counter * Impl->ab_translation;
					delta += current_translation - Impl->a_translation;
				}
				else
				{
					delta = current_translation - Impl->prev_translation;
				}
				Impl->prev_translation = current_translation;
			}
			ModelSpaceDeltaTranslation = delta;
			DeltaRotation = SimpleMath::Quaternion::Identity;
			const auto FromModelToWorldSpace = GWorld.WorldTransforms["eveSkinnedModel"] * GWorld.Capsules["eve"].getMatrix();
			DeltaTranslation = SimpleMath::Vector3::TransformNormal(ModelSpaceDeltaTranslation, FromModelToWorldSpace);
		}

		if (Impl->playing && Impl->Rate != .0){
			Impl->loop_counter = 0;

			double prev_global_time = Impl->global_time;
			double global_time = Impl->global_time + elapsedTime;

			if (!Impl->looped){
				if ( XorSign(prev_global_time, global_time) || fabs(global_time) > Impl->global_duration ){
					setPlaying(false);
				}
				else{
					Impl->global_time = global_time;
					computeLocalTime();
				}
			}
			else{
				if ( XorSign(prev_global_time, global_time) ){
					Impl->loop_counter += 1;
				};
				if (fabs(global_time) > Impl->global_duration){
					double loop = global_time / Impl->global_duration;
					int trunc_loop = loop;
					Impl->loop_counter += abs(trunc_loop);
					global_time = (loop - trunc_loop) * Impl->global_duration;
				}
				Impl->global_time = global_time;
				computeLocalTime();
				preComputeRange(Impl->Rate * elapsedTime);
			}
		}
	};

	void setCurrentAnimationTime(float global_time)
	{
		Impl->global_time = global_time;
		computeLocalTime();
		preComputeRange(Impl->Rate * 1.f);

		{
			auto TempCurrentMetaChannels = CurrentMetaChannels;
			Impl->getMetaSample(0, Impl->local_time, TempCurrentMetaChannels[0]);
			Impl->prev_translation = TempCurrentMetaChannels[0];
		}
	}

};

void __AnimSetRootDeltaRotation(AnimationBase* Anim, SimpleMath::Quaternion RootRotation)
{
	Anim->RootDeltaRotation = RootRotation;
	Anim->RootSampledRotation = Anim->CurrentJoints[64][1];
	Anim->CurrentJoints[64][1] = SimpleMath::Quaternion::Concatenate(
		Anim->RootDeltaRotation,
		Anim->RootSampledRotation
	);
}

void __AnimResampleCurrentRootRotation(AnimationBase* Anim)
{
	Anim->CurrentJoints[64][1] = Anim->RootSampledRotation;
}

std::vector<JointSQT>& __AnimGetJointsByTime(AnimationBase* Anim, float Time)
{
	static std::vector<JointSQT> Joints;
	if (auto ret = dynamic_cast<Animation2*>(Anim))
	{
		Joints.resize(ret->Impl->jointsCount);

		for (int i = 0; i < ret->Impl->jointsCount; i++)
		{
			ret->Impl->resetSampleIndex(i, ret->Impl->Rate);
		}

		auto SampleAtTime = min(ret->Impl->global_duration, max(0.f, Time));
		SampleAtTime = ret->Impl->Rate * Time;
		if (SampleAtTime < .0){
			SampleAtTime = ret->Impl->local_duration + SampleAtTime;
		}

		for (int i = 0; i < ret->Impl->jointsCount; i++)
		{
			ret->Impl->getJointSQT(i, SampleAtTime, Joints[i]);
		}

		for (int i = 0; i < ret->Impl->jointsCount; i++)
		{
			ret->Impl->resetSampleIndex(i, ret->Impl->Rate);
		}

		return Joints;
	}
	return Anim->__AnimGetJointsByTime(Time);
}

std::vector<SimpleMath::Vector3>& __AnimGetMetaByTime(AnimationBase* Anim, float Time)
{
	static std::vector<SimpleMath::Vector3> Meta;
	if (auto ret = dynamic_cast<Animation2*>(Anim))
	{
		Meta.resize(3);

		for (int i = 0; i < 3; i++)
		{
			ret->Impl->resetMetaSampleIndex(i, ret->Impl->Rate);
		}

		auto SampleAtTime = min(ret->Impl->global_duration, max(0.f, Time));
		SampleAtTime = ret->Impl->Rate * Time;
		if (SampleAtTime < .0){
			SampleAtTime = ret->Impl->local_duration + SampleAtTime;
		}

		for (int i = 0; i < 3; i++)
		{
			ret->Impl->getMetaSample(i, SampleAtTime, Meta[i]);
		}

		for (int i = 0; i < 3; i++)
		{
			ret->Impl->resetMetaSampleIndex(i, ret->Impl->Rate);
		}

		return Meta;
	}
	assert(false);
	return Meta;
}

SimpleMath::Quaternion __AnimSubstructRootDeltaRotation(AnimationBase* Anim)
{
	if (auto ret = dynamic_cast<Animation2*>(Anim))
	{
		auto V1 = SimpleMath::Vector3(0,0,-1);
		auto V2 = SimpleMath::Matrix::CreateFromQuaternion(__AnimGetJointsByTime(Anim, 0.f)[64][1]).Forward();

		TDeltaRotation DeltaRotation(V1, V2);
		auto InverseDelta = DeltaRotation.InverseDelta;

		ret->Impl->TransformJointRotationSamples(64, [InverseDelta](SimpleMath::Vector4 JointRotaion){
			return SimpleMath::Quaternion::Concatenate(InverseDelta, JointRotaion);
		});

		return DeltaRotation.Delta;
	}
}

SimpleMath::Matrix aiMatrixToSimpleMathMatrix(const aiMatrix4x4& aiMe);

/////
bool _replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}
/////

void collectBones(aiNode * node, AnimationRep2 * anim, std::vector<JointSQT> * Joints, std::map<std::string, unsigned int> * FramesNamesIndex, char * replace)
{
	for (int i = 0; i < node->mNumChildren; i++)
	{
		//get Childeren node 
		auto ChildrenNode = node->mChildren[i];

		std::string key(ChildrenNode->mName.C_Str());

		if (replace)
		{
			_replace(key, replace, std::string(""));
		}

		auto pFrameIndex = FramesNamesIndex->find(key);
		if (pFrameIndex == FramesNamesIndex->end())
		{
			throw "";
		}

		auto index = pFrameIndex->second;

		anim->setJoint(index, aiMatrixToSimpleMathMatrix(ChildrenNode->mTransformation));
		anim->getJointSQT(index, (*Joints)[index]);

		// deeper and deeper
		collectBones(ChildrenNode, anim, Joints, FramesNamesIndex, replace);
	}
}

void  __beginAnimationLoad(const char * path, Assimp::Importer &importer, std::map<std::string, unsigned int> & FramesNamesIndex, Animation2*& ret, const aiScene*& scene)
{
	scene = importer.ReadFile(path, aiProcess_ConvertToLeftHanded);

	ret = new Animation2();

	ret->init(FramesNamesIndex.size(), 1.0, 1.0);
}

void __grabAnimationChannelsFromUnreal(std::map<std::string, unsigned int> & FramesNamesIndex, Animation2*& ret, const aiScene*& scene)
{
	JointSamples RootPositionSamples;
	JointQuaternionSamples RootRotationSamples;

	for (int i = 0; scene->mNumAnimations > 0 && i < scene->mAnimations[0]->mNumChannels; i++)
	{
		double mDuration = scene->mAnimations[0]->mDuration;

		auto c = scene->mAnimations[0]->mChannels[i];

		auto name = std::string(c->mNodeName.C_Str());

		auto f = FramesNamesIndex.find(name);
		if (f == FramesNamesIndex.end())
		{
			if (name == "root")
			{
				for (int j = 0; j < c->mNumRotationKeys; j++)
				{
					auto & r = c->mRotationKeys[j];

					RootRotationSamples.append(r.mTime / mDuration, r.mValue.x, r.mValue.y, r.mValue.z, r.mValue.w);
				}
				for (int j = 0; j < c->mNumPositionKeys; j++)
				{
					auto & p = c->mPositionKeys[j];

					RootPositionSamples.append(p.mTime / mDuration, p.mValue.x, p.mValue.y, p.mValue.z, 0.f);
				}
				continue;
			}

			throw "";
		}

		unsigned int index = f->second;

		for (int j = 0; j < c->mNumScalingKeys; j++)
		{
			auto & s = c->mScalingKeys[j];

			ret->Impl->appendScaling(index, s.mTime / mDuration, s.mValue.x, s.mValue.y, s.mValue.z);
		}

		for (int j = 0; j < c->mNumRotationKeys; j++)
		{
			auto & r = c->mRotationKeys[j];

			ret->Impl->appendRotation(index, r.mTime / mDuration, r.mValue.x, r.mValue.y, r.mValue.z, r.mValue.w);
		}

		for (int j = 0; j < c->mNumPositionKeys; j++)
		{
			auto & t = c->mPositionKeys[j];

			ret->Impl->appendTranslation(index, t.mTime / mDuration, t.mValue.x, t.mValue.y, t.mValue.z);
		}
	}

	auto& HipRotationSamples = ret->Impl->JointsRotationSamples[FramesNamesIndex.find("Hips")->second].X;

	auto& HipTranslationSamples = ret->Impl->JointsTranslationSamples[FramesNamesIndex.find("Hips")->second].X;

	for (int i = 0; i < HipTranslationSamples.size(); i++)
	{
		DirectX::XMFLOAT4 RootRotationSample;
		RootRotationSamples.getSample(HipTranslationSamples[i].time, RootRotationSample);
		DirectX::XMFLOAT4 RootPositionSample;
		RootPositionSamples.getSample(HipTranslationSamples[i].time, RootPositionSample);

		SimpleMath::Quaternion Q2(RootRotationSample);

		HipTranslationSamples[i].payload = SimpleMath::Vector4(RootPositionSample) + SimpleMath::Vector4::Transform(HipTranslationSamples[i].payload, Q2);

		/////
		SimpleMath::Quaternion Q1 = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationX(90.0*PI / 180.0));
		HipTranslationSamples[i].payload = SimpleMath::Vector4::Transform(HipTranslationSamples[i].payload, Q1);
	}

	for (int i = 0; i < HipRotationSamples.size(); i++)
	{
		DirectX::XMFLOAT4 RootRotationSample;
		RootRotationSamples.getSample(HipRotationSamples[i].time, RootRotationSample);
		
		SimpleMath::Quaternion Q2(RootRotationSample);
		SimpleMath::Quaternion Q3(HipRotationSamples[i].payload);

		HipRotationSamples[i].payload = SimpleMath::Quaternion::Concatenate(Q2, Q3);

		/////
		SimpleMath::Quaternion Q1 = SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationX(90.0*PI / 180.0));
		HipRotationSamples[i].payload = SimpleMath::Quaternion::Concatenate(Q1, HipRotationSamples[i].payload);
	}

	ret->Impl->JointsRotationSamples[FramesNamesIndex.find("Hips")->second].Y = HipRotationSamples[0].payload;

	ret->Impl->JointsTranslationSamples[FramesNamesIndex.find("Hips")->second].Y = HipTranslationSamples[0].payload;
}

void __grabAnimationChannels(std::map<std::string, unsigned int> & FramesNamesIndex, Animation2*& ret, const aiScene*& scene, char * replace = nullptr)
{
	for (int i = 0; scene->mNumAnimations > 0 && i < scene->mAnimations[0]->mNumChannels; i++)
	{
		double mDuration = scene->mAnimations[0]->mDuration;

		auto c = scene->mAnimations[0]->mChannels[i];

		auto key = std::string(c->mNodeName.C_Str());

		if (replace)
		{
			_replace(key, replace, std::string(""));
		}

		auto f = FramesNamesIndex.find(key);
		if (f == FramesNamesIndex.end())
		{
			throw "";
		}
		unsigned int index = f->second;

		for (int j = 0; j < c->mNumScalingKeys; j++)
		{
			auto & s = c->mScalingKeys[j];

			ret->Impl->appendScaling(index, s.mTime / mDuration, s.mValue.x, s.mValue.y, s.mValue.z);
		}

		for (int j = 0; j < c->mNumRotationKeys; j++)
		{
			auto & r = c->mRotationKeys[j];

			ret->Impl->appendRotation(index, r.mTime / mDuration, r.mValue.x, r.mValue.y, r.mValue.z, r.mValue.w);
		}

		for (int j = 0; j < c->mNumPositionKeys; j++)
		{
			auto & t = c->mPositionKeys[j];

			ret->Impl->appendTranslation(index, t.mTime / mDuration, t.mValue.x, t.mValue.y, t.mValue.z);
		}
	}
}

Animation* loadAnimationFromBlender(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex)
{
	Animation2* ret;
	const aiScene* scene;
	Assimp::Importer importer;

	__beginAnimationLoad(path, importer, FramesNamesIndex, ret, scene);

	{
		auto pFrameIndex = FramesNamesIndex.find("RootNode");
		if (pFrameIndex == FramesNamesIndex.end())
		{
			throw "";
		}

		auto index = pFrameIndex->second;

		auto RootTransform = aiMatrixToSimpleMathMatrix(scene->mRootNode->mChildren[0]->mTransformation) * aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation);

		ret->Impl->setJoint(index, RootTransform);
		ret->Impl->getJointSQT(index, ret->CurrentJoints[index]);

		collectBones(scene->mRootNode->mChildren[0], ret->Impl, &(ret->CurrentJoints), &FramesNamesIndex, "Armature_");
	}

	__grabAnimationChannels(FramesNamesIndex, ret, scene);

	return ret;
}

Animation* loadAnimationFromUnreal(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex)
{
	Animation2* ret;
	const aiScene* scene;
	Assimp::Importer importer;

	__beginAnimationLoad(path, importer, FramesNamesIndex, ret, scene);

	{
		auto pFrameIndex = FramesNamesIndex.find("RootNode");
		if (pFrameIndex == FramesNamesIndex.end())
		{
			throw "";
		}

		auto index = pFrameIndex->second;

		auto RootTransform = SimpleMath::Matrix::CreateScale(0.01f);//aiMatrixToSimpleMathMatrix(scene->mRootNode->mChildren[0]->mTransformation) * aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation);

		ret->Impl->setJoint(index, RootTransform);
		ret->Impl->getJointSQT(index, ret->CurrentJoints[index]);

		collectBones(scene->mRootNode->mChildren[0], ret->Impl, &(ret->CurrentJoints), &FramesNamesIndex, nullptr);
	}

	__grabAnimationChannelsFromUnreal(FramesNamesIndex, ret, scene);

	return ret;
}

Animation* loadAnimation(const char * path, std::map<std::string, unsigned int> & FramesNamesIndex, char * replace = nullptr)
{
	Animation2* ret;
	const aiScene* scene;
	Assimp::Importer importer;

	__beginAnimationLoad(path, importer, FramesNamesIndex, ret, scene);

	{
		auto pFrameIndex = FramesNamesIndex.find(scene->mRootNode->mName.C_Str());
		if (pFrameIndex == FramesNamesIndex.end())
		{
			throw "";
		}

		auto index = pFrameIndex->second;

		ret->Impl->setJoint(index, aiMatrixToSimpleMathMatrix(scene->mRootNode->mTransformation));
		ret->Impl->getJointSQT(index, ret->CurrentJoints[index]);

		collectBones(scene->mRootNode, ret->Impl, &(ret->CurrentJoints), &FramesNamesIndex, replace);
	}

	__grabAnimationChannels(FramesNamesIndex, ret, scene, replace);

	return ret;
}

void extractAnimationMeta(Animation * anim, bool extractHeight, double duration, std::function<SimpleMath::Matrix * __cdecl(unsigned int index)> getSkeletMatrix, std::function<void __cdecl()> calculateFramesTransformations)
{
	auto ret = (Animation2*)anim;

	ret->Impl->global_duration *= duration;
	ret->Impl->local_duration *= duration;

	//if (std::string(path) == "Media\\Animations\\HangingIdle.dae")
	//	ret->Impl->SetMeta0ToZero();
	//else

	//ret->Impl->met
	ret->Impl->extractRootMotionToMeta0(64, 7, 2, 0, extractHeight, getSkeletMatrix, calculateFramesTransformations);
	ret->Impl->extractHandsToMeta1AndMeta2(64, 54, 31, getSkeletMatrix, calculateFramesTransformations);//findTransformationFrameByName(skelet->frame, "LeftHand"), findTransformationFrameByName(skelet->frame, "RightHand")
}

void AnimationSetJointT(Animation * anim, int JointNum, SimpleMath::Vector3 Translation)
{
	JointSQT joint;
	auto ret = (Animation2*)anim;
	ret->Impl->getJointSQT(JointNum, joint);
	joint[2] = SimpleMath::Vector4(Translation.x, Translation.y, Translation.z, 0.f);
	ret->Impl->setJoint(JointNum, joint);
}

float AnimationComputeLocalTime(Animation * anim, float time)
{
	auto ret = (Animation2*)anim;

	float local_time = ret->Impl->Rate * time;

	if (ret->Impl->Rate < .0)
	{
		local_time = ret->Impl->local_duration + local_time;
	}

	return local_time;
}

SimpleMath::Vector3 sampleAnimationMeta(Animation * anim, unsigned int chanel, float time)
{
	SimpleMath::Vector3 result;

	auto ret = (Animation2*)anim;

	ret->Impl->resetMetaSampleIndex(chanel, ret->Impl->Rate);

	ret->Impl->getMetaSample(chanel, time, result);

	ret->Impl->resetMetaSampleIndex(chanel, ret->Impl->Rate);

	return result;
}

void rotateHips(Animation * anim, SimpleMath::Quaternion Rotation)
{
	SimpleMath::Vector3 result;

	auto ret = (Animation2*)anim;

	//множества поз из тайм линии
	{
		auto& HipTranslationSamples = ret->Impl->JointsTranslationSamples[64].X;

		for (int i = 0; i < HipTranslationSamples.size(); i++)
		{
			HipTranslationSamples[i].payload = SimpleMath::Vector4::Transform(HipTranslationSamples[i].payload, Rotation);
		}

		auto& HipRotationSamples = ret->Impl->JointsRotationSamples[64].X;

		for (int i = 0; i < HipRotationSamples.size(); i++)
		{
			SimpleMath::Quaternion Q1(Rotation);

			SimpleMath::Quaternion Q2(HipRotationSamples[i].payload);

			HipRotationSamples[i].payload = SimpleMath::Quaternion::Concatenate(Q1, Q2);
		}
	}

	//когда просто одна поза
	{
		auto& HipTranslation = ret->Impl->JointsTranslationSamples[64].Y;

		HipTranslation = SimpleMath::Vector4::Transform(HipTranslation, Rotation);

		auto& HipRotation = ret->Impl->JointsRotationSamples[64].Y;

		SimpleMath::Quaternion Q1(Rotation);

		SimpleMath::Quaternion Q2(HipRotation);

		HipRotation = SimpleMath::Quaternion::Concatenate(Q1, Q2);
	}
}

SimpleMath::Vector3 getModelSpaceDeltaTranslationFromAnimation2(AnimationBase* Anim)
{
	return 	((Animation2*)Anim)->ModelSpaceDeltaTranslation;
}
