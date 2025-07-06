#include <locale>
#include <codecvt>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "DirectXHelpers.h"
#include "SimpleMath.h"

using namespace DirectX;

struct JointSQT{
	DirectX::XMFLOAT4 X[3];
	DirectX::XMFLOAT4& operator[](unsigned int index){
		return X[index];
	}
	SimpleMath::Matrix matrix()
	{
		auto _S = SimpleMath::Matrix::CreateScale(X[0].x, X[0].y, X[0].z);
		auto _Q = SimpleMath::Matrix::CreateFromQuaternion(SimpleMath::Quaternion(X[1].x, X[1].y, X[1].z, X[1].w));
		auto _T = SimpleMath::Matrix::CreateTranslation(X[2].x, X[2].y, X[2].z);
		return _S * _Q * _T;
	}
};

struct AnimationBase
{
	virtual void sample(double elapsedTime){};

	virtual void advanse(double elapsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation) = 0;

	virtual bool getPlaying() = 0;

	virtual void setPlaying(bool value) = 0;

	virtual bool isPlaying() = 0;

	virtual void reset() = 0;

	SimpleMath::Quaternion RootDeltaRotation;

	SimpleMath::Quaternion RootSampledRotation;

	std::vector<JointSQT> CurrentJoints;

	std::vector<SimpleMath::Vector3> CurrentMetaChannels;

	virtual ~AnimationBase();

	virtual std::vector<JointSQT>& __AnimGetJointsByTime(float Time){ return CurrentJoints; };

	virtual void SetBlendTime(double& blendTime, bool& blendRoot){ };

	virtual void PostActions(){ };

	virtual void setCurrentAnimationTime(float global_time){ };
};

struct Animation : public AnimationBase
{
	virtual void subscribe(char * eventName, std::function<void __cdecl(bool state)>) = 0;

	virtual double getRate() = 0;

	virtual void setRate(double speed) = 0;

	virtual int getFrame() = 0;

	virtual double getLocTime() = 0;

	virtual void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f) = 0;

	virtual void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f) = 0;

	virtual void setLooping(bool loop) = 0;

	virtual bool IsLoop() = 0;

	virtual ~Animation();
};

struct AnimationLinearBlend : public AnimationBase
{
	virtual SimpleMath::Vector3 getCurrentMeta(int channelId) = 0;
	virtual SimpleMath::Vector3 getCurrentMetaOffset(int channelId) = 0;
	virtual SimpleMath::Vector3 getCurrentJointTranslationOffset(int jointId) = 0;
	virtual float getTimeRatio() = 0;
	virtual ~AnimationLinearBlend();
};

struct AnimationWithImpl : public Animation
{
	class AnimationRep* Impl;

	std::function<void __cdecl(bool state)> onPlayingChanged;

	AnimationWithImpl();

	~AnimationWithImpl();

	void TransformMetaSamples(int channelId, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f);

	void TransformJointSamples(int jointId, char* sampleKind, std::function<SimpleMath::Vector4 __cdecl(SimpleMath::Vector4)> f);

	void setRate(double speed);

	void setLooping(bool loop);

	double getRate();

	int getFrame();
	
	double getLocTime(); 

	bool IsLoop();

	void subscribe(char * eventName, std::function<void __cdecl(bool state)> callback);
	
	bool getPlaying();

	void setPlaying(bool value);

	bool isPlaying();

	void reset();
};
