#include <locale>
#include <codecvt>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <memory>

#include "GeometricPrimitive.h"
#include "DirectXHelpers.h"
#include "SimpleMath.h"
#include "VertexTypes.h"
#include "DirectXMath.h"
#include "Model.h"

using namespace DirectX;

void disposeFrames(struct TransformationFrame * Frame);

void characterfillFramesTransformsRefs(struct Character * character, struct TransformationFrame * frame);

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
	void Rotate(const SimpleMath::Quaternion& Quat, const DirectX::XMFLOAT4& Origin)
	{
		Rotate(Quat, SimpleMath::Vector3(SimpleMath::Vector4(Origin)));
	}
	void Rotate(const SimpleMath::Quaternion& Quat, const SimpleMath::Vector3& Origin)
	{
		X[1] = SimpleMath::Quaternion::Concatenate(Quat, X[1]);
		const auto ThisJointLocation = SimpleMath::Vector3(SimpleMath::Vector4(X[2]));
		const auto result = SimpleMath::Vector3::Transform(ThisJointLocation - Origin, Quat) + Origin;
		X[2].x = result.x;
		X[2].y = result.y;
		X[2].z = result.z;
	}
	void Rotate(const SimpleMath::Quaternion& Quat)
	{
		X[1] = SimpleMath::Quaternion::Concatenate(Quat, X[1]);
	}
};

struct VertexPositionNormalTangentColorTextureSkinning2 : public VertexPositionNormalTangentColorTexture
{
	uint32_t indices;
	DirectX::XMFLOAT4 weights;

	uint32_t indices2;
	DirectX::XMFLOAT4 weights2;

	VertexPositionNormalTangentColorTextureSkinning2()
	{
		SetBlendIndices(XMUINT4(0, 0, 0, 0));
		SetBlendWeights(FXMVECTOR());

		SetBlendIndices2(XMUINT4(0, 0, 0, 0));
		SetBlendWeights2(FXMVECTOR());
	}

	void SetBlendIndices(XMUINT4 const& iindices)
	{
		this->indices = ((iindices.w & 0xff) << 24) | ((iindices.z & 0xff) << 16) | ((iindices.y & 0xff) << 8) | (iindices.x & 0xff);
	}

	void SetBlendIndices2(XMUINT4 const& iindices)
	{
		this->indices2 = ((iindices.w & 0xff) << 24) | ((iindices.z & 0xff) << 16) | ((iindices.y & 0xff) << 8) | (iindices.x & 0xff);
	}

	void SetBlendWeights(FXMVECTOR iweights)
	{
		this->weights = SimpleMath::Vector4(iweights);
		//DirectX::PackedVector::XMUBYTEN4 packed;
		//DirectX::PackedVector::XMStoreUByteN4(&packed, iweights);
		//this->weights2 = packed.v;
	}

	void SetBlendWeights2(FXMVECTOR iweights)
	{
		this->weights2 = SimpleMath::Vector4(iweights);
		//DirectX::PackedVector::XMUBYTEN4 packed;
		//DirectX::PackedVector::XMStoreUByteN4(&packed, iweights);
		//this->weights2 = packed.v;
	}
};

struct TransformationFrame
{
	TransformationFrame() :NextSibling(), FirstChild(), Parent(){}
	std::string Name;
	SimpleMath::Matrix Transformation;
	TransformationFrame * NextSibling;
	TransformationFrame * FirstChild;
	TransformationFrame * Parent;

	std::vector< DirectX::ModelMeshPart* > Meshes;
	std::vector< std::vector<VertexPositionNormalTangentColorTextureSkinning2> > MeshesVertex;
	std::vector< std::vector<UINT> > MeshesIndices;

	std::map<std::string, SimpleMath::Matrix> socketsTransforms;
};

struct Skeleton
{
	std::map<std::string, unsigned int> JointsIndex;
	std::map<std::string, unsigned int> SocketsIndex;
};

struct Character
{
	Character(Skeleton* newSkeleton) :TotalJoints(), ReservJoints(), TotalTransform(){ skeleton = newSkeleton; };

	Character::~Character(){
		disposeFrames(frame);
	};

	Skeleton* skeleton;

	std::vector<SimpleMath::Matrix> SocketsTransforms;
	std::vector<SimpleMath::Matrix> BoneOffSetTransformation;

	TransformationFrame * frame;

	std::vector<SimpleMath::Matrix*> framesTransformsRefs;
	std::vector<SimpleMath::Matrix*> socketsTransformsRefs;

	unsigned int TotalJoints;
	unsigned int ReservJoints;
	unsigned int TotalTransform;

	void fillFramesTransformsRefs()
	{
		characterfillFramesTransformsRefs(this, frame);
	}

	unsigned int findFrame(const std::string& name)
	{
		auto& JointsIndex = skeleton->JointsIndex;

		unsigned int LastJointIndex;
		auto Iter = JointsIndex.find(name);
		if (JointsIndex.end() != Iter)
		{
			LastJointIndex = (*Iter).second;
			if (TotalJoints < (LastJointIndex + 1))
			{
				TotalJoints = LastJointIndex + 1;
			}
		}
		else
		{
			LastJointIndex = TotalJoints;
			JointsIndex.insert(std::pair<std::string, unsigned int>(name, TotalJoints));
			TotalJoints++;
		}

		if (ReservJoints < (LastJointIndex + 1))
		{
			ReservJoints = LastJointIndex + 65;
			framesTransformsRefs.reserve(ReservJoints);
			BoneOffSetTransformation.reserve(ReservJoints);
		}

		if (TotalTransform < (LastJointIndex + 1))
		{
			TotalTransform = LastJointIndex + 1;
			framesTransformsRefs.resize(LastJointIndex + 1);
			BoneOffSetTransformation.resize(LastJointIndex + 1);
		}

		return LastJointIndex;
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
