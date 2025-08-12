#include "GeometricPrimitive.h"
#include "Effects.h"
#include "DirectXHelpers.h"
#include "Model.h"
#include "CommonStates.h"
#include "Keyboard.h"
#include "Mouse.h"
#include "SimpleMath.h"

#include "DirectXMath.h"

#include "DXUT.h"

#include <wrl.h>

#include <map>
#include <algorithm>
#include <array>
#include <memory>
#include <assert.h>
#include <malloc.h>
#include <Exception>
#include <string>

#include "ConstantBuffer.h"

#include "AppBuffers.h"

#include "AnimationClasses.h"

using namespace DirectX;
extern char DebugBuffer[1024];
void Debug();
struct EffectShaderFileDef{
	WCHAR * name;
	WCHAR * entry_point;
	WCHAR * shader_ver;
};
class IPostProcess
{
public:
	virtual ~IPostProcess() { }

	virtual void __cdecl Process(_In_ ID3D11DeviceContext* deviceContext, _In_opt_ std::function<void __cdecl()> setCustomState = nullptr) = 0;
};

inline ID3D11RenderTargetView** renderTargetViewToArray(ID3D11RenderTargetView* rtv1, ID3D11RenderTargetView* rtv2 = 0, ID3D11RenderTargetView* rtv3 = 0){
	static ID3D11RenderTargetView* rtvs[10];
	rtvs[0] = rtv1;
	rtvs[1] = rtv2;
	rtvs[2] = rtv3;
	return rtvs;
};
inline ID3D11ShaderResourceView** shaderResourceViewToArray(ID3D11ShaderResourceView* rtv1, ID3D11ShaderResourceView* rtv2 = 0, ID3D11ShaderResourceView* rtv3 = 0, ID3D11ShaderResourceView* rtv4 = 0, ID3D11ShaderResourceView* rtv5 = 0){
	static ID3D11ShaderResourceView* srvs[10];
	srvs[0] = rtv1;
	srvs[1] = rtv2;
	srvs[2] = rtv3;
	srvs[3] = rtv4;
	srvs[4] = rtv5;
	return srvs;
};
inline ID3D11Buffer** constantBuffersToArray(ID3D11Buffer* c1, ID3D11Buffer* c2){
	static ID3D11Buffer* cbs[10];
	cbs[0] = c1;
	cbs[1] = c2;
	return cbs;
};
inline ID3D11Buffer** constantBuffersToArray(DirectX::ConstantBuffer<SceneState> &cb){
	static ID3D11Buffer* cbs[10];
	cbs[0] = cb.GetBuffer();
	return cbs;
};
inline ID3D11Buffer** constantBuffersToArray(DirectX::ConstantBuffer<PostProccessState> &cb){
	static ID3D11Buffer* cbs[10];
	cbs[0] = cb.GetBuffer();
	return cbs;
};
inline ID3D11Buffer** constantBuffersToArray(DirectX::ConstantBuffer<BoneToModelPalite> &cb){
	static ID3D11Buffer* cbs[10];
	cbs[0] = cb.GetBuffer();
	return cbs;
};
inline ID3D11Buffer** constantBuffersToArray(DirectX::ConstantBuffer<EvePath> &cb){
	static ID3D11Buffer* cbs[10];
	cbs[0] = cb.GetBuffer();
	return cbs;
};
inline ID3D11SamplerState** samplerStateToArray(ID3D11SamplerState* ss1, ID3D11SamplerState* ss2 = 0){
	static ID3D11SamplerState* sss[10];
	sss[0] = ss1;
	sss[1] = ss2;
	return sss;
};

namespace Camera{
	void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext);
}
std::unique_ptr<DirectX::IEffect> createHlslEffect(ID3D11Device* device, std::map<const WCHAR*, EffectShaderFileDef>& fileDef);

class GraphicResources {
public:
	std::unique_ptr<CommonStates> render_states;

	std::unique_ptr<DirectX::IEffect> water_effect;

	std::unique_ptr<DirectX::IEffect> eve_effect;

	std::unique_ptr<DirectX::IEffect> sky_effect;

	std::unique_ptr<DirectX::IEffect> winstons_barrier_effect;

	std::unique_ptr<DirectX::IEffect> std_lit_effect;

	std::unique_ptr<DirectX::IEffect> box_effect;

	std::unique_ptr<DirectX::IEffect> ledge_box_effect;

	std::unique_ptr<DirectX::IEffect> box_glow_effect;

	std::unique_ptr<DirectX::IEffect> blur_horizontal_effect;

	std::unique_ptr<DirectX::IEffect> blur_vertical_effect;

	std::unique_ptr<DirectX::IEffect> post_proccess_effect;

	std::unique_ptr<DirectX::IEffect> ground_effect;

	std::unique_ptr<DirectX::IEffect> eve_path_effect;

	std::unique_ptr<DirectX::ConstantBuffer<SceneState> > scene_constant_buffer;

	std::unique_ptr<DirectX::ConstantBuffer<EvePath> > eve_path_constant_buffer;

	std::unique_ptr<DirectX::ConstantBuffer<PostProccessState> > post_proccess_constant_buffer;

	std::unique_ptr<ConstantBuffer<BoneToModelPalite> > bone_to_model_constant_buffer;

	std::unique_ptr<GeometricPrimitive> sphere_model;

	std::unique_ptr<GeometricPrimitive> cylinder_model;

	std::unique_ptr<DirectX::ModelMeshPart> water_model;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> sky_cube_texture;

	Microsoft::WRL::ComPtr<ID3D11InputLayout> sphere_input_layout;

	Microsoft::WRL::ComPtr<ID3D11InputLayout> eve_input_layout;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> ground_texture;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> ground_normal_texture;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> hexagon_texture;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> eve_d_texture;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> eve_n_texture;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> water_texture;

	std::unique_ptr<DirectX::IEffect> copy_effect;

	Microsoft::WRL::ComPtr<ID3D11BlendState> AdditiveBlend;

	Microsoft::WRL::ComPtr<ID3D11InputLayout> dxtk_primitive_to_unlit_white_input_layout;

	std::unique_ptr<DirectX::IEffect> unlit_white_effect;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> default_Material_Grid_M;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> default_Material_Grid_N;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> Holster_Albedo_M;

	std::unique_ptr<DirectX::IEffect> lambert_effect;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> Female_Western_Gun_M;
};

class SwapChainGraphicResources {
public:
	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_glowObjects;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_glowObjects;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_glowObjects;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_scene;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_scene;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_scene;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_glowBlurObjects;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_glowBlurObjects;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_glowBlurObjects;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_glowBlurObjects1;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_glowBlurObjects1;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_glowBlurObjects1;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_depth;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_depth;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> DS_depth;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_depth1;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_depth1;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> DS_depth1;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_reflection;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_reflection;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_reflection;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> T_refraction;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SR_refraction;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> RT_refraction;
};

struct AnimationGraph;

SimpleMath::Matrix* GetSkeletonMatrix(Character *character, int index);

inline void fillSkeletonTransformFromJoint(AnimationBase* animation, Character *character)
{
	for (int i = 0; animation && i < animation->CurrentJoints.size(); i++)
	{
		(*GetSkeletonMatrix(character, i)) = animation->CurrentJoints[i].matrix();
	}
}

struct StaticObject
{
	TransformationFrame * frame;

	void getAllTriangles(std::vector<SimpleMath::Vector3> &vertex, const SimpleMath::Matrix & M);

	std::vector< std::string > EmitLedges(const std::string& SpaceName, SimpleMath::Vector3& BoundBoxMin, SimpleMath::Vector3& BoundBoxMax);

	~StaticObject();
};

enum AnimationState
{
	idle,
	walking,
	idle_to_walking,
	walking_to_idle,
	jump,
	walking_to_jump,
	idle_to_jump,
	jump_to_walking,
	jump_to_idle,
	jumpUp,
	jumpUp_to_idle,
	idle_to_jumpUp,
	climbing,
};

struct IAnimationGraph
{
	virtual void advanse(double newElipsedTime, SimpleMath::Vector3 & deltaTranslation) = 0;
	virtual void setState(AnimationState newState) = 0;
	virtual AnimationState getState() = 0;
	virtual Animation* gatAnimationByState(AnimationState state) = 0;
	virtual void setPlaying(bool value) = 0;
	virtual bool IsPlaying() = 0;

	virtual ~IAnimationGraph();
};

struct AnimationGraph2Link;
struct AnimationGraph2Links;

struct IAnimationGraph2
{
	virtual void registerAnimation(char * name, AnimationBase * animation) = 0;
	virtual AnimationGraph2Links createLinks(int Count, AnimationBase ** fromAnimations) = 0;
	virtual AnimationGraph2Link createLink(AnimationBase * fromAnimation) = 0;
	virtual void registerLink(AnimationBase * fromAnimation, AnimationBase * toAnimation, std::function<bool __cdecl()> cond) = 0;
	virtual void start(AnimationBase * anim) = 0;
	virtual void advanse(double newElipsedTime, SimpleMath::Vector3& DeltaTranslation, SimpleMath::Quaternion& DeltaRotation) = 0;
	virtual char * getAnimationName() = 0;
	virtual char * getPrevAnimationName() = 0;
	template<class T> T* getAnimation(){
		return dynamic_cast<T*>(_getAnimation());
	};
	virtual AnimationBase * _getAnimation() = 0;
	virtual AnimationBase * getAnimation(char* name) = 0;
	virtual AnimationLinearBlend* getAnimationBlend() = 0;

	virtual ~IAnimationGraph2();

	virtual AnimationBase* getPlayingAnimation() = 0;

	virtual bool IsBlendActivated() = 0;

	virtual char* GetCurrentConditionNodeName() = 0;

	virtual AnimationBase* GetCurrentConditionNodeAnimation() = 0;
};

struct AnimationGraph2Link
{
	IAnimationGraph2 * graph;
	AnimationBase * _fromAnimation;
	AnimationBase * _toAnimation;
	AnimationGraph2Link& setEndPoint(AnimationBase * toAnimation){
		_toAnimation = toAnimation;
		return *this;
	};
	AnimationGraph2Link& forward(std::function<bool __cdecl()> cond){
		graph->registerLink(_fromAnimation, _toAnimation, cond);
		return *this;
	};
	AnimationGraph2Link& reverse(std::function<bool __cdecl()> cond){
		graph->registerLink(_toAnimation, _fromAnimation, cond);
		return *this;
	};
};

struct AnimationGraph2Links
{
	IAnimationGraph2 * graph;
	int Count;
	AnimationBase * _fromAnimation[10];
	AnimationBase * _toAnimation[10];
	AnimationGraph2Links& setEndPoint(AnimationBase ** toAnimation){
		for (int i = 0; i < Count; i++)
		{
			_toAnimation[i] = toAnimation[i];
		}
		return *this;
	};
	AnimationGraph2Links& forward(std::function<bool __cdecl()> cond){
		for (int i = 0; i < Count; i++)
		{
			graph->registerLink(_fromAnimation[i], _toAnimation[i], cond);
		}
		return *this;
	};
	AnimationGraph2Links& reverse(std::function<bool __cdecl()> cond){
		for (int i = 0; i < Count; i++)
		{ 
			graph->registerLink(_toAnimation[i], _fromAnimation[i], cond);
		}
		return *this;
	};
};

Character* loadCharacter(ID3D11Device* device, char * file_name, Skeleton& HumanSkeleton);
StaticObject* loadStaticObject(ID3D11Device* device, char * file_name);
StaticObject* loadStaticObject2(ID3D11Device* device, char * file_name);
void loadAnimations(IAnimationGraph2 *, Skeleton* skeleton);

AnimationLinearBlend* makeBlend();
AnimationBase* makeAnimationPose();
void copyAnimationPose(AnimationBase* src, AnimationBase* dst);
IAnimationGraph2 * makeAnimationGraph(AnimationLinearBlend * transitionAnim, AnimationBase * animationPose);
//IAnimationGraph * __makeAnimationGraph(std::map<AnimationState, Animation *> newAnimations, CharacterSkelet * skelet);
void disposeAnimationGraph(IAnimationGraph2* stateMachine);

void calculateFramesTransformations(TransformationFrame * frame, SimpleMath::Matrix ParentTransformation);
void calculateFramesTransformations(Character * CharacterObject, SimpleMath::Matrix ParentTransformation);
SimpleMath::Matrix* calculateAnimationPalite(Character * character);
void drawFrames(TransformationFrame * Frame, ID3D11DeviceContext* context, IEffect* effect, ID3D11InputLayout* inputLayout, std::function<void __cdecl()> setCustomState, std::function<void __cdecl(TransformationFrame * Frame)> onFrameWithMesh = [](TransformationFrame *){});
void getJointFrame(Character * skelet, char * JointName, std::function<void __cdecl(SimpleMath::Matrix M)> OnFrame);
SimpleMath::Vector4 vec4(SimpleMath::Vector3 v3, float w);
void extractAnimationMeta(Animation * anim, bool extractHeight, double duration);

struct JointHelpers
{
	static void AnimToChain(const std::vector<int> & chain, AnimationBase * Anim, std::vector<JointSQT> & Joints);
	static void ChainToAnim(const std::vector<int> & chain, std::vector<JointSQT> & Joints, AnimationBase * Anim);
	static void ChainToAnim(int StartIndex, int FinishIndex, const std::vector<int> & chain, std::vector<JointSQT> & Joints, AnimationBase * Anim);
	static void localToModel(int Count, std::vector<JointSQT> & Joints);
	static void modelToLocal(int Count, std::vector<JointSQT> & Joints);
	static void localToModel(int StartIndex, int FinishIndex, std::vector<JointSQT> & Joints);
	static void modelToLocal(int StartIndex, int FinishIndex, std::vector<JointSQT> & Joints);
	static void fixJointRotation(SimpleMath::Vector3 oldDir, SimpleMath::Vector3 newDir, JointSQT & Joint);
	static SimpleMath::Vector3 transformFromFirstToLastJointFrame(const std::vector<int> & indexChain, AnimationBase * anim, const SimpleMath::Vector3 & position);
};

void GetChain(Character * character, char * JointName0, char * JointName1, std::vector<int>& indexes);
void GetChain(Character * character, char * JointName0, std::vector<int>& indexes);

struct JointsRefsChainCollection
{
	std::vector<int> RightShoulderRightHand;
	std::vector<int> HipsSpine2;

	std::vector<int> RightArmRightHand;
	std::vector<int> HipsRightShoulder;

	std::vector<int> HipsRightArm;

	std::vector<int> HipsRightHand;

	std::vector<int> LeftArmLeftHand;
	std::vector<int> HipsLeftShoulder;


	std::vector<int> HipsLeftHand;
	std::vector<int> HipsHeadTop_End;
	std::vector<int> HipsLeftToe_End;
	std::vector<int> HipsRightToe_End;

	std::vector<int> Hand_Childs[2][5];

	int size()
	{
		return 5;
	}

	std::vector<int>& operator[] (int Index)
	{
		static std::vector<int>* Chains[] = 
		{
			&HipsRightHand,
			&HipsLeftHand,

			&HipsHeadTop_End,

			&HipsRightToe_End,
			&HipsLeftToe_End
		};

		return *(Chains[Index]);
	}

	void init(Character * character)
	{
		GetChain(character, "RightShoulder", "RightHand", RightShoulderRightHand);
		GetChain(character, "Hips", "Spine2", HipsSpine2);

		GetChain(character, "RightArm", "RightHand", RightArmRightHand);
		GetChain(character, "Hips", "RightShoulder", HipsRightShoulder);

		GetChain(character, "Hips", "RightArm", HipsRightArm);

		GetChain(character, "Hips", "RightHand", HipsRightHand);

		GetChain(character, "LeftArm", "LeftHand", LeftArmLeftHand);
		GetChain(character, "Hips", "LeftShoulder", HipsLeftShoulder);


		GetChain(character, "Hips", "LeftHand", HipsLeftHand);
		GetChain(character, "Hips", "HeadTop_End", HipsHeadTop_End);
		GetChain(character, "Hips", "LeftToe_End", HipsLeftToe_End);
		GetChain(character, "Hips", "RightToe_End", HipsRightToe_End);

		char* Domain = "Hand";
		char* DomainPrefics[] = { "Left" , "Right" };
		char* SubDomains[] = { "Thumb" , "Index" , "Middle" , "Ring" , "Pinky" };

		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 5; j++)
			{
				auto Start = std::string(DomainPrefics[i]) + "Hand" + SubDomains[j] + "1";
				GetChain(character, &Start[0], Hand_Childs[i][j]);
			}
	}
};

struct IKSolverInterface
{
	virtual std::vector<JointSQT>& chainRef(int chain) = 0;
	//virtual void setChainSize(int chain, int size) = 0;
	virtual void solve(int index, int ChainStart, int ChainFinish, SimpleMath::Vector3 target, int total_iter = 10, float epsilon = 0.001f) = 0;
	virtual ~IKSolverInterface();
};

IKSolverInterface* makeIKSolver(Character * character);

struct HitInfo{
	SimpleMath::Vector3 position;
	SimpleMath::Vector3 velocity;
	float nearestDistance;
	SimpleMath::Vector3 intersectionPoint;
	bool hit;
};

struct CollisionFrame{
	bool overlap;
	CollisionFrame(){
		overlap = false;
	}
	SimpleMath::Quaternion orientation;
	SimpleMath::Vector3 origin;
	SimpleMath::Matrix getMatrix() const{
		return
			SimpleMath::Matrix::CreateFromQuaternion(orientation) *
			SimpleMath::Matrix::CreateTranslation(origin);
	};
};

struct Capsule : CollisionFrame{
	float r;
	float ab;
	void getAB(SimpleMath::Vector3 & A, SimpleMath::Vector3 & B) const {
		A = origin + r * getMatrix().Up();
		B = A + ab * getMatrix().Up();
	};
	SimpleMath::Vector3 getProb() const
	{
		return SimpleMath::Vector3(r,0,0);
	};
	SimpleMath::Matrix getMiddleCapsuleRaySystem() const
	{
		return SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3(r, r + 0.5f*ab, 0)) * getMatrix();
	};
};

struct Box : CollisionFrame{
	Box(){}
	Box(SimpleMath::Vector3& BoundBoxMin, SimpleMath::Vector3& BoundBoxMax, SimpleMath::Quaternion& __orientation)
	{
		origin = 0.5f*(BoundBoxMin + BoundBoxMax);
		orientation = __orientation;
		size = BoundBoxMax - BoundBoxMin;
	}
	static bool rayIntersectWithBox(const SimpleMath::Vector3& _S, const SimpleMath::Vector3& _V, float& t, SimpleMath::Vector3& _P)
	{
		struct VectorToArray{
			float items[3];
			VectorToArray(const SimpleMath::Vector3& V){
				items[0] = V.x;
				items[1] = V.y;
				items[2] = V.z;
			}
			float operator [](int index){
				return items[index];
			}
		};
		VectorToArray v(_V);
		VectorToArray s(_S);
		for (int i = 0; i < 3; i++){
			if (abs(v[i]) > 0.0001f){
				float r = v[i] < .0f ? 1 : 0;
				t = (r - s[i]) / v[i];
				if (t > 0.0){
					_P = _S + t*_V;
					VectorToArray p(_P);
					int j = (i + 1) % 3;
					int k = (i + 2) % 3;
					if (0.0f <= p[j] && p[j] <= 1.0f){
						if (0.0f <= p[k] && p[k] <= 1.0f)
							return true;
					}
				}
			}
		}
		return false;
	}
	static bool rayIntersectWithBox2(const SimpleMath::Vector3& _S, const SimpleMath::Vector3& _V, float& t, SimpleMath::Vector3& _P)
	{
		struct VectorToArray{
			float items[3];
			VectorToArray(const SimpleMath::Vector3& V){
				items[0] = V.x;
				items[1] = V.y;
				items[2] = V.z;
			}
			float operator [](int index){
				return items[index];
			}
		};
		VectorToArray v(_V);
		VectorToArray s(_S);
		int Res = 0;
		bool intersect = false;
		for (int i = 0; i < 3; i++){
			if (abs(v[i]) > 0.0001f){
				for (int r = 0; r < 2; r++)
				{
					t = (r - s[i]) / v[i];
					_P = _S + t*_V;
					VectorToArray p(_P);
					int j = (i + 1) % 3;
					int k = (i + 2) % 3;
					if (0.0f <= p[j] && p[j] <= 1.0f){
						if (0.0f <= p[k] && p[k] <= 1.0f)
						{
							intersect = intersect || (0.0f <= t && t <= 1.0f);
							Res += (t < 0.0 ? -1 : 2);
						}
					}
				}
			}
		}
		return Res == 1 || intersect;
	}
	SimpleMath::Quaternion orientationInverse;
	SimpleMath::Vector3 worldForward;
	SimpleMath::Vector3 worldBackSide;
	SimpleMath::Vector3 worldSize;
	SimpleMath::Vector3 size;
	SimpleMath::Vector3 getHalfSize() const{
		return	0.5f * size;
	};
	bool Intersect(const SimpleMath::Vector3& S, const SimpleMath::Vector3& V, float* t_out = nullptr) const
	{
		const auto InvBoxMatrix = 
		SimpleMath::Matrix::CreateTranslation(-origin) *
		SimpleMath::Matrix::CreateFromQuaternion(orientationInverse) *
		SimpleMath::Matrix::CreateScale(SimpleMath::Vector3(1.f / size.x, 1.f / size.y, 1.f / size.z)) *
		SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3(.5f, .5f, .5f));

		const auto TrS = SimpleMath::Vector3::Transform(S, InvBoxMatrix);
		const auto TrV = SimpleMath::Vector3::TransformNormal(V, InvBoxMatrix);

		float t;
		SimpleMath::Vector3 TrP;

		if (rayIntersectWithBox(TrS, TrV, t, TrP))
		{
			if (t_out) *t_out = t;
			return t <= 1.f;
		}

		return false;
	}
	bool IntersectOrContain(const SimpleMath::Vector3& S, const SimpleMath::Vector3& V) const
	{
		const auto InvBoxMatrix =
			SimpleMath::Matrix::CreateTranslation(-origin) *
			SimpleMath::Matrix::CreateFromQuaternion(orientationInverse) *
			SimpleMath::Matrix::CreateScale(SimpleMath::Vector3(1.f / size.x, 1.f / size.y, 1.f / size.z)) *
			SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3(.5f, .5f, .5f));

		const auto TrS = SimpleMath::Vector3::Transform(S, InvBoxMatrix);
		const auto TrV = SimpleMath::Vector3::TransformNormal(V, InvBoxMatrix);

		float t;
		SimpleMath::Vector3 TrP;

		return rayIntersectWithBox2(TrS, TrV, t, TrP);
	}
	void evalWorldSize()
	{
		worldSize = SimpleMath::Vector3::TransformNormal(size, getMatrix());
		worldSize.x = fabs(worldSize.x);
		worldSize.y = fabs(worldSize.y);
		worldSize.z = fabs(worldSize.z);
		worldForward = SimpleMath::Vector3::TransformNormal(SimpleMath::Vector3(size.x,0.f,0.f), getMatrix());
		worldBackSide = SimpleMath::Vector3::TransformNormal(SimpleMath::Vector3(0.f, 0.f, size.z), getMatrix());

		orientation.Inverse(orientationInverse);
	}
	bool Containe(SimpleMath::Vector3 Point) const{
		auto RelPoint = Point - origin;
		auto HalfSize = 0.5f*worldSize;
		if (abs(RelPoint.x) <= HalfSize.x)
			if (abs(RelPoint.y) <= HalfSize.y)
				if (abs(RelPoint.z) <= HalfSize.z)
					return true;
		return false;
	};
	SimpleMath::Quaternion GetPawnOrientation()
	{
		auto X = -worldBackSide;
		X.Normalize();
		auto Z = worldForward;
		Z.Normalize();
		return SimpleMath::Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix(X, SimpleMath::Vector3(0, 1, 0), Z));
	}
};

struct Mesh : CollisionFrame{
	std::vector<SimpleMath::Vector3> vertex;
	const std::vector<SimpleMath::Vector3> & getVertex() const
	{
		return vertex;
	};
};

bool IsJointOrNestedJointsIntoBox(Character * skelet, const SimpleMath::Matrix& FromModelSpaceToWorld, char * JointName, int deep, const Box& box);

struct Ellipsoid : CollisionFrame{
	SimpleMath::Vector3 size;
};

struct Box2 : Box {
	SimpleMath::Matrix worldTransform;
	Box2(){}
	Box2(const Box& TheBox) :Box(TheBox){}
	void eval()
	{
		evalWorldSize();
		worldTransform = SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) * SimpleMath::Matrix::CreateScale(size) * getMatrix();
	}
};

struct Ledge {
	std::vector<Box> Owners;
	std::vector<Box2> Boxes;
	static Ledge Make(const Box& TheBox)
	{
		Ledge Ret;
		Box2 box(TheBox);
		box.eval();
		Ret.Boxes.push_back(box);
		return Ret;
	}
};

extern char* LeftHandName;
extern char* RightHandName;
extern char* Hands[2];

struct World{
	std::map<std::string, DirectX::XMFLOAT4X4> WorldTransforms;
	std::map<std::string, Box> Boxes;
	std::map<std::string, Ellipsoid> Ellipsoids;
	std::map<std::string, Capsule> Capsules;
	std::map<std::string, Mesh> Meshes;
	std::map<std::string, Ledge> Ledges;
	void Collision(const SimpleMath::Vector3&, const SimpleMath::Vector3&, Capsule&, HitInfo&);
};

struct HitInfo2{
	int objectType;
	std::string objectName;
	SimpleMath::Vector3 hitPoint;
	SimpleMath::Vector3 hitNormal;
	float t;
	static void Push(std::vector<HitInfo2>* HitInfos, const HitInfo2& HitInfo);
};

SimpleMath::Vector3 GetHandLocation(const SimpleMath::Matrix& FromModelSpaceToWorld, char* HandName);
SimpleMath::Vector3 GetHeadLocation(const SimpleMath::Matrix& FromModelSpaceToWorld);
void UpdateEveCapsuleHeight(const SimpleMath::Matrix& FromModelSpaceToWorld);
void physCollision2(const Capsule& ca, const SimpleMath::Vector3 & worldDistance, float & T, SimpleMath::Vector3& impactPoint, SimpleMath::Vector3& impactNormal);

namespace Simulation
{
	void AddLedge(const std::string& The_Ledge_Name, const Box& The_Ledge_Box, const int The_Ledge_BoxIndex);
	void AutoKneeling(const SimpleMath::Matrix& FromModelSpaceToWorld);
	void FallingAndHangOn(const SimpleMath::Vector3& CapsuleOrigin, const SimpleMath::Vector3& CapsuleA);
	void LadderDetection();
	void GrabLedgeByHand(SimpleMath::Matrix& FromModelSpaceToWorld);
	void EnterIntoWater(char* CapsuleName, char* ModelTransformName);
	void HoldHand(char* CapsuleName, char* ModelTransformName);
	void HoldToe(char* CapsuleName, char* ModelTransformName);
	void HoldHandWhileBlending(char* CapsuleName, char* ModelTransformName);
	void StartBallisticFly(const SimpleMath::Matrix& FromModelSpaceToWorld, const SimpleMath::Vector3& CapsuleForward);
	void FinishBallisticFly(const SimpleMath::Matrix& CapsuleSystem, const float CapsuleAB, const float CapsuleR);
	void Collision(Capsule& Capsule, SimpleMath::Vector3 DeltaLocation, SimpleMath::Quaternion DeltaRotation);
	void CharacterLerpRotation(float simulationTime, char* CapsuleName, char* ModelTransformName);
	void UpdateCapsuleRotation(Capsule& capsule);
	void UpdateCapsuleRotation_ApplyTargetRootDeltaRotation(char* _animation1, char* _animation2, float BlendTime);
	void UpdateCapsuleRotation_SetParams(const SimpleMath::Quaternion TheTargetRootDeltaRotation, const SimpleMath::Quaternion TheTargetOrientation);
};

const float PI = 3.141592653589793238462643383279;


void FillGrid_Indexed(std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices, DWORD dwWidth, DWORD dwLength, _In_opt_ std::function<float __cdecl(SimpleMath::Vector3)> setHeight);
DirectX::ModelMeshPart* CreateModelMeshPart(ID3D11Device* device, std::function<void(std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices)> createGeometry);
void water_set_object1_matrix(SimpleMath::Matrix v);

struct ClimbingPath
{
	//struct HandState
	//{
	//	SimpleMath::Vector3 GetNewHandLocation();
	//};

	static Ledge Create(const Box* box1);
	static Ledge Create(const Box* box1, const Box* box2);
	//static std::vector<SimpleMath::Vector3> Create(const Box& box1, const Box& box2, const Box& box3);
};

struct IClimbingPathHelper
{
	enum EPath { Edge_Straight, Corner_Inside, Pre_Corner_Outside, Corner_Outside, Pre_Edge_Break_Off, Fold_Edge_Break_Off, Edge_Break_Off };

	virtual void LedgeToSegments() = 0;// SimpleMath::Vector3 HandLocation, char* PathName, int CatchSegmentIndex) = 0;

	virtual SimpleMath::Vector3 ToSegmentBasis(const SimpleMath::Vector3 & Location, int SegmentIndex) = 0;

	virtual int GetCurrentSegmentIndex() = 0;

	virtual EPath Advanse(Animation* Sender, float LocalTime, float GlobalTime, float DeltaTime, int MasterHandIndex, SimpleMath::Vector3 MasterHandLocation, int SlaveHandIndex, SimpleMath::Vector3 SlaveHandLocation, DirectX::XMFLOAT4& RootRotation, SimpleMath::Quaternion& DeltaRotation, SimpleMath::Vector3& DeltaTranslation) = 0;
	
	virtual std::vector<SimpleMath::Vector3> GetPath() = 0;

	virtual void ResetStates() = 0; 

	virtual SimpleMath::Matrix GetEveModelMatrix() = 0;

	virtual ~IClimbingPathHelper();

	virtual void ResetPrevGlobalTime() = 0;

	virtual bool IsSegmentChanging(){ return false; };

	//virtual void PredictHands(Animation* Sender, SimpleMath::Vector3 DeltaTranslation, unsigned int channel, float CurrentAnimationTime, SimpleMath::Vector3 HandTargetLocation){ };
};

IClimbingPathHelper* MakeClimbingPathHelper();

struct Quat
{
	float yaw0;
	float pitch0;
	float roll0;

	float yaw1;
	float pitch1;
	float roll1;

	float DeltaYaw;
	SimpleMath::Quaternion RootRotation;
	SimpleMath::Quaternion DeltaRotation;

	void ExtractPitchYawRollFromXMMatrix(float* flt_p_PitchOut, float* flt_p_YawOut, float* flt_p_RollOut, const DirectX::XMMATRIX* XMMatrix_p_Rotation)
	{
		DirectX::XMFLOAT4X4 XMFLOAT4X4_Values;
		DirectX::XMStoreFloat4x4(&XMFLOAT4X4_Values, DirectX::XMMatrixTranspose(*XMMatrix_p_Rotation));
		*flt_p_PitchOut = (float)asin(-XMFLOAT4X4_Values._23);
		*flt_p_YawOut = (float)atan2(XMFLOAT4X4_Values._13, XMFLOAT4X4_Values._33);
		*flt_p_RollOut = (float)atan2(XMFLOAT4X4_Values._21, XMFLOAT4X4_Values._22);
	}

	Quat& decompose0(SimpleMath::Quaternion q)
	{
		DirectX::XMMATRIX r = SimpleMath::Matrix::CreateFromQuaternion(q);
		ExtractPitchYawRollFromXMMatrix(&pitch0, &yaw0, &roll0, &r);

		return *this;
	}

	Quat& decompose1(SimpleMath::Quaternion q)
	{
		DirectX::XMMATRIX r = SimpleMath::Matrix::CreateFromQuaternion(q);
		ExtractPitchYawRollFromXMMatrix(&pitch1, &yaw1, &roll1, &r);

		return *this;
	}

	Quat& decompose(float YawBase, SimpleMath::Quaternion Prev, SimpleMath::Quaternion Cur)
	{
		decompose0(Prev);
		decompose1(Cur);

		DeltaRotation = SimpleMath::Quaternion::CreateFromYawPitchRoll(DeltaYaw = (yaw1 - yaw0), 0, 0);

		RootRotation = SimpleMath::Quaternion::CreateFromYawPitchRoll(YawBase, pitch1, roll1);

		return *this;
	}
};

struct TDeltaRotation
{
	bool IsValid;
	float RotationAngle;
	SimpleMath::Quaternion Rotation;
	SimpleMath::Vector3 RotationAxis;

	SimpleMath::Quaternion Delta;
	SimpleMath::Quaternion InverseDelta;
	TDeltaRotation(SimpleMath::Vector3 V1 = SimpleMath::Vector3::Zero, SimpleMath::Vector3 V2 = SimpleMath::Vector3::Zero)
	{
		V1.y = 0; V1.Normalize();
		V2.y = 0; V2.Normalize();
		RotationAxis = V1.Cross(V2);
		RotationAngle = atan2(RotationAxis.Length(), V1.Dot(V2));
		IsValid = !XMVector3Equal(RotationAxis, XMVectorZero());
		InverseDelta = Delta = SimpleMath::Quaternion::Identity;
		if (IsValid)
		{
			Delta = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, RotationAngle);
			InverseDelta = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, -RotationAngle);
		}
	}
	TDeltaRotation& Log(std::string msg)
	{
		if (IsValid)
		{
			msg += std::string(" TDeltaRotation RotationAngle == %f\n");
			sprintf(DebugBuffer, msg.data(), (RotationAngle / XM_PI)*180.f); Debug();
		}
		return *this;
	}
	void Add(DirectX::XMFLOAT4& Rotation)
	{
		if (IsValid)
		{
			Rotation = SimpleMath::Quaternion::Concatenate(Delta, Rotation);
		}
	}
	void Substruct(DirectX::XMFLOAT4& Rotation)
	{
		if (IsValid)
		{
			Rotation = SimpleMath::Quaternion::Concatenate(InverseDelta, Rotation);
		}
	}
};

struct TDeltaRotation2
{
	bool IsValid;

	float RotationAngle;
	SimpleMath::Vector3 RotationAxis;

	SimpleMath::Quaternion Delta;
	SimpleMath::Quaternion InverseDelta;

	TDeltaRotation2(SimpleMath::Vector3 V1 = SimpleMath::Vector3::Zero, SimpleMath::Vector3 V2 = SimpleMath::Vector3::Zero) :InverseDelta(SimpleMath::Quaternion::Identity), Delta(SimpleMath::Quaternion::Identity)
	{
		V1.Normalize();
		V2.Normalize();

		RotationAxis = V1.Cross(V2);
		RotationAngle = atan2(RotationAxis.Length(), V1.Dot(V2));

		IsValid = !XMVector3Equal(RotationAxis, XMVectorZero());

		if (IsValid)
		{
			Delta = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, RotationAngle);
			InverseDelta = SimpleMath::Quaternion::CreateFromAxisAngle(RotationAxis, -RotationAngle);
		}
	}
	void Add(DirectX::XMFLOAT4& Rotation)
	{
		Rotation = SimpleMath::Quaternion::Concatenate(Delta, Rotation);
	}
	void Substruct(DirectX::XMFLOAT4& Rotation)
	{
		Rotation = SimpleMath::Quaternion::Concatenate(InverseDelta, Rotation);
	}
};

extern int HipsJointIndex;
extern int LeftToeEndJointIndex;
extern int RightToeEndJointIndex;
extern int LeftHandJointIndex;
extern int RightHandJointIndex;
extern int HeadTopEndJointIndex;
extern JointsRefsChainCollection jointsRefsChains;
extern Skeleton HumanSkeleton;

/////
inline bool _replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

inline SimpleMath::Vector3 _getJointLocationByIndex_MS(const std::vector<JointSQT>& CharactersJoints, const std::vector<int>& ChainsJointsIndex, const int JointIndex, std::vector<JointSQT>& ChainsJoints)
{
	for (int i = 0; i < ChainsJointsIndex.size(); i++){ ChainsJoints[i] = CharactersJoints[ChainsJointsIndex[i]]; };
	JointHelpers::localToModel(ChainsJointsIndex.size(), ChainsJoints);
	return 0.01f*SimpleMath::Vector3(SimpleMath::Vector4(ChainsJoints[JointIndex][2]));
}
/////