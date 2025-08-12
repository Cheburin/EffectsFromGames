#include "main.h"

#include "DXUTgui.h"
#include "SDKmisc.h"

HWND DXUTgetWindow();

GraphicResources * G;

SceneState scene_state;
PostProccessState post_proccess_state;
BoneToModelPalite bone_to_model_palite;

std::unique_ptr<Keyboard> _keyboard;
std::unique_ptr<Mouse> _mouse;

CDXUTDialogResourceManager          g_DialogResourceManager;
CDXUTTextHelper*                    g_pTxtHelper = NULL;

float ViewPortWidth;

float ViewPortHeight;

#include <codecvt>

Character * Eve;

StaticObject * Pool;

StaticObject * Ladder;

StaticObject * LeftHolster;

StaticObject * RightHolster;

StaticObject * Gun;

SimpleMath::Vector3 LadderSize;

IAnimationGraph2 * EveAnimationGraph;

IKSolverInterface * EveIKSolver;

IClimbingPathHelper* ClimbingPathHelper;

JointsRefsChainCollection jointsRefsChains;

EvePath g_EvePath;

World GWorld;

Character * Mannequin;

Animation * MannequinPose;

std::vector< std::string > GLedgesNames;

extern D3D11_INPUT_ELEMENT_DESC CharacterInputElements[9];

Skeleton HumanSkeleton;

void CreateInputLayoutForCharacter(_In_ ID3D11Device* device, IEffect* effect, _Outptr_ ID3D11InputLayout** pInputLayout)
{
	assert(pInputLayout != 0);

	void const* shaderByteCode;
	size_t byteCodeLength;

	effect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

	device->CreateInputLayout(CharacterInputElements,
		sizeof(CharacterInputElements) / sizeof(CharacterInputElements[0]),
		shaderByteCode, byteCodeLength,
		pInputLayout
	);
}

inline float Deg(double a)
{
	return (PI * a) / 180.0;
}

HRESULT CALLBACK OnD3D11CreateDevice(ID3D11Device* device, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc,
	void* pUserContext)
{
	HRESULT hr;

	ID3D11DeviceContext* context = DXUTGetD3D11DeviceContext();

	G = new GraphicResources();

	G->render_states = std::make_unique<CommonStates>(device);

	G->scene_constant_buffer = std::make_unique<ConstantBuffer<SceneState> >(device);
	G->eve_path_constant_buffer = std::make_unique<ConstantBuffer<EvePath> >(device);
	g_EvePath.Count = 0;
	G->post_proccess_constant_buffer = std::make_unique<ConstantBuffer<PostProccessState> >(device);
	G->bone_to_model_constant_buffer = std::make_unique<ConstantBuffer<BoneToModelPalite> >(device);

	HWND hwnd = DXUTgetWindow();

	_keyboard = std::make_unique<Keyboard>();
	_mouse = std::make_unique<Mouse>();
	_mouse->SetWindow(hwnd);

	g_DialogResourceManager.OnD3D11CreateDevice(device, context);
	g_pTxtHelper = new CDXUTTextHelper(device, context, &g_DialogResourceManager, 15);

	//effects SINGLE FILE FOR ALL EFFECTS
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"GROUND_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"GROUND_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"GROUND_PS", L"ps_5_0" };

		G->ground_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"BOX_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"BOX_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"BOX_PS", L"ps_5_0" };

		G->box_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"BOX_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"BOX_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"LEDGE_BOX_PS", L"ps_5_0" };

		G->ledge_box_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"STD_LIT_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"LEDGE_BOX_PS", L"ps_5_0" };

		G->std_lit_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"BOX_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"BOX_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"BOX_GLOW_PS", L"ps_5_0" };

		G->box_glow_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"BLUR_HORIZONTAL_PS", L"ps_5_0" };

		G->blur_horizontal_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"BLUR_VERTICAL_PS", L"ps_5_0" };

		G->blur_vertical_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"SKY_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"SKY_PS", L"ps_5_0" };

		G->sky_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"WINSTONS_BARRIER_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"WINSTONS_BARRIER_PS", L"ps_5_0" };

		G->winstons_barrier_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"POST_PROCCESS_PS", L"ps_5_0" };

		G->post_proccess_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"POST_PROCCESS_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"COPY_PS", L"ps_5_0" };

		G->copy_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"EVE_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"EVE_PS", L"ps_5_0" };

		G->eve_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"EVE_PATH_VS", L"vs_5_0" };
		shaderDef[L"GS"] = { L"ModelVS.hlsl", L"EVE_PATH_GS", L"gs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"EVE_PATH_PS", L"ps_5_0" };

		G->eve_path_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"UNLIT_WHITE_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"UNLIT_WHITE_PS", L"ps_5_0" };

		G->unlit_white_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"WATER_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"WATER_PS", L"ps_5_0" };

		G->water_effect = createHlslEffect(device, shaderDef);
	}
	{
		std::map<const WCHAR*, EffectShaderFileDef> shaderDef;
		shaderDef[L"VS"] = { L"ModelVS.hlsl", L"LAMBERT_VS", L"vs_5_0" };
		shaderDef[L"PS"] = { L"ModelPS.hlsl", L"LAMBERT_PS", L"ps_5_0" };

		G->lambert_effect = createHlslEffect(device, shaderDef);
	}

	//textures
	{
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\meadow.dds", NULL, NULL, G->sky_cube_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\dirt.dds", NULL, NULL, G->ground_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\normal.dds", NULL, NULL, G->ground_normal_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\hexagon01.png", NULL, NULL, G->hexagon_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Characters\\textures\\SpacePirate_diffuse.png", NULL, NULL, G->eve_d_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Characters\\textures\\SpacePirate_normal.png", NULL, NULL, G->eve_n_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\waternormal.dds", NULL, NULL, G->water_texture.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\T_Default_Material_Grid_M.BMP", NULL, NULL, G->default_Material_Grid_M.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Textures\\T_Default_Material_Grid_N.BMP", NULL, NULL, G->default_Material_Grid_N.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Models\\Holster_Albedo_1.003.png", NULL, NULL, G->Holster_Albedo_M.ReleaseAndGetAddressOf(), NULL);
		hr = D3DX11CreateShaderResourceViewFromFile(device, L"Media\\Models\\Texture_Lighter_Female_Western_Gun_1.003.png", NULL, NULL, G->Female_Western_Gun_M.ReleaseAndGetAddressOf(), NULL);
	}

	//procedural models
	{
		G->sphere_model = GeometricPrimitive::CreateSphere(context, 1, 18U, false, false);
		G->sphere_model->CreateInputLayout(G->sky_effect.get(), G->sphere_input_layout.ReleaseAndGetAddressOf());
		G->sphere_model->CreateInputLayout(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.ReleaseAndGetAddressOf());

		G->cylinder_model = GeometricPrimitive::CreateCylinder(context, 1, 1, 18U, false);

		G->water_model = std::unique_ptr<DirectX::ModelMeshPart>(CreateModelMeshPart(device, [=](std::vector<VertexPositionNormalTangentColorTextureSkinning2> & _vertices, std::vector<UINT> & _indices){
			FillGrid_Indexed(_vertices, _indices, 1, 1, [=](SimpleMath::Vector3 p){
				return 0;
			});
		}));
	}

	GWorld.WorldTransforms["eveSkinnedModel"] = SimpleMath::Matrix::CreateScale(5, 5, 5) * SimpleMath::Matrix::CreateRotationY(-90.0*PI / 180.0);
	auto M = (SimpleMath::Matrix::CreateRotationY((PI * 180) / 180.0));
	auto K = SimpleMath::Vector3::Transform(SimpleMath::Vector3(1, 0, 0), M);

	GWorld.WorldTransforms["MannequinWorldFrame"] = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * SimpleMath::Matrix::CreateScale(1.f / 1.3f) * SimpleMath::Matrix::CreateRotationY(-0.5f*PI + 45.f/180.f*PI) * SimpleMath::Matrix::CreateTranslation(-100, 0, -79);

	//animation models
	{
		Eve = loadCharacter(device, "Media\\Characters\\eve_j_gonzales.dae", HumanSkeleton);
		Mannequin = loadCharacter(device, "Media\\Characters\\Mannequin\\MaleStandingPose.dae", HumanSkeleton);
	}
	{
		HipsJointIndex = HumanSkeleton.JointsIndex.find("Hips")->second;
		LeftToeEndJointIndex = HumanSkeleton.JointsIndex.find("LeftToe_End")->second;
		RightToeEndJointIndex = HumanSkeleton.JointsIndex.find("RightToe_End")->second;
		LeftHandJointIndex = HumanSkeleton.JointsIndex.find("LeftHand")->second;
		RightHandJointIndex = HumanSkeleton.JointsIndex.find("RightHand")->second;
		HeadTopEndJointIndex = HumanSkeleton.JointsIndex.find("HeadTop_End")->second;
	}
	{
		//Eve->fillFramesTransformsRefs();
		//Mannequin->fillFramesTransformsRefs();
	}
	{

		EveAnimationGraph = makeAnimationGraph(makeBlend(), makeAnimationPose());

		jointsRefsChains.init(Eve);

		loadAnimations(EveAnimationGraph, &HumanSkeleton);

		EveIKSolver = makeIKSolver(Eve);

		ClimbingPathHelper = MakeClimbingPathHelper();

		CreateInputLayoutForCharacter(device, G->eve_effect.get(), G->eve_input_layout.GetAddressOf());
	}
	
	//sockets
	{
		void GetRelativeJoint(JointSQT & Joint1, JointSQT & Joint2);
		SimpleMath::Matrix GetFirstChildFrameTransformation(TransformationFrame * Frame);
		std::vector<JointSQT> GetChainsJoints_MS(Animation* Anim, const std::vector<int>& Chain);
		TransformationFrame* findTransformationFrameByName(TransformationFrame * Frame, char * JointName);
		void AddSiblingSocket(Character* CharacterObject, TransformationFrame * Frame, std::string name, SimpleMath::Matrix Transformation = SimpleMath::Matrix::Identity, std::map<std::string, SimpleMath::Matrix> innerFramesTransformations = std::map<std::string, SimpleMath::Matrix>());
		void AddFirstChildSocket(Character* CharacterObject, TransformationFrame * Frame, std::string name, SimpleMath::Matrix Transformation = SimpleMath::Matrix::Identity, std::map<std::string, SimpleMath::Matrix> innerFramesTransformations = std::map<std::string, SimpleMath::Matrix>());
		void attachSocket(Character* CharacterObject, char* socketName, char* destFrameName);
		void addSocketTransformation(Character* CharacterObject, char* frameName, std::pair<std::string, SimpleMath::Matrix> SocketTransformation);
		auto GetJointFromPivotFile = [](ID3D11Device* device, char * file_name)
		{
			StaticObject * HolsterPivotObject = loadStaticObject(device, file_name);
			DirectX::XMVECTOR S, Q, T;
			DirectX::FXMMATRIX M = GetFirstChildFrameTransformation(HolsterPivotObject->frame);
			assert(XMMatrixDecompose(&S, &Q, &T, M));
			JointSQT HolsterPivotJoint_MS;
			HolsterPivotJoint_MS[0] = SimpleMath::Vector4(1.f, 1.f, 1.f, 1.f);
			HolsterPivotJoint_MS[1] = SimpleMath::Vector4(Q);
			HolsterPivotJoint_MS[2] = (1.f / 0.01f) * SimpleMath::Vector4(T);
			delete HolsterPivotObject;
			return HolsterPivotJoint_MS;
		};

		auto TPoseLeftLegJoint_MS = GetChainsJoints_MS((Animation*)EveAnimationGraph->getAnimation("TPose"), jointsRefsChains.HipsLeftToe_End);
		auto TPoseRightLegJoint_MS = GetChainsJoints_MS((Animation*)EveAnimationGraph->getAnimation("TPose"), jointsRefsChains.HipsRightToe_End);

		JointSQT LeftHolsterPivot_MS = GetJointFromPivotFile(device, "Media\\Models\\LeftHolsterPivot.dae");
		JointSQT RightHolsterPivot_MS = GetJointFromPivotFile(device, "Media\\Models\\RightHolsterPivot.dae");

		GetRelativeJoint(TPoseLeftLegJoint_MS[1], LeftHolsterPivot_MS);
		GetRelativeJoint(TPoseRightLegJoint_MS[1], RightHolsterPivot_MS);

		AddSiblingSocket(Eve, findTransformationFrameByName(Eve->frame, "LeftLeg"), "LeftLegHolster", LeftHolsterPivot_MS.matrix(), { { "LeftGun", SimpleMath::Matrix::CreateTranslation(2.5f, 10.f, 5.f) } } );
		AddSiblingSocket(Eve, findTransformationFrameByName(Eve->frame, "RightLeg"), "RightLegHolster", RightHolsterPivot_MS.matrix(), { { "RightGun", SimpleMath::Matrix::CreateTranslation(-2.5f, 10.f, 5.f) } } );

		addSocketTransformation(Eve, "LeftHand", { "LeftGun", SimpleMath::Matrix::CreateTranslation(-0.5,-6.5,0) * SimpleMath::Matrix::CreateRotationZ(PI / 180.f * 90.f) });
		addSocketTransformation(Eve, "RightHand", { "RightGun", SimpleMath::Matrix::CreateTranslation(0.5,-6.5,0) * SimpleMath::Matrix::CreateRotationZ(-PI / 180.f * 90.f) });

		AddFirstChildSocket(Eve, findTransformationFrameByName(Eve->frame, "LeftLegHolster"), "LeftGun");
		AddFirstChildSocket(Eve, findTransformationFrameByName(Eve->frame, "RightLegHolster"), "RightGun");

		attachSocket(Eve, "LeftGun", "LeftLegHolster");
		attachSocket(Eve, "RightGun", "RightLegHolster");
	}

	//static models
	{
		Pool = loadStaticObject(device, "Media\\Models\\pool.dae");
		calculateFramesTransformations(Pool->frame, SimpleMath::Matrix::Identity);

		Ladder = loadStaticObject(device, "Media\\Models\\Ladder.dae");
		//calculateFramesTransformations(Ladder->frame, SimpleMath::Matrix::Identity);

		LeftHolster = loadStaticObject(device, "Media\\Models\\LeftHolster.dae");
		RightHolster = loadStaticObject(device, "Media\\Models\\RightHolster.dae");
		Gun = loadStaticObject(device, "Media\\Models\\Gun.dae");
	}

	//set states 
	{
		post_proccess_state.ScanWidth = 5.0f;

		scene_state.vTime.x = scene_state.vTime.y = scene_state.vTime.z = 0.0f;
	}

	//RStates
	{
		D3D11_BLEND_DESC desc;
		ZeroMemory(&desc, sizeof(desc));

		desc.RenderTarget[0].BlendEnable = true;

		desc.RenderTarget[0].SrcBlend = desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
		desc.RenderTarget[0].DestBlend = desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
		desc.RenderTarget[0].BlendOp = desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;

		desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

		HRESULT hr = device->CreateBlendState(&desc, G->AdditiveBlend.ReleaseAndGetAddressOf());
	}

	using namespace SimpleMath;

	GWorld.Boxes["ground"].size = Vector3(5000, 0.9999f, 5000);
	GWorld.Boxes["ground"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["ground"].origin = Vector3(0, -0.5, 0);

	GWorld.Boxes["platform1"].size = Vector3(15, 0.5, 15);
	GWorld.Boxes["platform1"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform1"].origin = Vector3( -30 + 15.0f / 2, 2, -30 + 15.0f / 2 );

	GWorld.Boxes["platform2"].size = Vector3(15, 0.5, 15);
	GWorld.Boxes["platform2"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform2"].origin = Vector3(-45 + 15.0f / 2, 2.0*4, -30 + 15.0f / 2);

	GWorld.Boxes["platform3"].size = Vector3(15, 0.5, 30);
	GWorld.Boxes["platform3"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationX(Deg(-30.0)));
	auto Y3 = 0.25 - sin(Deg(60.0)) * 0.25;
	auto Z3 = cos(Deg(60.0))*0.25 - 0.00f;
	GWorld.Boxes["platform3"].origin = GWorld.Boxes["platform2"].origin + Vector3(0,Y3,-0.5 * 15 + Z3) + Vector3(0, -sin(Deg(30.0))* 0.5 * 30, -cos(Deg(30.0))*0.5 * 30);

	GWorld.Boxes["platform4"].size = Vector3(30, 8.5, 15);
	GWorld.Boxes["platform4"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform4"].origin = GWorld.Boxes["platform2"].origin + Vector3(-(15+7.5+15),-8+0.5*8.5,7.5f);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	GWorld.Boxes["platform5"].size = Vector3(15, 0.5, 7.5);
	GWorld.Boxes["platform5"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform5"].origin = GWorld.Boxes["platform4"].origin + Vector3(-7.5, 0.5*8.5 + 8 + 0.5, 7.5 + 0.5*7.5 + 7.5);

	GWorld.Boxes["platform6"].size = Vector3(15, 6, 15); //Vector3(15, 0.5, 15);
	GWorld.Boxes["platform6"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform6"].origin = GWorld.Boxes["platform4"].origin + Vector3(7.5, 0.5*8.5 + 8 + 0.5 + 0.25f - 3.f, 7.5 + 0.5 * 15 + 7.5); //Vector3(7.5, 0.5*8.5 + 8 + 0.5, 7.5 + 0.5 * 15 + 7.5)

	GWorld.Boxes["platform7"].size = Vector3(6, 4, 7.5);
	GWorld.Boxes["platform7"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform7"].origin = GWorld.Boxes["platform5"].origin + Vector3(0, 0.5*1+0.5*4 + 5, 0);

	GWorld.Boxes["platform8"].size = Vector3(6, 4, 7.5);
	GWorld.Boxes["platform8"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform8"].origin = GWorld.Boxes["platform6"].origin + Vector3(0, 0.25 + 3 + 0.5*4 + 5, -7.5*0.5); //Vector3(0, 0.5*1 + 0.5 * 4+5, -7.5*0.5);

	GWorld.Boxes["platform9"].size = Vector3(12, 8, 7.5);
	GWorld.Boxes["platform9"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform9"].origin = GWorld.Boxes["platform4"].origin + Vector3(-15, 8.5*0.5+4, -7.5 + -7.5 + -1.25);

	GWorld.Boxes["platform10"].size = Vector3(12, 8.4, 7.5);
	GWorld.Boxes["platform10"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Boxes["platform10"].origin = GWorld.Boxes["platform4"].origin + Vector3(15, -GWorld.Boxes["platform4"].origin.y + 8.4*0.5, -7.5 + -7.5 + -1.25);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	GWorld.Boxes["box"].size = Vector3(10, 14.0, 5);
	GWorld.Boxes["box"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationX((PI * 0.0) / 180.0));
	GWorld.Boxes["box"].origin = Vector3(-5, 0, -5) + Vector3::Transform(0.5f*Vector3(10, 14.0, 5), Matrix::CreateRotationX((PI * 0.0) / 180.0));;

	GWorld.Boxes["sub_box0"].overlap = true;
	GWorld.Boxes["sub_box0"].size = Vector3(10, 0.5, 1.6);
	GWorld.Boxes["sub_box0"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationX((PI * 0.0) / 180.0));
	GWorld.Boxes["sub_box0"].origin = Vector3(-5, 0, -5) + Vector3::Transform(Vector3(5, 9.5, -1.6*0.5), Matrix::CreateRotationX((PI * 0.0) / 180.0));;
	GWorld.Boxes["sub_box0"].evalWorldSize();

	GWorld.Boxes["sub_box1"].overlap = true;
	GWorld.Boxes["sub_box1"].size = Vector3(10, 0.5, 1.6);
	GWorld.Boxes["sub_box1"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationX((PI * 0.0) / 180.0));
	GWorld.Boxes["sub_box1"].origin = Vector3(-5, 0, -5) + Vector3::Transform(Vector3(5, 13.75, -1.6*0.5), Matrix::CreateRotationX((PI * 0.0) / 180.0));;
	GWorld.Boxes["sub_box1"].evalWorldSize();

	GWorld.Boxes["Water"].overlap = true;
	GWorld.Boxes["Water"].size = Vector3(48, 12, 48);
	GWorld.Boxes["Water"].orientation = Quaternion::Identity;
	GWorld.Boxes["Water"].origin = Vector3(25, 6, 25) + Vector3(8, 0, -5);
	GWorld.Boxes["Water"].evalWorldSize();

	/*
	GWorld.Boxes["sub_box2"].overlap = true;
	GWorld.Boxes["sub_box2"].size = Vector3(15, 0.4, 0.4);
	GWorld.Boxes["sub_box2"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationY((PI * 90) / 180.0));
	GWorld.Boxes["sub_box2"].origin = GWorld.Boxes["platform4"].origin + Vector3(15,0.5*8.5,0);
	GWorld.Boxes["sub_box2"].evalWorldSize();
	*/

	/*
	GWorld.Boxes["sub_box3"].overlap = true;
	GWorld.Boxes["sub_box3"].size = Vector3(15, 0.4, 0.4);
	GWorld.Boxes["sub_box3"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationY((PI * 180) / 180.0));
	GWorld.Boxes["sub_box3"].origin = GWorld.Boxes["platform5"].origin + Vector3(0, 0.5*0.5, -0.5*7.5);
	GWorld.Boxes["sub_box3"].evalWorldSize();

	GWorld.Boxes["sub_box4"].overlap = true;
	GWorld.Boxes["sub_box4"].size = Vector3(15, 0.4, 0.4);
	GWorld.Boxes["sub_box4"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationY((PI * 180) / 180.0));
	GWorld.Boxes["sub_box4"].origin = GWorld.Boxes["platform5"].origin + Vector3(0, 0.5*0.5, 0.5*7.5);
	GWorld.Boxes["sub_box4"].evalWorldSize();

	GWorld.Boxes["sub_box5"].overlap = true;
	GWorld.Boxes["sub_box5"].size = Vector3(15, 0.3, 0.3);
	GWorld.Boxes["sub_box5"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::CreateRotationY((PI * 180) / 180.0));
	GWorld.Boxes["sub_box5"].origin = GWorld.Boxes["platform5"].origin + Vector3(0, 0.5*0.5, 0.5*7.5);
	GWorld.Boxes["sub_box5"].evalWorldSize();
	*/

	GWorld.Ellipsoids["barrier"].size = Vector3(20, 20, 20);
	GWorld.Ellipsoids["barrier"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Ellipsoids["barrier"].origin = Vector3(-8, 8, 8);

	GWorld.Capsules["eve"].orientation = Quaternion::CreateFromRotationMatrix(Matrix::Identity);
	GWorld.Capsules["eve"].origin = GWorld.Boxes["platform4"].origin + Vector3(0, 25, 0);// Vector3(0, 0, -20.0f);
	GWorld.Capsules["eve"].ab = 2.0f*(0.01*5.0*72.0)-2.5f;
	GWorld.Capsules["eve"].r = 1.0f;

	GWorld.WorldTransforms["box"] = 
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["box"].size) *
		GWorld.Boxes["box"].getMatrix();

	GWorld.WorldTransforms["sub_box0"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box0"].size) *
		GWorld.Boxes["sub_box0"].getMatrix();

	GWorld.WorldTransforms["sub_box1"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box1"].size) *
		GWorld.Boxes["sub_box1"].getMatrix();

	/*
	GWorld.WorldTransforms["sub_box2"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box2"].size) *
		GWorld.Boxes["sub_box2"].getMatrix();
	*/

	/*
	GWorld.WorldTransforms["sub_box3"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box3"].size) *
		GWorld.Boxes["sub_box3"].getMatrix();
	GWorld.WorldTransforms["sub_box4"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box4"].size) *
		GWorld.Boxes["sub_box4"].getMatrix();
	GWorld.WorldTransforms["sub_box5"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["sub_box5"].size) *
		GWorld.Boxes["sub_box5"].getMatrix();
	*/

	GWorld.WorldTransforms["barrier"] = 
		SimpleMath::Matrix::CreateScale(GWorld.Ellipsoids["barrier"].size) * 
		GWorld.Ellipsoids["barrier"].getMatrix();

	GWorld.WorldTransforms["platform1"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform1"].size) *
		GWorld.Boxes["platform1"].getMatrix();

	GWorld.WorldTransforms["platform2"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform2"].size) *
		GWorld.Boxes["platform2"].getMatrix();

	GWorld.WorldTransforms["platform3"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform3"].size) *
		GWorld.Boxes["platform3"].getMatrix();

	GWorld.WorldTransforms["platform4"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform4"].size) *
		GWorld.Boxes["platform4"].getMatrix();

	//////////////////////////////////////////////////////////////////////
	GWorld.WorldTransforms["platform5"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform5"].size) *
		GWorld.Boxes["platform5"].getMatrix();

	GWorld.WorldTransforms["platform6"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform6"].size) *
		GWorld.Boxes["platform6"].getMatrix();

	GWorld.WorldTransforms["platform7"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform7"].size) *
		GWorld.Boxes["platform7"].getMatrix();

	GWorld.WorldTransforms["platform8"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform8"].size) *
		GWorld.Boxes["platform8"].getMatrix();

	GWorld.WorldTransforms["platform9"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform9"].size) *
		GWorld.Boxes["platform9"].getMatrix();

	GWorld.WorldTransforms["platform10"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform10"].size) *
		GWorld.Boxes["platform10"].getMatrix();
	//////////////////////////////////////////////////////////////////////

	GWorld.WorldTransforms["pool"] =
		SimpleMath::Matrix::CreateRotationX( (PI * 90.0) / 180.0 ) *
		SimpleMath::Matrix::CreateTranslation(25, 6.5, 25) *
		SimpleMath::Matrix::CreateTranslation(8, 0, -5);

	GWorld.WorldTransforms["pools_water"] =
		SimpleMath::Matrix::CreateTranslation(-0.5, 0, -0.5) *
		SimpleMath::Matrix::CreateScale(48, 1, 48) *
		SimpleMath::Matrix::CreateTranslation(25, 12, 25) *
		SimpleMath::Matrix::CreateTranslation(8, 0, -5);

	GWorld.Meshes["pool"].origin = Vector3(25 + 8, 6.5, 25 - 5);
	GWorld.Meshes["pool"].orientation = Quaternion::CreateFromRotationMatrix(SimpleMath::Matrix::CreateRotationX((PI * 90.0) / 180.0));
	Pool->getAllTriangles(GWorld.Meshes["pool"].vertex, GWorld.Meshes["pool"].getMatrix());

	////////////////////////////////////////////////////////////////////////

	GWorld.WorldTransforms["Ladder"] =
		SimpleMath::Matrix::CreateTranslation(0, 0, -0.05f) *
		GWorld.WorldTransforms["eveSkinnedModel"] *
		SimpleMath::Matrix::CreateRotationY((PI * 90.0) / 180.0) *
		SimpleMath::Matrix::CreateScale(Vector3(1.0f / GWorld.Boxes["platform10"].size.x, 1.0f / GWorld.Boxes["platform10"].size.y, 1.0f / GWorld.Boxes["platform10"].size.z)) *
		SimpleMath::Matrix::CreateTranslation(0, 0.5, -0.5) *
		SimpleMath::Matrix::CreateScale(GWorld.Boxes["platform10"].size) * GWorld.Boxes["platform10"].getMatrix();

	{
		#undef min // use __min instead
		#undef max // use __max instead
		std::vector<SimpleMath::Vector3> vertex;
		SimpleMath::Vector3 BoundBoxMin, BoundBoxMax;
		BoundBoxMin = SimpleMath::Vector3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		BoundBoxMax = SimpleMath::Vector3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
		Ladder->getAllTriangles(vertex, SimpleMath::Matrix::Identity);
		for (int i = 0; i < vertex.size(); i++)
		{
			BoundBoxMin.x = __min(BoundBoxMin.x, vertex[i].x);
			BoundBoxMin.y = __min(BoundBoxMin.y, vertex[i].y);
			BoundBoxMin.z = __min(BoundBoxMin.z, vertex[i].z);
			BoundBoxMax.x = __max(BoundBoxMax.x, vertex[i].x);
			BoundBoxMax.y = __max(BoundBoxMax.y, vertex[i].y);
			BoundBoxMax.z = __max(BoundBoxMax.z, vertex[i].z);
		}
		LadderSize = SimpleMath::Vector3::TransformNormal(BoundBoxMax - BoundBoxMin, SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]));
		LadderSize.x = fabs(LadderSize.x); LadderSize.y = fabs(LadderSize.y); LadderSize.z = fabs(LadderSize.z);
	}

	////////////////////////////////////////////////////////////////////////
	GWorld.Ledges["ClimbingPlatform5Platform6"] = ClimbingPath::Create(&GWorld.Boxes["platform5"], &GWorld.Boxes["platform6"]);
	GWorld.Ledges["ClimbingPlatform4"] = ClimbingPath::Create(&GWorld.Boxes["platform4"]);
	GWorld.Ledges["ClimbingPlatform9"] = ClimbingPath::Create(&GWorld.Boxes["platform9"]);
	GWorld.Ledges["Ledge0"] = Ledge::Make(GWorld.Boxes["sub_box0"]);
	GWorld.Ledges["Ledge1"] = Ledge::Make(GWorld.Boxes["sub_box1"]);
	GWorld.Ledges["Water"] = Ledge::Make(GWorld.Boxes["Water"]);
	
	GWorld.Boxes["platform5"].evalWorldSize();
	GWorld.Boxes["platform6"].evalWorldSize();
	GWorld.Boxes["platform4"].evalWorldSize();
	GWorld.Boxes["platform9"].evalWorldSize();
	GWorld.Boxes["sub_box0"].evalWorldSize();
	GWorld.Boxes["sub_box1"].evalWorldSize();

	//static models
	{
		char* Pivot = "platform9";
		char* LedgesGroupName = "Ledges_Group_001";
		StaticObject* wallLedges = loadStaticObject2(device, "Media\\Models\\wallLedges.dae");

		auto FrameOrientation = SimpleMath::Quaternion::Identity;
		auto FrameOrigin = GWorld.Boxes[Pivot].origin + SimpleMath::Vector3(-1.8f*GWorld.Boxes[Pivot].size.x, -GWorld.Boxes[Pivot].origin.y, -11.f);

		calculateFramesTransformations(wallLedges->frame, SimpleMath::Matrix::CreateFromQuaternion(FrameOrientation) * SimpleMath::Matrix::CreateTranslation(FrameOrigin));
		SimpleMath::Vector3 BoundBoxMin, BoundBoxMax;
		GLedgesNames = wallLedges->EmitLedges(LedgesGroupName, BoundBoxMin, BoundBoxMax);

		BoundBoxMin.y = 0.f;
		Box LedgesGroupBox = Box(BoundBoxMin, BoundBoxMax, FrameOrientation);
		LedgesGroupBox.origin += SimpleMath::Vector3(-0.4f, 0, 0);
		LedgesGroupBox.evalWorldSize();
		GWorld.Boxes[LedgesGroupName] = LedgesGroupBox;
		GWorld.WorldTransforms[LedgesGroupName] = SimpleMath::Matrix::CreateTranslation(-0.5, -0.5, -0.5) *	SimpleMath::Matrix::CreateScale(LedgesGroupBox.size) * LedgesGroupBox.getMatrix();
		for (int i = 0; i < GLedgesNames.size(); i++)
		{
			assert(GWorld.Ledges[GLedgesNames[i]].Boxes.size() == 1);
			GWorld.Ledges[GLedgesNames[i]].Owners.push_back(LedgesGroupBox);
		}
		//GLedgesNames.push_back(LedgesGroupName);

		delete wallLedges;
	}

	return S_OK;
}

extern AnimationBase* DebugAnimation;

//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11CreateDevice 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11DestroyDevice(void* pUserContext)
{
	delete g_pTxtHelper;

	delete EveIKSolver;

	delete ClimbingPathHelper;

	delete EveAnimationGraph;

	delete Eve;

	delete Pool;

	delete Ladder;

	delete LeftHolster;

	delete RightHolster;
	
	delete Gun;

	g_DialogResourceManager.OnD3D11DestroyDevice();

	_mouse = 0;

	_keyboard = 0;

	delete G;

	delete DebugAnimation;

	delete Mannequin;

	delete MannequinPose;
}
