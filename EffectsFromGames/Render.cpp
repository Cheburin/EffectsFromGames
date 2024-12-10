#include "main.h"

#include "DXUTgui.h"
#include "SDKmisc.h"

#include <algorithm>
#include <wchar.h>
extern float ViewPortWidth;

extern float ViewPortHeight;

extern GraphicResources * G;

extern SwapChainGraphicResources * SCG;

extern SceneState scene_state;

extern PostProccessState post_proccess_state;

extern BoneToModelPalite bone_to_model_palite;

extern CDXUTTextHelper*                    g_pTxtHelper;

extern Character* Eve;

extern StaticObject * Pool;

extern IAnimationGraph2 * EveAnimationGraph;

extern IKSolverInterface * EveIKSolver;

extern JointsRefsChainCollection jointsRefsChains;

extern EvePath g_EvePath;

extern World GWorld;

ID3D11ShaderResourceView* null[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

const float clearColor[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

bool FallingAndHangOn_TargetToRightHandDefine = false;

SimpleMath::Vector3 FallingAndHangOn_TargetToRight;

SimpleMath::Vector3 FallingAndHangOn_01;

SimpleMath::Vector3 FallingAndHangOn_02;

SimpleMath::Vector3 FallingAndHangOn_03;

SimpleMath::Matrix FallingAndHangOn_Plane;
extern IClimbingPathHelper* ClimbingPathHelper;
//exports
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_scene_world_matrix_into_camera_origin();
void set_ground_world_matrix(SimpleMath::Matrix & w);
void set_box_world_matrix(SimpleMath::Matrix & w);
void set_winstons_barrier_world_matrix(SimpleMath::Matrix & w);
void set_eve_world_matrix(SimpleMath::Matrix & w);
void set_bone_to_model_palite_transforms(SimpleMath::Matrix * p);
void set_eve_path_world_matrix(SimpleMath::Matrix w);
void DrawQuad(ID3D11DeviceContext* pd3dImmediateContext, _In_ IEffect* effect, _In_opt_ std::function<void __cdecl()> setCustomState);
extern SceneState scene_state;
extern float state_CapsulAlfa_WS;
extern float state_HipsAlfa_WS;
//draw helpers
inline void set_scene_constant_buffer(ID3D11DeviceContext* context){
	G->scene_constant_buffer->SetData(context, scene_state);
};
inline void set_post_proccess_constant_buffer(ID3D11DeviceContext* context){
	G->post_proccess_constant_buffer->SetData(context, post_proccess_state);
};
inline void set_bone_to_model_palite_constant_buffer(ID3D11DeviceContext* context){
	G->bone_to_model_constant_buffer->SetData(context, bone_to_model_palite);
}
void RenderText()
{
	g_pTxtHelper->Begin();
	g_pTxtHelper->SetInsertionPos(2, 0);
	g_pTxtHelper->SetForegroundColor(D3DXCOLOR(1.0f, 1.0f, 0.0f, 1.0f));
	g_pTxtHelper->DrawTextLine(DXUTGetFrameStats(true && DXUTIsVsyncEnabled()));
	g_pTxtHelper->DrawTextLine(DXUTGetDeviceStats());

	g_pTxtHelper->SetInsertionPos(2, scene_state.vFrustumParams.y-20.f);
	g_pTxtHelper->SetForegroundColor(D3DXCOLOR(0.0f, 0.0f, 1.0f, 1.0f));

	WCHAR buffer[1024]; swprintf(buffer, L"CapsulAlfa == %f HipsAlfa_WS == %f Delta == %f", state_CapsulAlfa_WS, state_HipsAlfa_WS, state_CapsulAlfa_WS - state_HipsAlfa_WS);
	g_pTxtHelper->DrawTextLine(buffer);

	g_pTxtHelper->End();
}
void clearAndSetRenderTarget(ID3D11DeviceContext* context, const float ClearColor[], int n, ID3D11RenderTargetView** pRTV, ID3D11DepthStencilView* pDSV){
	for (int i = 0; i < n; i++)
		context->ClearRenderTargetView(pRTV[i], ClearColor);

	if (pDSV)
		context->ClearDepthStencilView(pDSV, D3D11_CLEAR_DEPTH, 1.0f, 0);

	context->OMSetRenderTargets(n, pRTV, pDSV);
}
void setViewPort(ID3D11DeviceContext* context, float x1 = 0, float y1 = 0, float Width = 0, float Height = 0){
	D3D11_VIEWPORT vp;
	vp.TopLeftX = 0;
	vp.TopLeftY = 0;
	vp.Width = Width == 0 ? ViewPortWidth : Width;
	vp.Height = Height == 0 ? ViewPortHeight : Height;
	vp.MinDepth = 0;
	vp.MaxDepth = 1;
	context->RSSetViewports(1, &vp);
}

//helpers
float sign(const float& arg){
	return  arg < 0.f ? -1.f : 1.f;
}
SimpleMath::Vector3 ProjectOnGround(const SimpleMath::Vector3 & a)
{
	return SimpleMath::Vector3(a.x, 0, a.z);
}
inline SimpleMath::Matrix make_reflection_view(DirectX::XMFLOAT4 plane){
	auto reflection = DirectX::XMMatrixReflect(XMLoadFloat4(&plane));

	auto invView = XMMatrixTranspose(XMLoadFloat4x4(&scene_state.mInvView));

	DirectX::XMMATRIX reflection_inv_view;
	reflection_inv_view.r[0] = -1 * XMVector4Transform(invView.r[0], reflection);
	reflection_inv_view.r[1] = XMVector4Transform(invView.r[1], reflection);
	reflection_inv_view.r[2] = XMVector4Transform(invView.r[2], reflection);
	reflection_inv_view.r[3] = XMVector4Transform(invView.r[3], reflection);

	auto reflection_view = DirectX::XMMatrixInverse(0, reflection_inv_view);

	return reflection_view;
};

//debug
char DebugBuffer[1024];
void Debug()
{
	OutputDebugStringA(DebugBuffer);
}

bool RefrReflRTBinded = false;
SimpleMath::Matrix waterReflectionView;

bool ShowCapsule = false;
bool ShowJoints = false;

//OnFrame
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void OnRenderGlowObject(ID3D11DeviceContext* context, double fTime, float fElapsedTime);
void OnProccessGlow(ID3D11DeviceContext* context, double fTime, float fElapsedTime);
void OnRenderWaterRefraction(ID3D11DeviceContext* context, double fTime, float fElapsedTime);
void OnRenderWaterReflection(ID3D11DeviceContext* context, double fTime, float fElapsedTime);
void OnRenderScene(ID3D11DeviceContext* context, double fTime, float fElapsedTime);
void PostProccess(ID3D11DeviceContext* context);

void SceneSimulation(double fTime, float fElapsedTime, void* pUserContext);

void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext)
{
	SceneSimulation(fTime, fElapsedTime, pUserContext);
}

namespace
{
	static SimpleMath::Matrix __EvaDebugBase;
}

void CALLBACK OnD3D11FrameRender(ID3D11Device* pd3dDevice, ID3D11DeviceContext* context,
	double fTime, float fElapsedTime, void* pUserContext)
{
	using namespace SimpleMath;

	{
		context->GSSetConstantBuffers(0, 1, constantBuffersToArray(*(G->scene_constant_buffer)));

		context->VSSetConstantBuffers(0, 1, constantBuffersToArray(*(G->scene_constant_buffer)));

		context->VSSetConstantBuffers(1, 1, constantBuffersToArray(*(G->bone_to_model_constant_buffer)));

		context->PSSetSamplers(0, 1, samplerStateToArray(G->render_states->AnisotropicWrap()));

		context->PSSetSamplers(1, 1, samplerStateToArray(G->render_states->PointClamp()));

		context->PSSetShaderResources(0, 8, null);
	}

	OnRenderGlowObject(context, fTime, fElapsedTime);

	OnProccessGlow(context, fTime, fElapsedTime);

	waterReflectionView = make_reflection_view(vec4(SimpleMath::Vector3(0, 1, 0), -(SimpleMath::Matrix(GWorld.WorldTransforms["pools_water"]).Translation().y)));
	water_set_object1_matrix(waterReflectionView);

	RefrReflRTBinded = true;

	setViewPort(context);
	scene_state.vClipPlane0 = vec4(SimpleMath::Vector3(0, -1, 0), -(-SimpleMath::Matrix(GWorld.WorldTransforms["pools_water"]).Translation().y + 0.001f));
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_refraction.Get()), SCG->DS_depth.Get());
	OnRenderWaterRefraction(context, fTime, fElapsedTime);

	setViewPort(context);
	scene_state.vClipPlane0 = vec4(SimpleMath::Vector3(0, 1, 0), -(SimpleMath::Matrix(GWorld.WorldTransforms["pools_water"]).Translation().y + 0.001f));
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_reflection.Get()), SCG->DS_depth.Get());
	OnRenderWaterReflection(context, fTime, fElapsedTime);

	RefrReflRTBinded = false;

	setViewPort(context);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_scene.Get()), SCG->DS_depth.Get());
	scene_state.vClipPlane0 = vec4(SimpleMath::Vector3(0, 1, 0), 1);
	OnRenderScene(context, fTime, fElapsedTime);

	PostProccess(context);

	RenderText();
}

void OnRenderWaterRefraction(ID3D11DeviceContext* context, double fTime, float fElapsedTime)
{
	OnRenderScene(context, fTime, fElapsedTime);
}

void OnRenderWaterReflection(ID3D11DeviceContext* context, double fTime, float fElapsedTime)
{
	auto prevView = scene_state.mView;
	auto prevInvView = scene_state.mInvView;

	scene_state.mView = waterReflectionView.Transpose();
	scene_state.mInvView = waterReflectionView.Invert().Transpose();

	OnRenderScene(context, fTime, fElapsedTime);

	scene_state.mView = prevView;
	scene_state.mInvView = prevInvView;

}

void OnRenderGlowObject(ID3D11DeviceContext* context, double fTime, float fElapsedTime)
{
	using namespace SimpleMath;

	setViewPort(context);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_glowObjects.Get()), DXUTGetD3D11DepthStencilView());

	///render box
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["box"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_glow_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	//down size
	setViewPort(context, 0, 0, int(ViewPortWidth) / 2, int(ViewPortHeight) / 2);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_glowBlurObjects.Get()), 0);

	DrawQuad(context, G->copy_effect.get(), [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(SCG->SR_glowObjects.Get()));
		context->RSSetState(G->render_states->CullNone());
		context->OMSetDepthStencilState(G->render_states->DepthNone(), 0);
	});

	//D3DX11SaveTextureToFile(context, SCG->T_glowBlurObjects.Get(), D3DX11_IFF_DDS, L"glowBlurObjects.dds");
}

void OnProccessGlow(ID3D11DeviceContext* context, double fTime, float fElapsedTime)
{
	using namespace SimpleMath;

	context->PSSetSamplers(0, 1, samplerStateToArray(G->render_states->AnisotropicClamp()));

	setViewPort(context, 0, 0, int(ViewPortWidth) / 2, int(ViewPortHeight) / 2);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_glowBlurObjects1.Get()), 0);

	///blur
	set_scene_constant_buffer(context);
	DrawQuad(context, G->blur_horizontal_effect.get(), [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(SCG->SR_glowBlurObjects.Get()));
		context->RSSetState(G->render_states->CullNone());
		context->OMSetDepthStencilState(G->render_states->DepthNone(), 0);
	});

	context->PSSetShaderResources(3, 1, null);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(SCG->RT_glowBlurObjects.Get()), 0);

	set_scene_constant_buffer(context);
	DrawQuad(context, G->blur_vertical_effect.get(), [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(SCG->SR_glowBlurObjects1.Get()));
		context->RSSetState(G->render_states->CullNone());
		context->OMSetDepthStencilState(G->render_states->DepthNone(), 0);
	});

	context->PSSetSamplers(0, 1, samplerStateToArray(G->render_states->AnisotropicWrap()));
}

////some helpers
//render capsule
void std_set_world_matrix(SimpleMath::Matrix & w);
//extern SimpleMath::Vector3 ballisticFinish;
void helpers_RenderCapsule(ID3D11DeviceContext* context, float fElapsedTime, Capsule & capsule, bool _ShowCapsule)
{
	scene_state.vClipPlane2 = SimpleMath::Vector4(1.f, 1.f, 1.f, 1.f);

	if (ShowJoints)
	{
		getJointFrame(Eve, "Hips", [context](SimpleMath::Matrix M){
			std_set_world_matrix(
				M * SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
			);
			set_scene_constant_buffer(context);
			G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		});
	}

	if (_ShowCapsule)
	{
		SimpleMath::Matrix worldTransform = capsule.getMatrix();

		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(2.0f*capsule.r) *
			SimpleMath::Matrix::CreateTranslation(0.f, capsule.r, 0.f) *
			worldTransform
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});

		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(2.0f*capsule.r) *
			SimpleMath::Matrix::CreateTranslation(0.f, capsule.r + capsule.ab, 0.f) *
			worldTransform
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});

		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(2.0f*capsule.r, capsule.ab, 2.0f*capsule.r) *
			SimpleMath::Matrix::CreateTranslation(0.f, capsule.r + 0.5f*capsule.ab, 0.f) *
			worldTransform
			);
		set_scene_constant_buffer(context);
		G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});

		//std_set_world_matrix(
		//	SimpleMath::Matrix::CreateTranslation(0.f, 0.5f, 0.f) *
		//	SimpleMath::Matrix::CreateScale(.1f*capsule.r, (2.0f*(0.01*5.0*72.0) - 2.5f), .1f*capsule.r) *
		//	SimpleMath::Matrix::CreateRotationZ(-90.f*(3.14f / 180.f)) *
		//	SimpleMath::Matrix::CreateTranslation(0.f, capsule.r + 0.5f*capsule.ab, 0.f) *
		//	worldTransform
		//	);
		//set_scene_constant_buffer(context);
		//G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
		//	context->RSSetState(G->render_states->Wireframe());
		//});

		//////
		std_set_world_matrix(
			SimpleMath::Matrix::CreateTranslation(0.f, 0.5f, 0.f) *
			SimpleMath::Matrix::CreateScale(.1f*capsule.r, capsule.getProb().Length(), .1f*capsule.r) *
			SimpleMath::Matrix::CreateRotationZ(-90.f*(3.14f / 180.f)) *
			capsule.getMiddleCapsuleRaySystem()
		);
		set_scene_constant_buffer(context);
		G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
		//////
	}

	if (ShowJoints)
	{
		TransformationFrame* findTransformationFrameByName(TransformationFrame * Frame, char * JointName);
		SimpleMath::Vector3 GetTranslationFromTransformationFrame(TransformationFrame * Frame);
		extern char * state_hang_on_hand_name;

		scene_state.vClipPlane2 = SimpleMath::Vector4(1.f, 0.f, 0.f, 1.f);

		//if (state_hang_on_hand_name == std::string("LeftHand"))
		{
			auto lh = findTransformationFrameByName(Eve->frame, "LeftHand");
			std_set_world_matrix(
				SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
				SimpleMath::Matrix::CreateTranslation(GetTranslationFromTransformationFrame(lh)) *
				SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
				);
			set_scene_constant_buffer(context);
			G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}

		//if (state_hang_on_hand_name == std::string("RightHand"))
		{
			auto rh = findTransformationFrameByName(Eve->frame, "RightHand");
			std_set_world_matrix(
				SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
				SimpleMath::Matrix::CreateTranslation(GetTranslationFromTransformationFrame(rh)) *
				SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
				);
			set_scene_constant_buffer(context);
			G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}

		{
			auto rh = findTransformationFrameByName(Eve->frame, "HeadTop_End");
			std_set_world_matrix(
				SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
				SimpleMath::Matrix::CreateTranslation(GetTranslationFromTransformationFrame(rh)) *
				SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
				);
			set_scene_constant_buffer(context);
			G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}
	}
	{
		extern SimpleMath::Vector3 state_ballistic_fly_Finish_World_Location;
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.5f, .5f, .5f) *
			SimpleMath::Matrix::CreateTranslation(state_ballistic_fly_Finish_World_Location)
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}
	{
		extern SimpleMath::Vector3 state_ballistic_fly_Start_World_Location;
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.5f, .5f, .5f) *
			SimpleMath::Matrix::CreateTranslation(state_ballistic_fly_Start_World_Location)
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});

		if (EveAnimationGraph->getAnimationName() == "BallisticFly")
		{
			SimpleMath::Vector3 BallisticAnimation_SimulatePath(AnimationBase * Animation, float Time);
			const auto modelTransform = SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"])
				*SimpleMath::Matrix::CreateFromQuaternion(GWorld.Capsules["eve"].orientation)
				*SimpleMath::Matrix::CreateTranslation(state_ballistic_fly_Start_World_Location);

			for (int i = 0; i < 0; i++)
			{
				const auto _Location = BallisticAnimation_SimulatePath(EveAnimationGraph->_getAnimation(), i*0.01f);
				std_set_world_matrix(
					SimpleMath::Matrix::CreateScale(.25f, .25f, .25f) *
					SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3::Transform(_Location, modelTransform))
				);
				set_scene_constant_buffer(context);
				G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
					context->RSSetState(G->render_states->Wireframe());
				});
			}
		}
	}
	if (ShowJoints)
	{
		SimpleMath::Vector3 GetTranslationFromTransformationFrame(TransformationFrame * Frame);
		TransformationFrame* findTransformationFrameByName(TransformationFrame * Frame, char * JointName);

		auto rh = findTransformationFrameByName(Eve->frame, "RightToe_End");
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
			SimpleMath::Matrix::CreateTranslation(GetTranslationFromTransformationFrame(rh)) *
			SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}

	if (ShowJoints)
	{
		SimpleMath::Vector3 GetTranslationFromTransformationFrame(TransformationFrame * Frame);
		TransformationFrame* findTransformationFrameByName(TransformationFrame * Frame, char * JointName);

		auto rh = findTransformationFrameByName(Eve->frame, "LeftToe_End");
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
			SimpleMath::Matrix::CreateTranslation(GetTranslationFromTransformationFrame(rh)) *
			SimpleMath::Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix()
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}

	if (FallingAndHangOn_TargetToRightHandDefine)
	{
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.5f, .5f, .5f) *
			SimpleMath::Matrix::CreateTranslation(FallingAndHangOn_TargetToRight)
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}
}
////

extern SimpleMath::Vector3 PredictHandHangLocation;

void OnRenderScene(ID3D11DeviceContext* context, double fTime, float fElapsedTime)
{
	using namespace SimpleMath;
	{
		std_set_world_matrix(
			SimpleMath::Matrix::CreateScale(.3f, .3f, .3f) *
			SimpleMath::Matrix::CreateTranslation(PredictHandHangLocation)
			);
		set_scene_constant_buffer(context);
		G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}

	///simple test path
	{
		//auto& Path = GWorld.Pathes["ClimbingPlatform5Platform6"];
		auto Path = ClimbingPathHelper->GetPath();
		static bool Init = false;
		static int PathCurrentNodeIndex = 0;
		static SimpleMath::Vector3 Location;
		if (!Init)
		{
			if (Path.size() > 1)
			{
				Init = true;
				Location = Path[0];
			}
		}
		if (Path.size() > 1)
		{
			auto A = Path[PathCurrentNodeIndex];
			auto B = Path[(PathCurrentNodeIndex + 1) % Path.size()];
			auto PathForward = B - A;
			PathForward.Normalize();

			auto C = Location + 0.5f * fElapsedTime * PathForward;
			auto AC = C - A;

			if (AC.Dot(PathForward) > (B - A).Length())
			{
				Location = B;
				PathCurrentNodeIndex++;
				if (PathCurrentNodeIndex == Path.size())
				{
					PathCurrentNodeIndex = 0;
				}
			}
			else
			{
				Location = C;
			}

			std_set_world_matrix(
				SimpleMath::Matrix::CreateScale(.1f, .1f, .1f) *
				SimpleMath::Matrix::CreateTranslation(Location)
			);
			set_scene_constant_buffer(context);
			G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}
	}
	///

	///render sky sphere
	set_scene_world_matrix_into_camera_origin();
	set_scene_constant_buffer(context);
	G->sphere_model->Draw(G->sky_effect.get(), G->sphere_input_layout.Get(), false, false, [=]{
		context->PSSetShaderResources(2, 1, shaderResourceViewToArray(G->sky_cube_texture.Get()));

		context->RSSetState(G->render_states->CullClockwise());
	});

	///render ground
	set_ground_world_matrix(Matrix::CreateTranslation(-0.5, -0.5, 0) * Matrix::CreateScale(5000, 5000, 5000) * Matrix::CreateRotationX(PI/2.0));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->ground_effect.get(), [=]{
		context->PSSetShaderResources(1, 1, shaderResourceViewToArray(G->ground_normal_texture.Get()));
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(G->ground_texture.Get()));

		context->RSSetState(G->render_states->CullNone());
	});

	///render box
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["box"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render sub box
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box0"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box1"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	/*
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box2"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->Wireframe());
	});
	*/

	for (auto& Path : GWorld.Ledges)
	{
		for (auto& Box : Path.second.Boxes)
		{
			set_box_world_matrix(Box.worldTransform);
			set_scene_constant_buffer(context);
			DrawQuad(context, G->box_effect.get(), [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}
	}

	/*
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box3"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->Wireframe());
	});

	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box4"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->Wireframe());
	});

	set_box_world_matrix(Matrix(GWorld.WorldTransforms["sub_box5"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->Wireframe());
	});
	*/

	///render platform1
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform1"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform2
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform2"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform3
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform3"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});
	
	///render platform4
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform4"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform5
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform5"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform6
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform6"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform7
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform7"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform8
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform8"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render platform9
	set_box_world_matrix(Matrix(GWorld.WorldTransforms["platform9"]));
	set_scene_constant_buffer(context);
	DrawQuad(context, G->box_effect.get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render pool
	std_set_world_matrix(Matrix(GWorld.WorldTransforms["pool"]));
	set_scene_constant_buffer(context);
	///all transform must be bake into mesh, frame transform ignore
	drawFrames(Pool->frame, context, G->std_lit_effect.get(), G->eve_input_layout.Get(), [=]{
		context->RSSetState(G->render_states->CullCounterClockwise());
	});

	///render pools water
	std_set_world_matrix(Matrix(GWorld.WorldTransforms["pools_water"]));
	set_scene_constant_buffer(context);
	G->water_model->Draw(context, G->water_effect.get(), G->eve_input_layout.Get(), [=]{
		context->PSSetShaderResources(1, 1, shaderResourceViewToArray(G->water_texture.Get()));

		if (!RefrReflRTBinded)
		{
			context->PSSetShaderResources(4, 1, shaderResourceViewToArray(SCG->SR_refraction.Get()));
			context->PSSetShaderResources(5, 1, shaderResourceViewToArray(SCG->SR_reflection.Get()));
		}

		context->OMSetBlendState(G->render_states->Opaque(), Colors::Black, 0xFFFFFFFF);
		context->RSSetState(G->render_states->CullNone());
		context->OMSetDepthStencilState(G->render_states->DepthDefault(), 0);
	});

	///render eve path
	//G->eve_path_constant_buffer
	if (g_EvePath.Count > 1)
	{
		set_eve_path_world_matrix(Matrix::Identity);
		G->eve_path_constant_buffer->SetData(context, g_EvePath);
		set_scene_constant_buffer(context);
		DrawQuad(context, G->eve_path_effect.get(), [=]{
			context->VSSetConstantBuffers(2, 1, constantBuffersToArray(*(G->eve_path_constant_buffer)));
			context->GSSetConstantBuffers(2, 1, constantBuffersToArray(*(G->eve_path_constant_buffer)));

			context->RSSetState(G->render_states->Wireframe());
		});
	}
	if (EveAnimationGraph->getAnimationName() == "JumpFromWall")
	{
		SimpleMath::Vector3 GetJumpFromWallTrajectoryHandlerLocation(Animation *Anim, int index);
		for (int i = 0; i < 4; i++)
		{
			const auto _Location = GetJumpFromWallTrajectoryHandlerLocation(EveAnimationGraph->getAnimation<Animation>(), i);
			std_set_world_matrix(
				SimpleMath::Matrix::CreateScale(.25f, .25f, .25f) *
				SimpleMath::Matrix::CreateTranslation(_Location)
				);
			set_scene_constant_buffer(context);
			G->sphere_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
				context->RSSetState(G->render_states->Wireframe());
			});
		}
	};
	///render eve
	if (true)
	{
		SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);
		AnimationBase* animation;
		if (EveAnimationGraph->getAnimationBlend()->isPlaying())
		{
			animation = EveAnimationGraph->getAnimationBlend();
		}
		else
		{
			animation = EveAnimationGraph->_getAnimation();
		}
		for (int i = 0; i < animation->CurrentJoints.size(); i++)
		{
			auto & cj = animation->CurrentJoints[i];

			(*GetSkeletonMatrix(Eve->skelet, i)) = cj.matrix();
		}
		calculateFramesTransformations(Eve->frame, Matrix::Identity);
	}
	if (false)
	{
		auto X = SimpleMath::Matrix::CreateRotationX(-90.f*(3.14f / 180.f));
		auto Y = SimpleMath::Vector3::TransformNormal(SimpleMath::Vector3(0, 1, 0), X);
		SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);
		set_eve_world_matrix(
			SimpleMath::Matrix::CreateTranslation(0.f, 0.5f, 0.f) *
			SimpleMath::Matrix::CreateScale(5, 100, 5) * 
			SimpleMath::Matrix::CreateRotationX(-90.f*(3.14f / 180.f)) *
			*GetSkeletonMatrix(Eve->skelet, 64) *
			Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) *
			GWorld.Capsules["eve"].getMatrix()
		);
		set_scene_constant_buffer(context);
		G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
			context->RSSetState(G->render_states->Wireframe());
		});
	}
	{
		set_eve_world_matrix(Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * GWorld.Capsules["eve"].getMatrix());
		set_scene_constant_buffer(context);
	}
	{
		set_bone_to_model_palite_transforms(calculateAnimationPalite(Eve->skelet));
		set_bone_to_model_palite_constant_buffer(context);
	}
	if (!ShowJoints) drawFrames(Eve->frame, context, G->eve_effect.get(), G->eve_input_layout.Get(), [=]{
		context->PSSetShaderResources(1, 1, shaderResourceViewToArray(G->eve_n_texture.Get()));
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(G->eve_d_texture.Get()));

		context->RSSetState(G->render_states->CullCounterClockwise());
	});
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//debug poses
	if (false && EveAnimationGraph->getAnimationName() == "Hanging_Idle_With_Leg")
	{
		SimpleMath::Matrix GetJumpFromWallStartTransform(AnimationBase *Anim);
		SimpleMath::Matrix* GetSkeletonMatrix(CharacterSkelet * skelet, int index);
		std::vector<JointSQT>& GetAnimationJointsSet(AnimationBase* blend, int index, float Time = 0);
		auto AnimationBlend = EveAnimationGraph->getAnimationBlend();
		if (AnimationBlend->isPlaying())
		{

			for (int i = 1; i < 3; i++)
			{
				auto Base = SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3(0, -i*2.5f, 0))*__EvaDebugBase; // (i - 1)*10.f
				//auto T = GWorld.Capsules["eve"].getMatrix().Translation();
				//auto Base = SimpleMath::Matrix(
				//	SimpleMath::Vector4(0,0,1,0),
				//	SimpleMath::Vector4(0,1,0,0),
				//	SimpleMath::Vector4(-1, 0, 0, 0),
				//	SimpleMath::Vector4(T.x, T.y, T.z, 1));
				//Base = SimpleMath::Matrix::CreateTranslation(SimpleMath::Vector3(0, -i*5.f, 0))*Base;
				if (false && i == 2)
				{
					Base._41 = GWorld.Capsules["eve"].getMatrix().Translation().x;
					Base._42 = GWorld.Capsules["eve"].getMatrix().Translation().y;
					Base._43 = GWorld.Capsules["eve"].getMatrix().Translation().z;
				}
				std::vector<JointSQT>& JointsSet = GetAnimationJointsSet(AnimationBlend, i);
				for (int j = 0; j < JointsSet.size(); j++)
				{
					(*GetSkeletonMatrix(Eve->skelet, j)) = JointsSet[j].matrix();
				}
				//
				calculateFramesTransformations(Eve->frame, Matrix::Identity);
				{
					set_eve_world_matrix(Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) * Base);
					set_scene_constant_buffer(context);
				}
				{
					set_bone_to_model_palite_transforms(calculateAnimationPalite(Eve->skelet));
					set_bone_to_model_palite_constant_buffer(context);
				}
				drawFrames(Eve->frame, context, G->eve_effect.get(), G->eve_input_layout.Get(), [=]{
					context->PSSetShaderResources(1, 1, shaderResourceViewToArray(G->eve_n_texture.Get()));
					context->PSSetShaderResources(3, 1, shaderResourceViewToArray(G->eve_d_texture.Get()));

					context->RSSetState(G->render_states->CullCounterClockwise());
				});
				set_eve_world_matrix(
					SimpleMath::Matrix::CreateTranslation(0.f, 0.5f, 0.f) *
					SimpleMath::Matrix::CreateScale(5, 100, 5) *
					SimpleMath::Matrix::CreateRotationX(-90.f*(3.14f / 180.f)) *
					*GetSkeletonMatrix(Eve->skelet, 64) *
					Matrix(GWorld.WorldTransforms["eveSkinnedModel"]) *
					Base
				);
				set_scene_constant_buffer(context);
				G->cylinder_model->Draw(G->unlit_white_effect.get(), G->dxtk_primitive_to_unlit_white_input_layout.Get(), false, false, [=]{
					context->RSSetState(G->render_states->Wireframe());
				});
				//
			}
		}
	}
	else
	{
		__EvaDebugBase = GWorld.Capsules["eve"].getMatrix();
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	helpers_RenderCapsule(context, fElapsedTime, GWorld.Capsules["eve"], ShowCapsule);

	extern std::vector<Capsule> DebugCapsule;
	int i = 0;
	for (auto TheCapsule : DebugCapsule)
	{
		TheCapsule.origin = TheCapsule.origin + (i++)*SimpleMath::Vector3(0, 0, 2.f);
		helpers_RenderCapsule(context, fElapsedTime, TheCapsule, true);
	}

	//copy depth 
	context->CopyResource(SCG->T_depth1.Get(), SCG->T_depth.Get());

	//render Winston's Barrier
	set_winstons_barrier_world_matrix(Matrix(GWorld.WorldTransforms["barrier"]));
 	set_scene_constant_buffer(context);
	G->sphere_model->Draw(G->winstons_barrier_effect.get(), G->sphere_input_layout.Get(), false, false, [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(G->hexagon_texture.Get()));
		context->PSSetShaderResources(6, 1, shaderResourceViewToArray(SCG->SR_depth1.Get()));
		context->OMSetBlendState(G->AdditiveBlend.Get(), Colors::Black, 0xFFFFFFFF);

		context->RSSetState(G->render_states->CullClockwise());
	});
	G->sphere_model->Draw(G->winstons_barrier_effect.get(), G->sphere_input_layout.Get(), false, false, [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(G->hexagon_texture.Get()));
		context->PSSetShaderResources(6, 1, shaderResourceViewToArray(SCG->SR_depth1.Get()));
		context->OMSetBlendState(G->AdditiveBlend.Get(), Colors::Black, 0xFFFFFFFF);

		context->RSSetState(G->render_states->CullCounterClockwise());
	});
}

void PostProccess(ID3D11DeviceContext* context)
{
	context->PSSetConstantBuffers(1, 1, constantBuffersToArray(*(G->post_proccess_constant_buffer)));

	context->PSSetSamplers(0, 1, samplerStateToArray(G->render_states->AnisotropicClamp()));

	setViewPort(context);
	clearAndSetRenderTarget(context, clearColor, 1, renderTargetViewToArray(DXUTGetD3D11RenderTargetView()), 0);

	set_post_proccess_constant_buffer(context);
	DrawQuad(context, G->post_proccess_effect.get(), [=]{
		context->PSSetShaderResources(3, 1, shaderResourceViewToArray(SCG->SR_glowObjects.Get()));
		context->PSSetShaderResources(4, 1, shaderResourceViewToArray(SCG->SR_glowBlurObjects.Get()));
		context->PSSetShaderResources(5, 1, shaderResourceViewToArray(SCG->SR_scene.Get()));
		context->PSSetShaderResources(6, 1, shaderResourceViewToArray(SCG->SR_depth.Get()));

		context->RSSetState(G->render_states->CullNone());
		context->OMSetDepthStencilState(G->render_states->DepthNone(), 0);
	});

	context->PSSetSamplers(0, 1, samplerStateToArray(G->render_states->AnisotropicWrap()));
}