#include "main.h"

extern GraphicResources * G;

extern SceneState scene_state;

extern BoneToModelPalite bone_to_model_palite;

using namespace SimpleMath;

void loadMatrix_VP(Matrix & v, Matrix & invV, Matrix & p){
	v = Matrix(scene_state.mView).Transpose();
	p = Matrix(scene_state.mProjection).Transpose();
	invV = Matrix(scene_state.mInvView).Transpose();
}
void loadMatrix_WP(Matrix & w, Matrix & p){
	w = Matrix(scene_state.mWorld).Transpose();
	p = Matrix(scene_state.mProjection).Transpose();
}
void storeMatrix(Matrix & w, Matrix & wv, Matrix & wvp){
	scene_state.mWorld = w.Transpose();
	scene_state.mWorldView = wv.Transpose();
	scene_state.mWorldViewProjection = wvp.Transpose();
}

void DrawQuad(ID3D11DeviceContext* pd3dImmediateContext, _In_ IEffect* effect,
	_In_opt_ std::function<void __cdecl()> setCustomState){
	effect->Apply(pd3dImmediateContext);
	setCustomState();

	pd3dImmediateContext->IASetInputLayout(nullptr);
	pd3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);// D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	pd3dImmediateContext->Draw(1, 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void _set_world_matrix(){
	Matrix w, v, invV, p;
	loadMatrix_VP(v, invV, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void std_set_world_matrix(Matrix & w){
	Matrix v, invV, p;
	loadMatrix_VP(v, invV, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_scene_world_matrix_into_camera_origin(){
	Matrix w, v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	w = Matrix::CreateTranslation(inv_v._41, inv_v._42, inv_v._43);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_ground_world_matrix(Matrix & w){
	Matrix v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_box_world_matrix(Matrix & w){
	Matrix v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_eve_path_world_matrix(Matrix w){
	Matrix v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_winstons_barrier_world_matrix(Matrix & w){
	Matrix v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_eve_world_matrix(Matrix & w){
	Matrix v, inv_v, p;
	loadMatrix_VP(v, inv_v, p);

	Matrix  wv = w * v;
	Matrix wvp = wv * p;

	storeMatrix(w, wv, wvp);
}

void set_bone_to_model_palite_transforms(Matrix * p)
{
	for (int i = 0; i < 1024; i++)
	{
		bone_to_model_palite.BoneToModelMatrix[i] = p[i].Transpose();
	}
}

void water_set_object1_matrix(SimpleMath::Matrix v){
	Matrix w, p;
	loadMatrix_WP(w, p);

	Matrix  wvp = v * p;

	scene_state.mObject1WorldViewProjection = wvp.Transpose();
}