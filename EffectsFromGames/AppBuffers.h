#pragma once
struct SceneState
{
	DirectX::XMFLOAT4X4    mWorld;                         // World matrix
	DirectX::XMFLOAT4X4    mView;                          // View matrix
	DirectX::XMFLOAT4X4    mProjection;                    // Projection matrix
	DirectX::XMFLOAT4X4    mWorldViewProjection;           // WVP matrix
	DirectX::XMFLOAT4X4    mWorldView;                     // WV matrix
	DirectX::XMFLOAT4X4    mInvView;                       // Inverse of view matrix

	DirectX::XMFLOAT4X4    mObject1;                // VP matrix
	DirectX::XMFLOAT4X4    mObject1WorldView;                       // Inverse of view matrix
	DirectX::XMFLOAT4X4    mObject1WorldViewProjection;                       // Inverse of view matrix

	DirectX::XMFLOAT4X4    mObject2;                // VP matrix
	DirectX::XMFLOAT4X4    mObject2WorldView;                       // Inverse of view matrix
	DirectX::XMFLOAT4X4    mObject2WorldViewProjection;                       // Inverse of view matrix

	DirectX::XMFLOAT4      vFrustumNearFar;              // Screen resolution
	DirectX::XMFLOAT4      vFrustumParams;              // Screen resolution
	DirectX::XMFLOAT4      vTime;

	DirectX::XMFLOAT4      vClipPlane0;
	DirectX::XMFLOAT4      vClipPlane1;
	DirectX::XMFLOAT4      vClipPlane2;
};

struct PostProccessState
{
	float     ScanDistance;
	float     ScanWidth;
	float     dummy1;
	float     dummy2;

	DirectX::XMFLOAT4    ScanerPosition;
};

struct BoneToModelPalite
{
	DirectX::XMFLOAT4X4    BoneToModelMatrix[1024];
};

struct EvePath
{
	float Count;
	float     dummy1;
	float     dummy2;
	float     dummy3;
	DirectX::XMFLOAT4    Pivots[32];
};