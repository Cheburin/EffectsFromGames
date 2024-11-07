#include "main.h"

#include "DXUTgui.h"
#include "SDKmisc.h"

extern SceneState scene_state;

extern float ViewPortWidth;

extern float ViewPortHeight;

extern CDXUTTextHelper*                    g_pTxtHelper;
extern CDXUTDialogResourceManager          g_DialogResourceManager;

bool state_ortho_proj = false;

bool state_pers_proj = true;

SwapChainGraphicResources * SCG;
//--------------------------------------------------------------------------------------
// Create any D3D11 resources that depend on the back buffer
//--------------------------------------------------------------------------------------
HRESULT CALLBACK OnD3D11ResizedSwapChain(ID3D11Device* device, IDXGISwapChain* pSwapChain,
	const DXGI_SURFACE_DESC* backBufferSurfaceDesc, void* pUserContext)
{
	float ratio = (ViewPortWidth = backBufferSurfaceDesc->Width) / (ViewPortHeight = backBufferSurfaceDesc->Height);

	float np = 0.1, fp = 500, fov = D3DX_PI / 3;

	g_DialogResourceManager.OnD3D11ResizedSwapChain(device, backBufferSurfaceDesc);

	if(state_pers_proj)
		DirectX::XMStoreFloat4x4(&scene_state.mProjection, DirectX::XMMatrixTranspose(DirectX::XMMatrixPerspectiveFovLH(fov, ratio, np, fp)));

	if (state_ortho_proj)
		DirectX::XMStoreFloat4x4(&scene_state.mProjection, DirectX::XMMatrixTranspose(DirectX::XMMatrixOrthographicLH(50, 50, np, fp)));

	scene_state.vFrustumParams = SimpleMath::Vector4(backBufferSurfaceDesc->Width, backBufferSurfaceDesc->Height, scene_state.mProjection._11, scene_state.mProjection._22);

	scene_state.vFrustumNearFar = SimpleMath::Vector4(np, fp, 0, 0);

	SCG = new SwapChainGraphicResources();

	HRESULT hr;
    //
	D3D11_TEXTURE2D_DESC textureDesc;
	textureDesc.Width = backBufferSurfaceDesc->Width;
	textureDesc.Height = backBufferSurfaceDesc->Height;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.SampleDesc.Quality = 0;
	textureDesc.Usage = D3D11_USAGE_DEFAULT;
	textureDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_DEPTH_STENCIL;
	textureDesc.CPUAccessFlags = 0;
	textureDesc.MiscFlags = 0;
	//

	textureDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
	textureDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;

	//////water reflaction,refraction
	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_reflection.ReleaseAndGetAddressOf());
	hr = device->CreateRenderTargetView(SCG->T_reflection.Get(), nullptr, SCG->RT_reflection.ReleaseAndGetAddressOf());
	hr = device->CreateShaderResourceView(SCG->T_reflection.Get(), nullptr, SCG->SR_reflection.ReleaseAndGetAddressOf());

	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_refraction.ReleaseAndGetAddressOf());
	hr = device->CreateRenderTargetView(SCG->T_refraction.Get(), nullptr, SCG->RT_refraction.ReleaseAndGetAddressOf());
	hr = device->CreateShaderResourceView(SCG->T_refraction.Get(), nullptr, SCG->SR_refraction.ReleaseAndGetAddressOf());
	//////

	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_glowObjects.ReleaseAndGetAddressOf());
	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_scene.ReleaseAndGetAddressOf());
	textureDesc.Width = backBufferSurfaceDesc->Width / 2;
	textureDesc.Height = backBufferSurfaceDesc->Height / 2;
	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_glowBlurObjects.ReleaseAndGetAddressOf());
	hr = device->CreateTexture2D(&textureDesc, nullptr, SCG->T_glowBlurObjects1.ReleaseAndGetAddressOf());

	hr = device->CreateRenderTargetView(SCG->T_glowObjects.Get(), nullptr, SCG->RT_glowObjects.ReleaseAndGetAddressOf());
	hr = device->CreateRenderTargetView(SCG->T_scene.Get(), nullptr, SCG->RT_scene.ReleaseAndGetAddressOf());
	hr = device->CreateRenderTargetView(SCG->T_glowBlurObjects.Get(), nullptr, SCG->RT_glowBlurObjects.ReleaseAndGetAddressOf());
	hr = device->CreateRenderTargetView(SCG->T_glowBlurObjects1.Get(), nullptr, SCG->RT_glowBlurObjects1.ReleaseAndGetAddressOf());

	hr = device->CreateShaderResourceView(SCG->T_glowObjects.Get(), nullptr, SCG->SR_glowObjects.ReleaseAndGetAddressOf());
	hr = device->CreateShaderResourceView(SCG->T_scene.Get(), nullptr, SCG->SR_scene.ReleaseAndGetAddressOf());
	hr = device->CreateShaderResourceView(SCG->T_glowBlurObjects.Get(), nullptr, SCG->SR_glowBlurObjects.ReleaseAndGetAddressOf());
	hr = device->CreateShaderResourceView(SCG->T_glowBlurObjects1.Get(), nullptr, SCG->SR_glowBlurObjects1.ReleaseAndGetAddressOf());

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	{
		D3D11_TEXTURE2D_DESC _desc;
		_desc.Width = backBufferSurfaceDesc->Width;
		_desc.Height = backBufferSurfaceDesc->Height;
		_desc.MipLevels = 1;
		_desc.ArraySize = 1;
		_desc.Format = DXGI_FORMAT_R32_TYPELESS;
		_desc.SampleDesc.Count = 1;
		_desc.SampleDesc.Quality = 0;
		_desc.Usage = D3D11_USAGE_DEFAULT;
		_desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_DEPTH_STENCIL;
		_desc.CPUAccessFlags = 0;
		_desc.MiscFlags = 0;

		D3D11_DEPTH_STENCIL_VIEW_DESC _dsv_desc;
		_dsv_desc.Flags = 0;
		_dsv_desc.Format = DXGI_FORMAT_D32_FLOAT;
		_dsv_desc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
		_dsv_desc.Texture2D.MipSlice = 0;

		D3D11_SHADER_RESOURCE_VIEW_DESC _sr_desc;
		_sr_desc.Format = DXGI_FORMAT_R32_FLOAT;
		_sr_desc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		_sr_desc.Texture2D.MostDetailedMip = 0;
		_sr_desc.Texture2D.MipLevels = 1;

		hr = device->CreateTexture2D(&_desc, 0, SCG->T_depth.ReleaseAndGetAddressOf());
		hr = device->CreateDepthStencilView(SCG->T_depth.Get(), &_dsv_desc, SCG->DS_depth.ReleaseAndGetAddressOf());
		hr = device->CreateShaderResourceView(SCG->T_depth.Get(), &_sr_desc, SCG->SR_depth.ReleaseAndGetAddressOf());

		hr = device->CreateTexture2D(&_desc, 0, SCG->T_depth1.ReleaseAndGetAddressOf());
		hr = device->CreateDepthStencilView(SCG->T_depth1.Get(), &_dsv_desc, SCG->DS_depth1.ReleaseAndGetAddressOf());
		hr = device->CreateShaderResourceView(SCG->T_depth1.Get(), &_sr_desc, SCG->SR_depth1.ReleaseAndGetAddressOf());
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	return S_OK;
}
//--------------------------------------------------------------------------------------
// Release D3D11 resources created in OnD3D11ResizedSwapChain 
//--------------------------------------------------------------------------------------
void CALLBACK OnD3D11ReleasingSwapChain(void* pUserContext)
{
	g_DialogResourceManager.OnD3D11ReleasingSwapChain();

	delete SCG;
}