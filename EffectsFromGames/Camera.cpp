#include "main.h"

float m_pitch = 0;
float m_yaw = XM_PI / 2.0f;// -XM_PI / 2.0f;
SimpleMath::Vector2 m_move;

const float ROTATION_GAIN = 0.005f;
const float MOVEMENT_GAIN = 0.02f;

extern std::unique_ptr<Keyboard> _keyboard;
extern std::unique_ptr<Mouse> _mouse;

extern SceneState scene_state;

void updateOrientation(SimpleMath::Vector3 cameraDirection, SimpleMath::Vector3& _cameraPos, SimpleMath::Vector3& _cameraTarget);
void updatePosition(float fElapsedTime, SimpleMath::Vector2 move);

void reñeiveKeyBoardInput(DirectX::Keyboard::State kb);
void reñeiveMouseInput(DirectX::Mouse::State ms);

namespace Camera{
	void CALLBACK OnFrameMove(double fTime, float fElapsedTime, void* pUserContext)
	{
		// Update the camera's position based on user input 
		auto mouseState = _mouse->GetState();

		reñeiveMouseInput(mouseState);

		if (mouseState.leftButton && mouseState.positionMode == Mouse::MODE_RELATIVE)
		{
			SimpleMath::Vector3 delta = SimpleMath::Vector3(float(mouseState.x), float(mouseState.y), 0.f)
				* ROTATION_GAIN;

			m_pitch -= delta.y;
			m_yaw -= delta.x;

			// limit pitch to straight up or straight down
			// with a little fudge-factor to avoid gimbal lock
			float limit = XM_PI / 2.0f - 0.01f;
			m_pitch = max(-limit, m_pitch);
			m_pitch = min(+limit, m_pitch);

			// keep longitude in sane range by wrapping
			if (m_yaw > XM_PI)
			{
				m_yaw -= XM_PI * 2.0f;
			}
			else if (m_yaw < -XM_PI)
			{
				m_yaw += XM_PI * 2.0f;
			}
		}
		_mouse->SetMode(mouseState.leftButton || mouseState.rightButton ? Mouse::MODE_RELATIVE : Mouse::MODE_ABSOLUTE);
		{
			XMMATRIX view;
			XMMATRIX invView;

			float y = sinf(m_pitch);
			float r = cosf(m_pitch);
			float x = r*cosf(m_yaw);
			float z = r*sinf(m_yaw);

			SimpleMath::Vector3 cameraPos, cameraTarget;
			updateOrientation(XMFLOAT3(x, y, z), cameraPos, cameraTarget);

			view = XMMatrixLookAtLH(cameraPos, cameraTarget, SimpleMath::Vector3(0, 1, 0));
			invView = DirectX::XMMatrixInverse(0, view);
			DirectX::XMStoreFloat4x4(&scene_state.mView, DirectX::XMMatrixTranspose(view));
			DirectX::XMStoreFloat4x4(&scene_state.mInvView, DirectX::XMMatrixTranspose(invView));
		}

		auto keyboardState = _keyboard->GetState();

		reñeiveKeyBoardInput(keyboardState);

		{
			m_move = SimpleMath::Vector2::Zero;

			if (keyboardState.Up || keyboardState.W)
				m_move.x += 1.f;

			if (keyboardState.Down || keyboardState.S)
				m_move.x -= 1.f;

			if (keyboardState.Left || keyboardState.A)
				m_move.y -= 1.f;

			if (keyboardState.Right || keyboardState.D)
				m_move.y += 1.f;

			m_move *= MOVEMENT_GAIN;
		}
		{
			updatePosition(fElapsedTime, m_move);
		}
	}
}