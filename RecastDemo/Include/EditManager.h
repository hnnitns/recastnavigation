#pragma once

#include <DirectXMath.h>
#include <array>
#include <string>
#include <vector>

#include "SampleInterfaces.h"

enum class MouseState : uint8_t
{
	Down, Pushing, Up, Free
};

class EditManager
{
public:
	EditManager();
	~EditManager() noexcept;
	EditManager(const EditManager&) = delete;
	auto& operator=(const EditManager&) = delete;
	EditManager(EditManager&&) noexcept = delete;
	auto& operator=(EditManager&&) noexcept = delete;

public:
	_NODISCARD bool Init();
	_NODISCARD bool Update();
	void UnInit();

private:
	bool InputUpdate();
	void CameraUpdate(const float dt);
	void SystemUpdate(const float dt);
	void HandleUpdate(const float dt, const uint32_t time);
	void PropertiesUpdate();
	void SampleSelect();
	void MeshSelect();
	void HandleDraw();

private:
	struct SDL_Window* window{};
	bool presentationMode{};
	DirectX::XMINT2 screen_size{};
	float camr{};
	float tim{};
	float timeAcc{};
	Uint32 prevFrameTime;
	// Used to compute mouse movement totals across frames.
	//フレーム全体のマウスの動きの合計を計算するために使用されます。
	std::array<int, 2u> mousePos{}, origMousePos{};

	std::array<float, 2> cameraEulers{ 45, -45 };
	std::array<float, 3> cameraPos{};
	std::array<float, 2> origCameraEulers{}; // Used to compute rotational changes across frames. //フレーム全体の回転変化を計算するために使用されます。

	float moveFront{}, moveBack{}, moveLeft{}, moveRight{}, moveUp{}, moveDown{};

	float scrollZoom{};
	bool rotate{}, movedDuringRotate{};
	std::array<float, 3> ray_start{}, ray_end{};
	bool mouseOverMenu{}; // ImGui上のウィンドウにマウスが存在する
	bool showMenu{}, showLog{}, showTools{}, showLevels{}, showSample{}, delete_mesh{};

	// Window scroll positions.
	// ウィンドウのスクロール位置。
	int propScroll{}, logScroll{}, toolsScroll{};

	std::string sampleName{ "Choose Sample..." };

	std::vector<std::string> files;
	const std::string meshesFolder{ "Meshes" };
	std::string meshName{ "Choose Mesh..." };

	std::array<float, 3> markerPosition{};
	bool markerPositionSet{};

	/// 流石に見える部分で生ポインタで扱いたくなかった
	std::unique_ptr<class Sample> sample;

	class BuildContext ctx{};

	int mouseScroll{};          // マウスホイールの値
	bool processHitTest{};      // マウスの処理が行われた
	bool processHitTestShift{}; // シフトの処理が行われた

	bool mouse_hit_middle{};
	MouseState mouse_state;

	union SDL_Event event;
	uint8_t mouseButtonMask{};
	double modelviewMatrix[16];
	double projectionMatrix[16];
};