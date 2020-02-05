#pragma once

#include <DirectXMath.h>
#include <array>
#include <string>
#include <vector>

#include "SampleInterfaces.h"

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

private:
	struct SDL_Window* window{};
	bool presentationMode{};
	DirectX::XMINT2 screen_size{};
	float camr{};
	float tim{};
	float timeAcc{};
	Uint32 prevFrameTime;
	// Used to compute mouse movement totals across frames.
	//�t���[���S�̂̃}�E�X�̓����̍��v���v�Z���邽�߂Ɏg�p����܂��B
	std::array<int, 2u> mousePos{}, origMousePos{};

	std::array<float, 2> cameraEulers{ 45, -45 };
	std::array<float, 3> cameraPos{};
	std::array<float, 2> origCameraEulers{}; // Used to compute rotational changes across frames. //�t���[���S�̂̉�]�ω����v�Z���邽�߂Ɏg�p����܂��B

	float moveFront{}, moveBack{}, moveLeft{}, moveRight{}, moveUp{}, moveDown{};

	float scrollZoom{};
	bool rotate{}, movedDuringRotate{};
	std::array<float, 3> ray_start{}, ray_end{};
	bool mouseOverMenu{}; // ImGui��̃E�B���h�E�Ƀ}�E�X�����݂���
	bool showMenu{}, showLog{}, showTools{}, showLevels{}, showSample{}, showTestCases{};

	// Window scroll positions.
	// �E�B���h�E�̃X�N���[���ʒu�B
	int propScroll{}, logScroll{}, toolsScroll{};

	std::string sampleName{ "Choose Sample..." };

	std::vector<std::string> files;
	const std::string meshesFolder{ "Meshes" };
	std::string meshName{ "Choose Mesh..." };

	std::array<float, 3> markerPosition{};
	bool markerPositionSet{};

	/// ���΂Ɍ����镔���Ő��|�C���^�ň��������Ȃ�����
	std::unique_ptr<class InputGeom> geom;
	std::unique_ptr<class Sample> sample;

	const std::string testCasesFolder{ "TestCases" };
	std::unique_ptr<class TestCase> test;

	class BuildContext ctx{};

	int mouseScroll{};          // �}�E�X�z�C�[���̒l
	bool processHitTest{};      // �}�E�X�̏������s��ꂽ
	bool processHitTestShift{}; // �V�t�g�̏������s��ꂽ
	union SDL_Event event;
	uint8_t mouseButtonMask{};
};