//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <vector>
#include <string>
#include <array>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};

Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }

static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
};
constexpr int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

int main(int /*argc*/, char** /*argv*/)
{
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL.\nError: %s\n", SDL_GetError());
		return -1;
	}

	// Enable depth buffer.
	// 深度バッファを有効にします。
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

	// Set color channel depth.
	// カラーチャネルの深さを設定します。
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

	// 4x MSAA.
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_DisplayMode displayMode;
	SDL_GetCurrentDisplayMode(0, &displayMode);

	bool presentationMode = false;
	Uint32 flags = SDL_WINDOW_OPENGL;
	int width;
	int height;

	if (presentationMode)
	{
		// Create a fullscreen window at the native resolution.
		// ネイティブ解像度でフルスクリーンウィンドウを作成します。
		width = displayMode.w;
		height = displayMode.h;
		flags |= SDL_WINDOW_FULLSCREEN;
	}
	else
	{
		float aspect = 16.0f / 9.0f;
		width = rcMin(displayMode.w, (int)(displayMode.h * aspect)) - 80;
		height = displayMode.h - 80;
	}

	SDL_Window* window;
	SDL_Renderer* renderer;
	int errorCode = SDL_CreateWindowAndRenderer(width, height, flags, &window, &renderer);

	if (errorCode != 0 || !window || !renderer)
	{
		printf("Could not initialise SDL opengl\nError: %s\n", SDL_GetError());
		return -1;
	}

	SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
	SDL_GL_CreateContext(window);

	// ImGuiの初期化
	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		printf("Could not init GUI renderer.\n");
		SDL_Quit();
		return -1;
	}

	float t = 0.f;
	float timeAcc = 0.f;
	Uint32 prevFrameTime = SDL_GetTicks();
	int mousePos[2] = { 0, 0 };
	int origMousePos[2] = { 0, 0 }; // Used to compute mouse movement totals across frames. //フレーム全体のマウスの動きの合計を計算するために使用されます。

	float cameraEulers[] = { 45, -45 };
	float cameraPos[] = { 0, 0, 0 };
	float camr = 1000;
	float origCameraEulers[] = { 0, 0 }; // Used to compute rotational changes across frames. //フレーム全体の回転変化を計算するために使用されます。

	float moveFront = 0.f, moveBack = 0.f, moveLeft = 0.f, moveRight = 0.f, moveUp = 0.f, moveDown = 0.f;

	float scrollZoom = 0;
	bool rotate = false;
	bool movedDuringRotate = false; //
	float ray_start[3];
	float ray_end[3];
	bool mouseOverMenu = false; // ImGui上のウィンドウにマウスが存在する

	bool showMenu = !presentationMode;
	bool showLog = false;
	bool showTools = true;
	bool showLevels = false;
	bool showSample = false;
	bool showTestCases = false;

	// Window scroll positions.
	// ウィンドウのスクロール位置。
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;

	string sampleName = "Choose Sample...";

	vector<string> files;
	const string meshesFolder = "Meshes";
	string meshName = "Choose Mesh...";

	float markerPosition[3] = { 0, 0, 0 };
	bool markerPositionSet = false;

	/// 流石に見える部分で生ポインタで扱いたくなかった
	std::unique_ptr<InputGeom> geom;
	std::unique_ptr<Sample> sample;

	const string testCasesFolder = "TestCases";
	std::unique_ptr<TestCase> test;

	BuildContext ctx;

	// Fog.
	float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);

	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);

	bool done = false;
	while (!done)
	{
		/// Handle input events.
		/// 入力イベントを処理します。
		int mouseScroll = 0;              // マウスホイールの値
		bool processHitTest = false;      // マウスの処理が行われた
		bool processHitTestShift = false; // シフトの処理が行われた
		SDL_Event event{};

		// インプット系のアップデート
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
				// キーを押した瞬間
				case SDL_KEYDOWN:
				{
					// Handle any key presses here.
					// ここでキーの押下を処理します。
					switch (event.key.keysym.sym)
					{
						// 終了
						case SDLK_ESCAPE:
						{
							done = true;
							break;
						}
						// テストを行う
						case SDLK_t:
						{
							showLevels = false;
							showSample = false;
							showTestCases = true;
							scanDirectory(testCasesFolder, ".txt", files); // ファイルのスキャンして、一覧をfilesに保存

							break;
						}
						// ImGuiの表示非表示
						case SDLK_TAB:
						{
							showMenu = !showMenu;
							break;
						}
						// パスの追跡処理を行う
						case SDLK_SPACE:
						{
							if (sample) sample->handleToggle();

							break;
						}
						// 群衆AI時で１フレーム進む
						case SDLK_1:
						{
							if (sample) sample->handleStep();

							break;
						}
						// ナビメッシュ設定のファイル書き出し
						case SDLK_9:
						{
							if (sample && geom)
							{
								string savePath = meshesFolder + "/";

								BuildSettings settings{}; // 保存用構造体

								// コピー
								rcVcopy(settings.navMeshBMin, geom->getNavMeshBoundsMin());
								rcVcopy(settings.navMeshBMax, geom->getNavMeshBoundsMax());

								// 保存したい設定の収集
								sample->collectSettings(settings);

								// 保存を実行
								geom->saveGeomSet(&settings);
							}
							break;
						}
					}
					break;
				}
				// マウスホイール
				case SDL_MOUSEWHEEL:
				{
					if (event.wheel.y < 0)
					{
						// wheel down // ホイールダウン

						// ImGuiウィンドウ上で操作
						if (mouseOverMenu)
						{
							mouseScroll++;
						}
						// 通常
						else
						{
							scrollZoom += 1.f;
						}
					}
					else
					{
						// ImGuiウィンドウ上で操作
						if (mouseOverMenu)
						{
							mouseScroll--;
						}
						// 通常
						else
						{
							scrollZoom -= 1.f;
						}
					}

					break;
				}
				// マウスのボタンを押した瞬間
				case SDL_MOUSEBUTTONDOWN:
				{
					// 右ボタンでImGuiウィンドウ外
					if (event.button.button == SDL_BUTTON_RIGHT && !mouseOverMenu)
					{
						// Rotate view // ビューを回転
						rotate = true;
						movedDuringRotate = false;
						origMousePos[0] = mousePos[0];
						origMousePos[1] = mousePos[1];
						origCameraEulers[0] = cameraEulers[0];
						origCameraEulers[1] = cameraEulers[1];
					}

					break;
				}
				// マウスのボタンから離した瞬間
				case SDL_MOUSEBUTTONUP:
				{
					// Handle mouse clicks here.
					//　ここでマウスクリックを処理します。
					switch (event.button.button)
					{
						case SDL_BUTTON_RIGHT:
						{
							rotate = false;

							// ImGuiウィンドウ外でマウスを動かしていない
							if (!(mouseOverMenu || movedDuringRotate))
							{
								processHitTest = true;
								processHitTestShift = true;
							}

							break;
						}
						case SDL_BUTTON_LEFT:
						{
							// ImGuiウィンドウ外
							if (!mouseOverMenu)
							{
								processHitTest = true;
								processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
							}

							break;
						}
					}

					break;
				}
				// マウスを動かしている
				case SDL_MOUSEMOTION:
				{
					mousePos[0] = event.motion.x;
					mousePos[1] = height - 1 - event.motion.y;

					if (rotate)
					{
						int dx = mousePos[0] - origMousePos[0];
						int dy = mousePos[1] - origMousePos[1];

						cameraEulers[0] = origCameraEulers[0] - dy * 0.25f;
						cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;

						if (dx * dx + dy * dy > 3 * 3)
						{
							movedDuringRotate = true;
						}
					}

					break;
				}
				// 終了
				case SDL_QUIT:
				{
					done = true;
					break;
				}
				default:
					break;
			}
		}

		// マウス
		unsigned char mouseButtonMask{};

		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
			mouseButtonMask |= IMGUI_MBUT_LEFT;

		if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
			mouseButtonMask |= IMGUI_MBUT_RIGHT;

		Uint32 time = SDL_GetTicks();
		float dt = (time - prevFrameTime) / 1000.f;

		prevFrameTime = time;
		t += dt;

		// Hit test mesh.
		// テストメッシュをヒットします。

		// マウスの左ボタン・右ボタンを離した瞬間、メッシュデータが存在し、サンプルも選択している状態
		if (processHitTest && geom && sample)
		{
			float hit_dis{};

			// マウスの指すレイと読み込まれたメッシュとの当たり判定
			// ※ このエディタの場合はスタート地点やゴール地点をメッシュデータ上にマウスで配置するので、経路探索をこの中に持ってこれる
			if (geom->raycastMesh(ray_start, ray_end, hit_dis))
			{
				std::array<float, 3u> ray_vec{};

				// レイのベクトルを求める
				for (size_t i = 0; i < 3u; i++)
				{
					ray_vec[i] = ray_end[i] - ray_start[i];
				}

				// マーカー
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = true;

					// 実際の交点を計算
					markerPosition[0] = ray_start[0] + (ray_vec[0] * hit_dis);
					markerPosition[1] = ray_start[1] + (ray_vec[1] * hit_dis);
					markerPosition[2] = ray_start[2] + (ray_vec[2] * hit_dis);
				}
				// 通常時
				else
				{
					float pos[3];

					// 実際の交点を計算
					pos[0] = ray_start[0] + (ray_vec[0] * hit_dis);
					pos[1] = ray_start[1] + (ray_vec[1] * hit_dis);
					pos[2] = ray_start[2] + (ray_vec[2] * hit_dis);

					// 経路探索を行う
					sample->handleClick(ray_start, pos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = false;
				}
			}
		}

		// Update sample simulation.
		// サンプルシミュレーションを更新します。
		constexpr float SIM_RATE = 20;
		constexpr float DELTA_TIME = 1.f / SIM_RATE;
		int simIter{};

		timeAcc = rcClamp(timeAcc + dt, -1.f, 1.f);

		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;

			if (simIter < 5 && sample)
				sample->handleUpdate(DELTA_TIME);

			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		// すべてのCPUを占有しないように、フレームレートをクランプします。
		constexpr float MIN_FRAME_TIME = 1.f / 40.f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = static_cast<int>((MIN_FRAME_TIME - dt) * 1000.f);

			if (ms > 10) ms = 10;
			if (ms >= 0) SDL_Delay(ms);
		}

		// Set the viewport. // ビューポートを設定します。
		glViewport(0, 0, width, height);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);

		// Clear the screen // 画面をクリアします
		glClearColor(0.3f, 0.3f, 0.32f, 1.f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);

		// Compute the projection matrix.
		// 投影行列を計算します。
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.f, (float)width / (float)height, 1.f, camr);
		GLdouble projectionMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);

		// Compute the modelview matrix.
		// モデルビュー行列を計算します。
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(cameraEulers[0], 1, 0, 0);
		glRotatef(cameraEulers[1], 0, 1, 0);
		glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);

		// Get hit ray position and direction.
		// マウスのレイの開始地点を計算
		GLdouble x, y, z;
		gluUnProject(mousePos[0], mousePos[1], 0.f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		ray_start[0] = (float)x;
		ray_start[1] = (float)y;
		ray_start[2] = (float)z;

		// マウスのレイの終了地点を計算
		gluUnProject(mousePos[0], mousePos[1], 1.f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		ray_end[0] = (float)x;
		ray_end[1] = (float)y;
		ray_end[2] = (float)z;

		// Handle keyboard movement.
		// キーボードの動きを処理します。
		const Uint8* keystate = SDL_GetKeyboardState(nullptr);

		moveFront = rcClamp(moveFront + dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP]) ? 1 : -1), 0.f, 1.f);
		moveLeft = rcClamp(moveLeft + dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT]) ? 1 : -1), 0.f, 1.f);
		moveBack = rcClamp(moveBack + dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN]) ? 1 : -1), 0.f, 1.f);
		moveRight = rcClamp(moveRight + dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT]) ? 1 : -1), 0.f, 1.f);
		moveUp = rcClamp(moveUp + dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP]) ? 1 : -1), 0.f, 1.f);
		moveDown = rcClamp(moveDown + dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN]) ? 1 : -1), 0.f, 1.f);

		float keybSpeed = 22.0f;

		if (SDL_GetModState() & KMOD_SHIFT)
		{
			keybSpeed *= 4.0f;
		}

		float movex = (moveRight - moveLeft) * keybSpeed * dt;
		float movey = (moveBack - moveFront) * keybSpeed * dt + scrollZoom * 2.0f;
		scrollZoom = 0;

		cameraPos[0] += movex * (float)modelviewMatrix[0];
		cameraPos[1] += movex * (float)modelviewMatrix[4];
		cameraPos[2] += movex * (float)modelviewMatrix[8];

		cameraPos[0] += movey * (float)modelviewMatrix[2];
		cameraPos[1] += movey * (float)modelviewMatrix[6];
		cameraPos[2] += movey * (float)modelviewMatrix[10];

		cameraPos[1] += (moveUp - moveDown) * keybSpeed * dt;

		glEnable(GL_FOG);

		// メッシュと四角の描画
		if (sample) sample->handleRender();

		if (test) test->handleRender();

		glDisable(GL_FOG);

		// Render GUI // GUI描画
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		mouseOverMenu = false;

		imguiBeginFrame(mousePos[0], mousePos[1], mouseButtonMask, mouseScroll);

		if (sample)
			sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);

		if (test && test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
			mouseOverMenu = true;

		// Help text. // ヘルプテキスト
		if (showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, height - 20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255, 255, 255, 128));
		}

		// 右のウィンドウ画面（Properties）
		if (showMenu)
		{
			if (imguiBeginScrollArea("Properties", width - 250 - 10, 10, 250, height - 20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("Show Log", showLog))
				showLog = !showLog;

			if (imguiCheck("Show Tools", showTools))
				showTools = !showTools;

			imguiSeparator();
			imguiLabel("Sample");

			// 「Choose Sample...」部分
			if (imguiButton(sampleName.c_str()))
			{
				if (showSample)
				{
					showSample = false;
				}
				// sampleが選ばれた
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
				}
			}

			imguiSeparator();
			imguiLabel("Input Mesh");

			// 「Choose Mesh...」部分
			if (imguiButton(meshName.c_str()))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				// Input Meshが選ばれた
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;

					// 「.obj」・「.gset」をfilesに追加
					scanDirectory(meshesFolder, ".obj", files);
					scanDirectoryAppend(meshesFolder, ".gset", files);
				}
			}

			// 頂点数、ポリゴン数の表示
			if (geom)
			{
				std::array<char, 64u> text{};

				snprintf(text.data(), text.size(), "Verts: %.1fk  Tris: %.1fk",
					geom->getMesh()->getVertCount() / 1000.f,
					geom->getMesh()->getTriCount() / 1000.f);

				imguiValue(text.data());
			}

			imguiSeparator(); // 区切り

			if (geom && sample)
			{
				imguiSeparatorLine();

				// ナビメッシュ設定の表示
				sample->handleSettings();

				if (imguiButton("Build"))
				{
					ctx.resetLog();

					// ナビメッシュの生成
					if (!sample->handleBuild())
					{
						showLog = true;
						logScroll = 0;
					}

					ctx.dumpLog("Build log %s:", meshName.c_str());

					// Clear test.
					test = nullptr;
				}

				imguiSeparator();
			}

			if (sample)
			{
				imguiSeparatorLine();

				// Draw欄の表示
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}

		// Sample selection dialog.
		// サンプルが選ばれている
		if (showSample)
		{
			static int levelScroll;

			if (imguiBeginScrollArea("Choose Sample", width - 10 - 250 - 10 - 200, height - 10 - 250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			Sample* newSample{};

			// Imgui上での表示と選択
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name.c_str()))
				{
					// 新しいサンプルが選ばれた
					newSample = g_samples[i].create();
					if (newSample)
						sampleName = g_samples[i].name;
				}
			}

			// 新しいサンプルが選ばれている
			if (newSample)
			{
				sample.reset(newSample);
				sample->setContext(&ctx);

				if (geom) sample->handleMeshChanged(geom.get());

				showSample = false;
			}

			if (geom || sample)
			{
				const float* bmin{};
				const float* bmax{};

				if (geom)
				{
					bmin = geom->getNavMeshBoundsMin();
					bmax = geom->getNavMeshBoundsMax();
				}

				// Reset camera and fog to match the mesh bounds.
				// カメラとフォグをリセットして、メッシュの境界に一致させます。
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
						rcSqr(bmax[1] - bmin[1]) +
						rcSqr(bmax[2] - bmin[2])) / 2;
					cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
					cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
					cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}

				cameraEulers[0] = 45;
				cameraEulers[1] = -45;
				glFogf(GL_FOG_START, camr * 0.1f);
				glFogf(GL_FOG_END, camr * 1.25f);
			}

			imguiEndScrollArea();
		}

		// Level selection dialog. // レベル選択ダイアログ。
		// Chose Meshを選択している
		if (showLevels)
		{
			static int levelScroll;
			if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
				mouseOverMenu = true;

			auto filesEnd = files.end();
			auto levelToLoad = filesEnd;

			// Imgui上での選択肢を表示
			for (auto fileIter = files.begin(); fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					levelToLoad = fileIter;
				}
			}

			// ジオメトリが選択されたとき
			if (levelToLoad != filesEnd)
			{
				meshName = *levelToLoad;
				showLevels = false;

				// 既に読み込まれているジオメトリを削除
				geom = nullptr;

				// 読み込むパスを決定
				string path = meshesFolder + "/" + meshName;

				// ジオメトリを構築し
				geom = std::make_unique<InputGeom>();

				// 読み込む
				if (!geom->load(&ctx, path))
				{
					geom = nullptr;

					// Destroy the sample if it already had geometry loaded, as we've just deleted it!
					// 削除したばかりのジオメトリが既にロードされている場合、サンプルを破壊します！
					if (sample && sample->getInputGeom())
						sample = nullptr;

					showLog = true;
					logScroll = 0;

					ctx.dumpLog("Geom load log %s:", meshName.c_str());
				}

				if (sample && geom)
				{
					sample->handleMeshChanged(geom.get());
				}

				if (geom || sample)
				{
					const float* bmin{};
					const float* bmax{};

					if (geom)
					{
						bmin = geom->getNavMeshBoundsMin();
						bmax = geom->getNavMeshBoundsMax();
					}

					// Reset camera and fog to match the mesh bounds.
					// メッシュの境界に一致するようにカメラとフォグをリセットします。
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
							rcSqr(bmax[1] - bmin[1]) +
							rcSqr(bmax[2] - bmin[2])) / 2;
						cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
						cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
						cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}

					cameraEulers[0] = 45;
					cameraEulers[1] = -45;

					glFogf(GL_FOG_START, camr * 0.1f);
					glFogf(GL_FOG_END, camr * 1.25f);
				}
			}

			imguiEndScrollArea();
		}

		// Test cases // テスト画面
		if (showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &testScroll))
				mouseOverMenu = true;

			auto filesEnd = files.end();
			auto testToLoad = filesEnd;

			for (auto fileIter = files.begin(); fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					testToLoad = fileIter;
				}
			}

			if (testToLoad != filesEnd)
			{
				string path = testCasesFolder + "/" + *testToLoad;
				test = std::make_unique<TestCase>();

				if (test)
				{
					// Load the test.
					if (!test->load(path)) test = nullptr;

					// Create sample
					Sample* newSample{};

					for (int i = 0; i < g_nsamples; ++i)
					{
						if (g_samples[i].name == test->getSampleName())
						{
							newSample = g_samples[i].create();
							if (newSample)
								sampleName = g_samples[i].name;
						}
					}

					sample.reset(newSample);

					if (sample)
					{
						sample->setContext(&ctx);
						showSample = false;
					}

					// Load geom.
					meshName = test->getGeomFileName();

					path = meshesFolder + "/" + meshName;

					geom = std::make_unique<InputGeom>();

					if (!geom || !geom->load(&ctx, path))
					{
						geom = nullptr;
						sample = nullptr;
						showLog = true;
						logScroll = 0;
						ctx.dumpLog("Geom load log %s:", meshName.c_str());
					}

					if (sample && geom) { sample->handleMeshChanged(geom.get()); }

					// This will ensure that tile & poly bits are updated in tiled sample.
					// これにより、タイルサンプルでタイルビットとポリビットが更新されます。
					if (sample) sample->handleSettings();

					ctx.resetLog();

					if (sample && !sample->handleBuild())
					{
						ctx.dumpLog("Build log %s:", meshName.c_str());
					}

					if (geom || sample)
					{
						const float* bmin{};
						const float* bmax{};

						if (geom)
						{
							bmin = geom->getNavMeshBoundsMin();
							bmax = geom->getNavMeshBoundsMax();
						}

						// Reset camera and fog to match the mesh bounds.
						// メッシュの境界に一致するようにカメラとフォグをリセットします。
						if (bmin && bmax)
						{
							camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
									rcSqr(bmax[1] - bmin[1]) +
									rcSqr(bmax[2] - bmin[2])) / 2;

							cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
							cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
							cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
							camr *= 3;
						}

						cameraEulers[0] = 45;
						cameraEulers[1] = -45;
						glFogf(GL_FOG_START, camr * 0.2f);
						glFogf(GL_FOG_END, camr * 1.25f);
					}

					// Do the tests. // テストを行います。
					if (sample) test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
				}
			}

			imguiEndScrollArea();
		}

		// Log // ログ
		if (showLog && showMenu)
		{
			if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
				mouseOverMenu = true;

			for (int i = 0; i < ctx.getLogCount(); ++i)
				imguiLabel(ctx.getLogText(i));

			imguiEndScrollArea();
		}

		// Left column tools menu // 左列ツールメニュー
		// 左のウィンドウ画面（Tools）
		if (!showTestCases && showTools && showMenu) // && geom && sample)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
				mouseOverMenu = true;

			// Tool欄設定
			if (sample) sample->handleTools();

			imguiEndScrollArea();
		}

		// Marker // マーカー
		// gluProject（オブジェクト座標系をウィンドウ座標系に変換）
		// http://ja.manpages.org/gluproject/3
		if (markerPositionSet &&
			gluProject((GLdouble)markerPosition[0], (GLdouble)markerPosition[1], (GLdouble)markerPosition[2],
				modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
		{
			// Draw marker circle // マーカー円を描きます
			glLineWidth(5.0f);
			glColor4ub(240, 220, 0, 196);
			glBegin(GL_LINE_LOOP);

			constexpr float Radius = 20.f;  // 半径
			constexpr float Accuracy = 15.f; // 精度

			for (int i = 0; i < Accuracy; ++i)
			{
				const float a = (float)i / Accuracy * RC_PI * 2;
				const float fx = (float)x + cosf(a) * Radius;
				const float fy = (float)y + sinf(a) * Radius;

				glVertex2f(fx, fy); // 2D上に描画
			}

			glEnd();
			glLineWidth(1.f);
		}

		imguiEndFrame();
		imguiRenderGLDraw();

		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapWindow(window);
	}

	// 終了処理
	imguiRenderGLDestroy();
	SDL_Quit();

	return 0;
}