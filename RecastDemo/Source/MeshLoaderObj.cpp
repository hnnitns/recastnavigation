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

#include "MeshLoaderObj.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <DirectXMath.h>
#include <ppl.h>
#include "Other/XMFLOAT_Hlper.h"
#include <amp.h>

namespace PPL = Concurrency;

#ifdef _DEBUG
#define   new	new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

rcMeshLoaderObj::rcMeshLoaderObj() :
	m_scale(1.f), m_vertCount{}, m_triCount{}
{}

void rcMeshLoaderObj::addVertex(float x, float y, float z, int& cap)
{
	if (m_vertCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;

		//float* nv = new float[cap * 3];
		std::vector<float> nv(cap * 3, 0.f);

		if (m_vertCount)
			//memcpy(nv, m_verts, m_vertCount * 3 * sizeof(float));
			for (int i = 0; i < m_vertCount * 3; i++)
			{
				nv[i] = m_verts[i];
			}

		//delete[] m_verts;

		m_verts = nv;
	}
	float* dst = &m_verts[m_vertCount * 3];
	*dst++ = x * m_scale;
	*dst++ = y * m_scale;
	*dst++ = z * m_scale;
	m_vertCount++;
}

void rcMeshLoaderObj::addTriangle(int a, int b, int c, int& cap)
{
	if (m_triCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;

		//int* nv = new int[cap * 3];
		std::vector<int> nv(cap * 3, 0);

		if (m_triCount)
			//memcpy(nv, m_tris, m_triCount * 3 * sizeof(int));
			for (int i = 0; i < m_triCount * 3; i++)
			{
				nv[i] = m_tris[i];
			}

		//delete[] m_tris;
		m_tris = nv;
	}

	int* dst = &m_tris[m_triCount * 3];
	*dst++ = a;
	*dst++ = b;
	*dst++ = c;
	m_triCount++;
}

namespace
{
	char* parseRow(char* buf, char* bufEnd, char* row, int len)
	{
		bool start = true;
		bool done = false;
		int n = 0;
		while (!done && buf < bufEnd)
		{
			char c = *buf;
			buf++;
			// multirow
			switch (c)
			{
				case '\\':
					break;
				case '\n':
					if (start) break;
					done = true;
					break;
				case '\r':
					break;
				case '\t':
				case ' ':
					if (start) break;
				default:
					start = false;
					row[n++] = c;
					if (n >= len - 1)
						done = true;
					break;
			}
		}
		row[n] = '\0';
		return buf;
	}

	int parseFace(char* row, int* data, int n, int vcnt)
	{
		int j{};

		while (*row != '\0')
		{
			// Skip initial white space
			// 最初の空白をスキップします
			while (*row != '\0' && (*row == ' ' || *row == '\t'))
				row++;

			char* s = row;

			// Find vertex delimiter and terminated the string there for conversion.
			// 頂点区切り文字を見つけ、変換のためにそこで文字列を終了しました。
			while (*row != '\0' && *row != ' ' && *row != '\t')
			{
				if (*row == '/') *row = '\0';
				row++;
			}

			if (*s == '\0')
				continue;

			int vi = atoi(s);
			data[j++] = vi < 0 ? vi + vcnt : vi - 1;

			if (j >= n) return j;
		}
		return j;
	}
}

bool rcMeshLoaderObj::load(const std::string& filename, const float defalut_load_scale)
{
	char* buf{ nullptr };
	FILE* fp{ nullptr };

	m_scale = defalut_load_scale;

	// 開けない
	if (fopen_s(&fp, filename.c_str(), "rb") != 0) return false;

	// nullの可能性を排除
	if (!fp)	return false;

	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}

	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return false;
	}

	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}

	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}

	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);

	if (readLen != 1)
	{
		delete[] buf;
		return false;
	}

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512]{};
	int face[32]{};
	float x{}, y{}, z{};
	int nv{};
	int vcap{};
	int tcap{};

	while (src < srcEnd)
	{
		// Parse one row
		// 1行を解析します
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));

		// Skip comments
		// コメントをスキップ
		if (row[0] == '#') continue;

		// 頂点の追加
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			[[maybe_unused]]int temp{ sscanf_s(row + 1, "%f %f %f", &x, &y, &z) };
			addVertex(x, y, z, vcap);
		}

		// 三角形の追加
		if (row[0] == 'f')
		{
			// 法線
			nv = parseFace(row + 1, face, 32, m_vertCount);

			for (int i = 2; i < nv; ++i)
			{
				const int a = face[0];
				const int b = face[i - 1];
				const int c = face[i];

				if ((a < 0 || a >= m_vertCount) || (b < 0 || b >= m_vertCount) || (c < 0 || c >= m_vertCount))
					continue;

				addTriangle(a, b, c, tcap);
			}
		}
	}

	delete[] buf;

	// Calculate normals.
	// 法線を計算します。
	m_normals.resize(m_triCount * 3);

	for (int i = 0; i < m_triCount * 3; i += 3)
	{
		const float* v0 = &m_verts[m_tris[i] * 3];
		const float* v1 = &m_verts[m_tris[i + 1] * 3];
		const float* v2 = &m_verts[m_tris[i + 2] * 3];
		float e0[3], e1[3];

		for (int j = 0; j < 3; ++j)
		{
			e0[j] = v1[j] - v0[j];
			e1[j] = v2[j] - v0[j];
		}

		float* n = &m_normals[i];

		n[0] = e0[1] * e1[2] - e0[2] * e1[1];
		n[1] = e0[2] * e1[0] - e0[0] * e1[2];
		n[2] = e0[0] * e1[1] - e0[1] * e1[0];

		float d = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

		if (d > 0)
		{
			d = 1.f / d;
			n[0] *= d;
			n[1] *= d;
			n[2] *= d;
		}
	}

	m_filename = filename;
	m_origine_verts = m_verts; // 原点を保存

	return true;
}

class FLOAT4X4 : public DirectX::XMFLOAT4X4
{
public:
	FLOAT4X4() : DirectX::XMFLOAT4X4() {};
	~FLOAT4X4() {};

	FLOAT4X4(const float _11, const float _12, const float _13, const float _14,
		const float _21, const float _22, const float _23, const float _24,
		const float _31, const float _32, const float _33, const float _34,
		const float _41, const float _42, const float _43, const float _44)
	{
		this->_11 = _11; this->_12 = _12; this->_13 = _13; this->_14 = _14;
		this->_21 = _21; this->_22 = _22; this->_23 = _23; this->_24 = _24;
		this->_31 = _31; this->_32 = _32; this->_33 = _33; this->_34 = _34;
		this->_41 = _41; this->_42 = _42; this->_43 = _43; this->_44 = _44;
	}

	FLOAT4X4(const FLOAT4X4& f4x4)
	{
		_11 = f4x4._11; _12 = f4x4._12; _13 = f4x4._13; _14 = f4x4._14;
		_21 = f4x4._21; _22 = f4x4._22; _23 = f4x4._23; _24 = f4x4._24;
		_31 = f4x4._31; _32 = f4x4._32; _33 = f4x4._33; _34 = f4x4._34;
		_41 = f4x4._41; _42 = f4x4._42; _43 = f4x4._43; _44 = f4x4._44;
	}

	FLOAT4X4 operator*(const FLOAT4X4& m) const {
		float x = _11;
		float y = _12;
		float z = _13;
		float w = _14;

		FLOAT4X4 result;
		result._11 = (m._11 * x) + (m._21 * y) + (m._31 * z) + (m._41 * w);
		result._12 = (m._12 * x) + (m._22 * y) + (m._32 * z) + (m._42 * w);
		result._13 = (m._13 * x) + (m._23 * y) + (m._33 * z) + (m._43 * w);
		result._14 = (m._14 * x) + (m._24 * y) + (m._34 * z) + (m._44 * w);

		x = _21;
		y = _22;
		z = _23;
		w = _24;
		result._21 = (m._11 * x) + (m._21 * y) + (m._31 * z) + (m._41 * w);
		result._22 = (m._12 * x) + (m._22 * y) + (m._32 * z) + (m._42 * w);
		result._23 = (m._13 * x) + (m._23 * y) + (m._33 * z) + (m._43 * w);
		result._24 = (m._14 * x) + (m._24 * y) + (m._34 * z) + (m._44 * w);

		x = _31;
		y = _32;
		z = _33;
		w = _34;
		result._31 = (m._11 * x) + (m._21 * y) + (m._31 * z) + (m._41 * w);
		result._32 = (m._12 * x) + (m._22 * y) + (m._32 * z) + (m._42 * w);
		result._33 = (m._13 * x) + (m._23 * y) + (m._33 * z) + (m._43 * w);
		result._34 = (m._14 * x) + (m._24 * y) + (m._34 * z) + (m._44 * w);

		x = _41;
		y = _42;
		z = _43;
		w = _44;
		result._41 = (m._11 * x) + (m._21 * y) + (m._31 * z) + (m._41 * w);
		result._42 = (m._12 * x) + (m._22 * y) + (m._32 * z) + (m._42 * w);
		result._43 = (m._13 * x) + (m._23 * y) + (m._33 * z) + (m._43 * w);
		result._44 = (m._14 * x) + (m._24 * y) + (m._34 * z) + (m._44 * w);

		return result;
	}
};

// Concurrency::parallel_for_each関数内のみ使えるクラス
template<typename _Ty, int _Size>
struct GpuOnlyEasyArray
{
	using value_type = _Ty;
	using size_type = int;
	using difference_type = ptrdiff_t;
	using pointer = _Ty*;
	using const_pointer = const _Ty*;
	using reference = _Ty&;
	using const_reference = const _Ty&;

public:
	void fill(const _Ty& _Value) __GPU
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		for (int i = 0; i < _Size; i++)
		{
			_Elems[i] = _Value;
		}
	}

	void swap(GpuOnlyEasyArray<_Ty, _Size>& _Other) __GPU
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		pointer base_ptr{ _Elems[0] }, oth_ptr{ _Other[0] }, temp{ base_ptr };

		base_ptr = oth_ptr;
		oth_ptr = temp;
	}

	_NODISCARD _CONSTEXPR17 reference at(size_type _Pos) __GPU
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		if (_Size <= _Pos)
		{
			throw std::out_of_range;
		}

		return _Elems[_Pos];
	}

	_NODISCARD constexpr const_reference at(size_type _Pos) const __GPU
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		if (_Size <= _Pos)
		{
			throw std::out_of_range;
		}

		return _Elems[_Pos];
	}

	_NODISCARD _CONSTEXPR17 reference operator[](size_type _Pos) __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[_Pos];
	}

	_NODISCARD constexpr const_reference operator[](size_type _Pos) const __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[_Pos];
	}

	_NODISCARD _CONSTEXPR17 reference front() __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[0];
	}

	_NODISCARD constexpr const_reference front() const __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[0];
	}

	_NODISCARD _CONSTEXPR17 reference back() __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[_Size - 1];
	}

	_NODISCARD constexpr const_reference back() const __GPU noexcept /* strengthened */
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems[_Size - 1];
	}

	_NODISCARD _CONSTEXPR17 _Ty* data() __GPU noexcept
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems;
	}

	_NODISCARD _CONSTEXPR17 const _Ty* data() const __GPU noexcept
	{
		static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

		return _Elems;
	}

public:
	_Ty _Elems[_Size];
};

#if true
template<typename _Ty>
auto operator*(const GpuOnlyEasyArray<_Ty, 16>& left, const GpuOnlyEasyArray<_Ty, 16>& right) __GPU
{
	static_assert(std::is_floating_point_v<_Ty> || std::is_integral_v<_Ty>, "It is a type that can not be used on GPU");

	GpuOnlyEasyArray<_Ty, 16> rv{};

	for (int i = 0; i < 4u; i++)
	{
		const int numx{ 0 * (i * 4) }, numy{ 1 * (i * 4) }, numz{ 2 * (i * 4) }, numw{ 3 * (i * 4) };

		const float x = left[numx];
		const float y = left[numy];
		const float z = left[numz];
		const float w = left[numw];

		rv[numx] = (right[numx] * x) + (right[0 + 4] * y) + (right[0 + 8] * z) + (right[0 + 12] * w);
		rv[numy] = (right[numy] * x) + (right[1 + 4] * y) + (right[1 + 8] * z) + (right[1 + 12] * w);
		rv[numz] = (right[numz] * x) + (right[2 + 4] * y) + (right[2 + 8] * z) + (right[2 + 12] * w);
		rv[numw] = (right[numw] * x) + (right[3 + 4] * y) + (right[3 + 8] * z) + (right[3 + 12] * w);
	}

	return rv;
}
#endif

std::vector<PPL::accelerator> findAccelerators() {
	std::vector<PPL::accelerator> accels;
	accels = PPL::accelerator::get_all();

	//emulatorのアクセラレータを削除します
	accels.erase(std::remove_if(accels.begin(), accels.end(), [](PPL::accelerator& accel) {return accel.get_is_emulated(); }), accels.end());

	return accels;
}
void rcMeshLoaderObj::MoveVerts(
	const std::array<float, 3>& pos, const std::array<float, 3>& rotate, const std::array<float, 3>& scale)
{
	using namespace DirectX;
	using Math::ToRadian;

	const XMMATRIX S = XMMatrixScaling(scale[0], scale[1], scale[2]);
	const XMMATRIX R = XMMatrixRotationRollPitchYaw(ToRadian(rotate[0]), ToRadian(rotate[1]), ToRadian(rotate[2]));
	const XMMATRIX T = XMMatrixTranslation(pos[0], pos[1], pos[2]);

	// ワールド変換行列
	XMMATRIX W = S * R * T;
	XMFLOAT4X4 w;
	DirectX::XMStoreFloat4x4(&w, W);

#if false // GPU
	std::array<float, 16> w_arr{
	w._11, w._12, w._13, w._14,
	w._21, w._22, w._23, w._24,
	w._31, w._32, w._33, w._34,
	w._41, w._42, w._43, w._44 };

	PPL::array_view<float, 1> arr_view_origin(m_verts.size(), m_verts.data()),
		origin_verts(m_origine_verts.size(), m_origine_verts.data()),
		world_mat(w_arr.size(), w_arr.data());

	PPL::parallel_for_each(arr_view_origin.get_extent(), [=](PPL::index<1> idx) restrict(amp)
		{
			GpuOnlyEasyArray<float, 16> pos
			{
				1.f, 0.f, 0.f, 0.f,
				1.f, 0.f, 0.f, 0.f,
				1.f, 0.f, 0.f, 0.f,
				origin_verts[idx + 0], origin_verts[idx + 1], origin_verts[idx + 2], 1.f
			};
			GpuOnlyEasyArray<float, 16> world;

			for (int i = 0; i < 16; i++)
			{
				world[i] = world_mat[i];
			}

			const auto&& result{ pos * world };

			arr_view_origin[idx + 0] = result[12];
			arr_view_origin[idx + 1] = result[13];
			arr_view_origin[idx + 2] = result[14];
		});

	//for (size_t i = 0; i < 4; i++)
	//{
	//	for (size_t j = 0; j < 4; j++)
	//	{
	//		to_gpu_arr[j * 4 + i] = mat[i].m128_f32[j];
	//	}
	//}
#elif false
	auto&& ac = findAccelerators();

	std::array<float, 16> a;

	a.fill(16.f);

	PPL::array_view<float, 1> av{ int(a.size()), reinterpret_cast<float*>(&a[0]) };

	PPL::parallel_for_each(av.get_extent(), [=](PPL::index<1> idx) restrict(amp)
		{
			av[idx] = 10;
		});

#elif true // CPU
	PPL::parallel_for(0, m_vertCount * 3, 3, [&](const int i)
		{
			const auto Pos = XMMatrixTranslation(m_origine_verts[i + 0], m_origine_verts[i + 1], m_origine_verts[i + 2]);

			const auto result{ Pos * W };
			auto& vs_pos{ result.r[3].m128_f32 };

			m_verts[i + 0] = vs_pos[0];
			m_verts[i + 1] = vs_pos[1];
			m_verts[i + 2] = vs_pos[2];
		});
#endif
}
