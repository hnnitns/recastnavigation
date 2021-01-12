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

#include "SlideShow.h"
#include <string.h>
#include <stdio.h>
#include <SDL_opengl.h>
#define STBI_HEADER_FILE_ONLY
//#include "stb_image.c"

template<class T> inline T min(T a, T b) { return a < b ? a : b; }

SlideShow::SlideShow() :
	m_showCurSlide(true),
	m_slideAlpha(0.0f),
	m_masterAlpha(0.0f),
	m_curSlide(-1),
	m_nextSlide(0)
{
}

SlideShow::~SlideShow()
{
	purgeImage(m_curImg);
	purgeImage(m_nextImg);
}

void SlideShow::purgeImage(Img& img)
{
	if (img.texId)
		glDeleteTextures(1, (GLuint*)&img.texId);
	img.reset();
}

bool SlideShow::loadImage(Img& img, const char* path)
{
	//purgeImage(img);

	//int bpp;
	//unsigned char* data = stbi_load(path, &img.width, &img.height, &bpp, 4);
	//if (!data)
	//{
	//	printf("Could not load file '%s': %s\n", path, stbi_failure_reason());
	//	return false;
	//}

	//glGenTextures(1, (GLuint*)&img.texId);
	//glBindTexture(GL_TEXTURE_RECTANGLE_ARB, img.texId);
	//glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, img.width, img.height, 0,
	//			 GL_RGBA, GL_UNSIGNED_BYTE, data);

	//glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	//glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	//glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	//stbi_image_free(data);

	return true;
}

bool SlideShow::init(const char* path)
{
	strcpy(m_path, path);
	//scanDirectory(m_path, ".jpg", m_files);

	return true;
}

void SlideShow::nextSlide()
{
	setSlide(m_nextSlide+1);
}

void SlideShow::prevSlide()
{
	setSlide(m_nextSlide-1);
}

void SlideShow::setSlide(int n)
{
	const int maxIdx = m_files.size ? m_files.size-1 : 0;
	m_nextSlide = n;
	if (m_nextSlide < 0) m_nextSlide = 0;
	if (m_nextSlide > maxIdx) m_nextSlide = maxIdx;
}

void SlideShow::updateAndDraw(float dt, const float w, const float h, bool show)
{
	const float TRANSITION_TIME = 0.25f;

	if (show)
	{
		m_masterAlpha += dt/TRANSITION_TIME;
		if (m_masterAlpha > 1.0f)
			m_masterAlpha = 1.0f;
	}
	else
	{
		m_masterAlpha -= dt/TRANSITION_TIME;
		if (m_masterAlpha < 0.0f)
			m_masterAlpha = 0.0f;
	}


	if (m_curSlide != m_nextSlide)
	{
		if (m_nextImg.texId == 0)
		{
			if (m_nextSlide >= 0 && m_nextSlide < m_files.size)
			{
				char path[256];
				strcpy(path, m_path);
				strcat(path, m_files.files[m_nextSlide]);
				loadImage(m_nextImg, path);
			}
		}
		m_slideAlpha += dt / TRANSITION_TIME;
		if (m_slideAlpha > 1.0f)
		{
			m_slideAlpha = 0.0f;
			purgeImage(m_curImg);
			m_curImg = m_nextImg;
			m_nextImg.reset();
			m_curSlide = m_nextSlide;
		}
	}

	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	if (m_curImg.texId)
	{
		unsigned char alpha = (unsigned char)(m_masterAlpha*255.0f);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_curImg.texId);

		const float tw = (float)m_curImg.width;
		const float th = (float)m_curImg.height;
		const float hw = w*0.5f;
		const float hh = h*0.5f;

		glColor4ub(255,255,255,alpha);
		glBegin(GL_QUADS);
		glTexCoord2f(0,th);
		glVertex2f(hw-tw/2,hh-th/2);
		glTexCoord2f(tw,th);
		glVertex2f(hw+tw/2,hh-th/2);
		glTexCoord2f(tw,0);
		glVertex2f(hw+tw/2,hh+th/2);
		glTexCoord2f(0,0);
		glVertex2f(hw-tw/2,hh+th/2);
		glEnd();
	}

	if (m_nextImg.texId)
	{
		unsigned char alpha = (unsigned char)(min(m_masterAlpha, m_slideAlpha)*255.0f);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_nextImg.texId);

		const float tw = (float)m_nextImg.width;
		const float th = (float)m_nextImg.height;
		const float hw = w*0.5f;
		const float hh = h*0.5f;

		glColor4ub(255,255,255,alpha);
		glBegin(GL_QUADS);
		glTexCoord2f(0,th);
		glVertex2f(hw-tw/2,hh-th/2);
		glTexCoord2f(tw,th);
		glVertex2f(hw+tw/2,hh-th/2);
		glTexCoord2f(tw,0);
		glVertex2f(hw+tw/2,hh+th/2);
		glTexCoord2f(0,0);
		glVertex2f(hw-tw/2,hh+th/2);
		glEnd();
	}

	glDisable(GL_TEXTURE_RECTANGLE_ARB);



/*	float slideAlphaTarget = (m_showCurSlide && m_texId) ? 1.0f : 0.0f;
	if (m_curSlide != m_nextSlide)
		slideAlphaTarget = 0;

	if (slideAlphaTarget > m_slideAlpha)
		m_slideAlpha += dt*4;
	else if (slideAlphaTarget < m_slideAlpha)
		m_slideAlpha -= dt*4;
	if (m_slideAlpha < 0) m_slideAlpha = 0;
	if (m_slideAlpha > 1) m_slideAlpha = 1;

	if (m_curSlide != m_nextSlide && m_slideAlpha < 0.01f)
	{
		m_curSlide = m_nextSlide;
		if (m_curSlide >= 0 && m_curSlide < m_files.size)
		{
			char path[256];
			strcpy(path, m_path);
			strcat(path, m_files.files[m_curSlide]);
			loadImage(path);
		}
	}

	if (m_slideAlpha > 0.01f && m_texId)
	{
		unsigned char alpha = (unsigned char)(m_slideAlpha*255.0f);

		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_texId);

		const float tw = (float)m_width;
		const float th = (float)m_height;
		const float hw = w*0.5f;
		const float hh = h*0.5f;

		glColor4ub(255,255,255,alpha);
		glBegin(GL_QUADS);
		glTexCoord2f(0,th);
		glVertex2f(hw-tw/2,hh-th/2);
		glTexCoord2f(tw,th);
		glVertex2f(hw+tw/2,hh-th/2);
		glTexCoord2f(tw,0);
		glVertex2f(hw+tw/2,hh+th/2);
		glTexCoord2f(0,0);
		glVertex2f(hw-tw/2,hh+th/2);
		glEnd();

		glDisable(GL_TEXTURE_RECTANGLE_ARB);
	}*/

}
