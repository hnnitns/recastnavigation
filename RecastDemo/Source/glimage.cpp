#include "GLImage.h"
#include <SDL_Opengl.h>
#define STBI_HEADER_FILE_ONLY
//#include "stb_image.c"


GLImage::GLImage() :
	m_width(0),
	m_height(0),
	m_texId(0),
	m_type(IMAGE_L)
{
}

GLImage::~GLImage()
{
	purge();
}

void GLImage::purge()
{
	if (m_texId)
	{
		glDeleteTextures(1, (GLuint*)&m_texId);
		m_texId = 0;
	}
}

bool GLImage::create(const char* fileName, ImageWrap wrap)
{
//	purge();
//
//	int bpp;
//	unsigned char* data = stbi_load(fileName, &m_width, &m_height, &bpp, 0);
//	if (!data)
//	{
//		printf("Could not load file '%s': %s\n", fileName, stbi_failure_reason());
//		return false;
//	}
//
//	if (bpp == 1)
//		m_type = IMAGE_L;
//	else if (bpp == 2)
//		m_type = IMAGE_LA;
//	else if (bpp == 3)
//		m_type = IMAGE_RGB;
//	else if (bpp == 4)
//		m_type = IMAGE_RGBA;
//
//	GLint	format = (GLint)getGLFormat();
//
////	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//	glGenTextures(1, (GLuint*)&m_texId);
//	glBindTexture(GL_TEXTURE_2D, m_texId);
//	glTexImage2D(GL_TEXTURE_2D, 0, format, m_width, m_height, 0,
//		format, GL_UNSIGNED_BYTE, data);
////	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
//
//	if (wrap == IMAGE_WRAP_REPEAT)
//	{
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//	}
//	else
//	{
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//	}
//
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//
//	stbi_image_free(data);

	return true;
}

bool GLImage::create(int w, int h, ImageType type, const unsigned char* data, ImageWrap wrap)
{
	purge();

	m_width = w;
	m_height = h;
	m_type = type;
	GLint	format = (GLint)getGLFormat();

//	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, (GLuint*)&m_texId);
	glBindTexture(GL_TEXTURE_2D, m_texId);
	glTexImage2D(GL_TEXTURE_2D, 0, format, m_width, m_height, 0,
		format, GL_UNSIGNED_BYTE, data);
//	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	if (wrap == IMAGE_WRAP_REPEAT)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	}
	else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	return true;
}

void GLImage::updateData(const unsigned char* data)
{
	GLint	format = (GLint)getGLFormat();
	glBindTexture(GL_TEXTURE_2D, m_texId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_width, m_height, format, GL_UNSIGNED_BYTE, data);
}


int GLImage::getGLFormat()
{
	if (m_type == IMAGE_RGB)
		return GL_RGB;
	else if (m_type == IMAGE_RGBA)
		return GL_RGBA;
	else if (m_type == IMAGE_L)
		return GL_LUMINANCE;
	else if (m_type == IMAGE_LA)
		return GL_LUMINANCE_ALPHA;
	return GL_RGB;
}

void GLImage::bind(int unit)
{
//	glActiveTexture(GL_TEXTURE0 + unit);
	glBindTexture(GL_TEXTURE_2D, m_texId);
//	glActiveTexture(GL_TEXTURE0);
}

