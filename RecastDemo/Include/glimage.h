#ifndef GLIMAGE_H
#define GLIMAGE_H

enum ImageType
{
	IMAGE_RGB,
	IMAGE_RGBA,
	IMAGE_L,
	IMAGE_LA,
};

enum ImageWrap
{
	IMAGE_WRAP_REPEAT,
	IMAGE_WRAP_CLAMP,
};

class GLImage
{
public:
	GLImage();
	~GLImage();
	
	bool create(const char* fileName, ImageWrap wrap);
	bool create(int w, int h, ImageType type, const unsigned char* data, ImageWrap wrap);

	void updateData(const unsigned char* data);

	inline int getWidth() const { return m_width; }
	inline int getHeight() const { return m_height; }
	inline ImageType	getType() const { return m_type; }

	void bind(int unit = 0);
	
private:
	int	getGLFormat();
	void purge();

	int m_width;
	int m_height;
	ImageType	m_type;
	unsigned int m_texId;
};


#endif