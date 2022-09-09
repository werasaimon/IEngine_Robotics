#include "fontprovider.h"
#include <QFileDialog>
#include <iostream>

FontProvider::FontProvider()
{
	context = new QOpenGLContext();
	surface = new QOffscreenSurface();
}

FontProvider::~FontProvider()
{
	std::map<FT_ULong, Character>::iterator i = fontCharactersBold.begin();
	while ((i != fontCharactersBold.end()))
		fontCharactersBold.erase(i++);

	delete context;
	delete surface;
}

void FontProvider::initFontGeometry(const char* filepathname) {

	// FreeType
	FT_Library ft;
	if (FT_Init_FreeType(&ft))
		qDebug() << "ERROR::FREETYPE: Could not init FreeType Library\n";

	/*-------------------------------------------------------------------*/
	QStringList list1 = QString(filepathname).split("/");
	QString DirName;
	for(int i =0; i < list1.size()-1; ++i )
	{
		DirName += list1[i] + "/";
	}
	QString FontName = DirName + list1[list1.size()-1];

	if (QFile::exists(FontName))
	{
		QFile::remove(FontName);
		std::cout << "DELETE : " << FontName.toStdString().c_str() << std::endl;
	}
	else
	{
		QDir _dir;
		// if not, create it
		if( !_dir.exists(DirName) )
		{
			_dir.mkpath(DirName);
		}
	}

	if (QFile::copy(QString(":/") + FontName, FontName))
	{
		std::cout << "YES COPY FILE : " << FontName.toStdString().c_str() << std::endl;
	}
	/*---------------------------------------------------------*/

	FT_Face face;
	if (FT_New_Face(ft, filepathname, 0, &face))
	{
		qDebug() << "ERROR::FREETYPE: Failed to load font\n";
	}
	else
	{
		FT_Select_Charmap(face, FT_ENCODING_UNICODE);
		// Set size to load glyphs as
		FT_Set_Pixel_Sizes(face, 0, 48);

		// Disable byte-alignment restriction
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		FT_UInt index;
		FT_ULong  c = FT_Get_First_Char(face, &index);

		// Load all glyphs in ttf file
		while (index) {
			// Load character glyph
			if (FT_Load_Glyph(face, index, FT_LOAD_RENDER))
			{
				qDebug() << "ERROR::FREETYTPE: Failed to load Glyph\n";
				continue;
			}
			// Generate texture
			QOpenGLTexture* text = new QOpenGLTexture(QOpenGLTexture::Target::Target2D);
			text->setMinificationFilter(QOpenGLTexture::Nearest);
			text->setMagnificationFilter(QOpenGLTexture::Linear);
			text->setWrapMode(QOpenGLTexture::Repeat);
			text->create();
			text->setSize(face->glyph->bitmap.width, face->glyph->bitmap.rows, 1);
#ifdef PC_SETTINGS
			text->setFormat(QOpenGLTexture::RGBA8_UNorm);
#endif // PC_SETTINGS
#ifdef BOARD_SETTINGS
			text->setFormat(QOpenGLTexture::AlphaFormat);
#endif // BOARD_SETTINGS
			//    text->allocateStorage();
			text->allocateStorage(QOpenGLTexture::Alpha, QOpenGLTexture::UInt8);
			text->setData(QOpenGLTexture::Alpha, QOpenGLTexture::UInt8, face->glyph->bitmap.buffer);
			Character character = {
				text,
				QVector2D(face->glyph->bitmap.width, face->glyph->bitmap.rows),
				QVector2D(face->glyph->bitmap_left, face->glyph->bitmap_top),
				static_cast<GLuint>(face->glyph->advance.x)
			};
				fontCharactersBold.insert(std::pair<FT_ULong, Character>(c, character));
            text->release();
			c = FT_Get_Next_Char(face, c, &index);
		}
		FT_Done_Face(face);
		FT_Done_FreeType(ft);
	}

	emit finished();
}

void FontProvider::initializeFontProvider()
{
	//context->makeCurrent(surface);
	initializeOpenGLFunctions();
	initFontGeometry("resources/fonts/ReadboldV2000.ttf");
}

void FontProvider::drawFontGeometry(QOpenGLShaderProgram * programFont, GLfloat pos_x, GLfloat pos_y, const QString & text, GLfloat scale_factor)
{
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	int nCount = text.count();
	for (int i = 0; i < nCount; i++)
	{
		QChar cha = text.at(i);
		Character ch;
		ushort uni = cha.unicode();
        ch = fontCharactersBold[uni];
	
		if (ch.TextureID != NULL )
		{
			GLfloat xpos = pos_x + ch.Bearing.x() * scale_factor;
			GLfloat ypos = pos_y - (ch.Size.y() - ch.Bearing.y()) * scale_factor;
	
			GLfloat w = ch.Size.x() * scale_factor;
			GLfloat h = ch.Size.y() * scale_factor;
			GLfloat vertices[6][4] = {
				{ xpos,     ypos + h,   0.0, 0.0 },
				{ xpos,     ypos,       0.0, 1.0 },
				{ xpos + w, ypos,       1.0, 1.0 },
	
				{ xpos,     ypos + h,   0.0, 0.0 },
				{ xpos + w, ypos,       1.0, 1.0 },
				{ xpos + w, ypos + h,   1.0, 0.0 }
			};
			ch.TextureID->bind();
			QOpenGLBuffer* arrayBufFont = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
			arrayBufFont->create();
			arrayBufFont->bind();
			arrayBufFont->allocate(vertices, 4 * sizeof(vertices));
			int vertexLocation = programFont->attributeLocation("a_position");
			programFont->enableAttributeArray(vertexLocation);
			programFont->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 2, 4 * sizeof(GLfloat));
			int texcoordLocation = programFont->attributeLocation("a_texcoord");
			programFont->enableAttributeArray(texcoordLocation);
			programFont->setAttributeBuffer(texcoordLocation, GL_FLOAT, 2 * sizeof(GLfloat), 2, 4 * sizeof(GLfloat));
			glDrawArrays(GL_TRIANGLES, 0, 6);
			arrayBufFont->release();
			arrayBufFont->destroy();
			ch.TextureID->release();
			pos_x += (ch.Advance >> 6) * scale_factor; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
		}
	}
	
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}
