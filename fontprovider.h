#ifndef FONTPROVIDER_H
#define FONTPROVIDER_H

#define PC_SETTINGS
//#define BOARD_SETTINGS

#include <QObject>
#include <QOpenGLFunctions>
#include <QVector2D>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <ft2build.h>
#include <QThread>
#include <QOffscreenSurface>
#include FT_FREETYPE_H


struct Character {
	QOpenGLTexture*  TextureID;
	QVector2D		Size;    // Size of glyph
	QVector2D		Bearing;  // Offset from baseline to left/top of glyph
	GLuint			Advance;    // Horizontal offset to advance to next glyph
};

class FontProvider : public QThread, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit FontProvider();
	~FontProvider();

signals:
    void finished();

public slots:
void initializeFontProvider();

private:

public:
	QOpenGLContext*					context = nullptr;
	QOffscreenSurface*				surface = nullptr;
    void initFontGeometry();
	std::map<FT_ULong, Character>	fontCharactersBold;
    void drawFontGeometry(QOpenGLShaderProgram *programFont, GLfloat pos_x, GLfloat pos_y, const QString& text, GLfloat scale_factor);
};

#endif // FONTPROVIDER_H
