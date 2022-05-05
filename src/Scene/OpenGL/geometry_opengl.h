#ifndef GEOMETRY_OPENGL_H
#define GEOMETRY_OPENGL_H


#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "Engine/engine.hpp"


using namespace iengine;

class geometry_opengl : protected QOpenGLFunctions
{
public:
    geometry_opengl();
     geometry_opengl(const IMesh *i_mesh);
    virtual ~geometry_opengl();

    void drawCubeGeometry(QOpenGLShaderProgram *program);


private:

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer arrayBufUV;
    QOpenGLBuffer arrayBufNormal;
    QOpenGLBuffer arrayBufTanget;
    QOpenGLBuffer indexBuf;

    int mSizeVertex;

};



#endif // GEOMETRY_OPENGL_H
