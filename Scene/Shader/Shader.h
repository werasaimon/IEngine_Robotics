#ifndef SHADER_H_
#define SHADER_H_


#ifdef __ANDROID__
#elif defined(WIN32) || defined(__linux__)
//#include <GL/glew.h>
#endif


#include <assert.h>
#include <iostream>
#include <string>


#include "Engine/engine.hpp"


#include <QOpenGLShaderProgram>


using namespace IEngine;


class GLShaderProgram : public QOpenGLShaderProgram
{

  public:

      GLShaderProgram(QObject *parent = Q_NULLPTR):
      QOpenGLShaderProgram(parent)
      {

      }


      bool addSourceFile( const int type , const char* fileName )
      {
         return addShaderFromSourceFile( QOpenGLShader::ShaderTypeBit(type) , fileName );
      }



      void UniformValue(const char *name , int value);
      void UniformValue(const char *name, const float& value);
      void UniformValue(const char *name, const Vector2& value);
      void UniformValue(const char *name, const Vector3& value);
      void UniformValue(const char *name, const Vector4& value);
      void UniformValue(const char *name, const Matrix3& value);
      void UniformValue(const char *name, const Matrix4& value);




      enum ShaderType
      {
          Vertex                 = 0x0001,
          Fragment               = 0x0002,
          Geometry               = 0x0004,
          TessellationControl    = 0x0008,
          TessellationEvaluation = 0x0010,
          Compute                = 0x0020
      };

};



#endif /* SHADER_H_ */
