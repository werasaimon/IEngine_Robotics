/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/


//#ifdef __ANDROID__
//#elif defined(WIN32) || defined(__linux__)
//#include <GL/glew.h>
//#endif


#include <QMouseEvent>
#include <math.h>

#include <QCoreApplication>
#include <QDir>

#include <QMenu>

#include "GLWidget.h"


GLWidget::GLWidget(QWidget *parent)
  : QOpenGLWidget(parent)
{
    //mScene = new SceneEngine;
    //mScene = new SceneEngineNozzle;
    //mScene = new SceneEngineNuzzleGimbal;
    mScene = new SceneEngineRobocar;
    //mScene = new SceneEngineTest;


	setAttribute(Qt::WA_TranslucentBackground);
	setAttribute(Qt::WA_DeleteOnClose);
	//setFixedSize(SCR_WIDTH, SCR_HEIGHT);

    camSpec.cameraPos = QVector3D(0.0f, 0.0f, 1.0f);
    camSpec.cameraFront = QVector3D(0.0f, 0.0f, -1.0f);
    camSpec.cameraUp = QVector3D(0.0f, 1.0f, 0.0f);
    camSpec.yaw = -90.0f;
    camSpec.pitch = 0.0f;
    camSpec.lastX = SCR_WIDTH / 2.0;
    camSpec.lastY = SCR_HEIGHT / 2.0;
    camSpec.fov = 45.0f;

    screenDimension.setX(SCR_WIDTH);
    screenDimension.setY(SCR_HEIGHT);

	checkFirstFrameTimeElapsed = false;
	finishedLoadingBoldChars = false;
}


GLWidget::~GLWidget()
{
    if(mScene)
    {
        delete mScene;
    }

    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    doneCurrent();
}



QMatrix4x4 & GLWidget::getOrthoProjectionMatrix()
{
    return orthoProjection;
}


void GLWidget::setOrthoProjectionMatrix()
{
    orthoProjection.setToIdentity();

#ifdef NO_FLIP_VIEW
    orthoProjection.ortho(0.0f, static_cast<float>(SCR_WIDTH), 0.0f, static_cast<float>(SCR_HEIGHT), -1.0f, 1.0f);
#endif // NO_FLIP_VIEW

#ifdef FLIP_VIEW
    orthoProjection.ortho(static_cast<float>(SCR_WIDTH), 0.0f, static_cast<float>(SCR_HEIGHT), 0.0f, -1.0f, 1.0f);
#endif // FLIP_VIEW
}

QMatrix4x4 & GLWidget::getPerspectiveProjectionMatrix()
{
    return perspectiveProjection;
}

void GLWidget::setPerspectiveProjectionMatrix()
{
    perspectiveProjection.setToIdentity();

#ifdef NO_FLIP_VIEW
    perspectiveProjection.perspective(camSpec.fov, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
#endif // NO_FLIP_VIEW

#ifdef FLIP_VIEW
    perspectiveProjection.perspective(camSpec.fov, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
    perspectiveProjection.scale(-1.0f, -1.0f, 1.0f);
#endif // FLIP_VIEW

}

QMatrix4x4 & GLWidget::getViewMatrix()
{
    return viewMatrix;
}

void GLWidget::setViewMatrix()
{
    viewMatrix.setToIdentity();
    viewMatrix.lookAt(camSpec.cameraPos, camSpec.cameraPos + camSpec.cameraFront, camSpec.cameraUp);
}


void GLWidget::initShaders()
{
    //Font shaders
    if (!programFont.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vshaderFont30.glsl"))
        close();
    if (!programFont.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fshaderFont30.glsl"))
        close();
    if (!programFont.link())
        close();
}

void GLWidget::paintGLHelperForFontRendering()
{
	QVector4D fontColor(1.0f, 1.0f, 0.0f, 1.0f);
	programFont.bind();
	modelMatrix.setToIdentity();
	programFont.setUniformValue("mvp_matrix", getOrthoProjectionMatrix() * modelMatrix);
	programFont.setUniformValue("textColor", fontColor);
	programFont.setUniformValue("text", 0);
	QString stringToDisplay = QString::fromUtf8("Qt Font Load App With Unicode Ex: 会 毁 了");
	fpObj->drawFontGeometry(&programFont,10.0f, 10.0f, stringToDisplay, 0.75f);
	programFont.release();
}


void GLWidget::initializeGL()
{


//#ifdef __ANDROID__
//#elif defined(WIN32) || defined(__linux__)

//    GLenum err = glewInit();
//    if(err != GLEW_OK)
//    {
//        printf("%s",glewGetErrorString(err));
//    }

//#endif

    initializeOpenGLFunctions();

    /**

    glClearColor(0, 0, 0, 0.05f);
    setOrthoProjectionMatrix();
    setPerspectiveProjectionMatrix();
    setViewMatrix();
    initShaders();
    frameTime.start();
    frameCount = 0;
    frameTimeForFontLoad.start();

	QOpenGLContext *current = context();
	doneCurrent();
	fpObj = new FontProvider();

	// the background thread's context is shared from current
	QOpenGLContext *shared = fpObj->context;
	shared->setFormat(current->format());
	shared->setShareContext(current);
	shared->create();

	// must move the shared context to the background thread
	shared->moveToThread(fpObj);

	// setup the background thread's surface
	// must be created here in the main thread
	QOffscreenSurface *surface = fpObj->surface;
	surface->setFormat(shared->format());
	surface->create();

	// worker signal
	connect(fpObj, SIGNAL(started()), fpObj, SLOT(initializeFontProvider()));
	connect(fpObj, SIGNAL(finished()), this, SLOT(OnFinishedChildThread()));

	// must move the thread to itself
	fpObj->moveToThread(fpObj);

	// the worker can finally start
	fpObj->start();

	glEnable(GL_DEPTH_TEST);

	**/

//    glClearColor(0, 0, 0, 1);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);

    mTimeStep = (1.f/60.f);

    // Use QBasicTimer because its faster than QTimer
    timer.start( 10 , this);


    /// initilisation scene
    if(mScene)
    {
       mScene->initialization();
    }

}


void GLWidget::resizeGL( int w , int h )
{
   mScene->resize( w , h );
}


void GLWidget::paintGL()
{
    mScene->render(mTimeStep);

//#ifdef NO_FLIP_VIEW
//    glViewport(0, 0, (GLsizei)SCR_WIDTH, (GLsizei)SCR_HEIGHT);
//#endif
//#ifdef FLIP_VIEW
//	glViewport(-SCR_WIDTH_OFFSET, -SCR_HEIGHT_OFFSET, (GLsizei)SCR_WIDTH, (GLsizei)SCR_HEIGHT);
//#endif
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//	if (!checkFirstFrameTimeElapsed)
//	{
//		int loadTime = frameTimeForFontLoad.elapsed();
//		checkFirstFrameTimeElapsed = true;
//		qDebug() << "Total load Time for First painGL call" << loadTime << "\n";
//	}

//    if(finishedLoadingBoldChars)
//        paintGLHelperForFontRendering();

//	++frameCount;
//	if (frameTime.elapsed() >= 1000)
//	{
//		fps = frameCount / ((double)frameTime.elapsed() / 1000.0f);
//		qDebug() << "FPS" << "  " << fps;
//	}
//	//update();
}


void GLWidget::timerEvent(QTimerEvent *e)
{
    mScene->update();
   // e->accept();
    update();
}


void GLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
   mScene->specialKeyboardDown( keyEvent->key() );
   mScene->keyboard( keyEvent->key() );
}

void GLWidget::keyReleaseEvent(QKeyEvent *keyEvent)
{
   mScene->specialKeyboardUp(  keyEvent->key() );
   keyEvent->accept();
}



void GLWidget::wheelEvent(QWheelEvent *event)
{
   mScene->mouseWheel(event->delta());
   event->accept();
}


void GLWidget::mouseMoveEvent(QMouseEvent *e)
{
  mScene->mouseMove( e->pos().x() ,
                     e->pos().y() , static_cast<int>(e->button()) );
}

void GLWidget::mousePressEvent(QMouseEvent *e)
{
  mScene->mousePress( e->pos().x() ,
                      e->pos().y() ,
                      static_cast<int>(e->button()) );


  /**
  if( e->button() == Qt::MouseButton::RightButton )
  {
      QMenu menu(this);
      menu.addSeparator();
      menu.addAction( "Move"   ,  this , SLOT(Move())   );
      menu.addAction( "Scale"  ,  this , SLOT(Scale())  );
      menu.addAction( "Rotate" ,  this , SLOT(Rotate()) );
      menu.addSection("----------------------------");
      menu.addAction( "World"  ,  this , SLOT(World())  );
      menu.addAction( "Local"  ,  this , SLOT(Local()) );
      menu.exec(e->globalPos());
  }
  else
  {
      mScene->mousePress( e->pos().x() ,
                          e->pos().y() ,
                          static_cast<int>(e->button()) );

  }
  /**/
}

void GLWidget::mouseReleaseEvent(QMouseEvent *e)
{
   mScene->mouseReleasePress( e->pos().x() ,
                              e->pos().y() ,
                              static_cast<int>(e->button()) );
}


void GLWidget::closeEvent(QCloseEvent *event)
{
   mScene->destroy();
   event->accept();
}

SceneMain *GLWidget::scene()
{
    return mScene;
}

void GLWidget::OnFinishedChildThread()
{
  finishedLoadingBoldChars = true;
}






