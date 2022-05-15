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

#ifndef MAINWIDGET_H
#define MAINWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QWheelEvent>

#include <QFile>
#include <QDebug>
#include <QTextStream>
#include <QUdpSocket>



#include "Scene/SceneEngine.h"
#include "Scene/SceneEngineNozzle.h"
#include "Scene/SceneEngineNuzzleGimbal.h"
#include "Scene/SceneEngineRobocar.h"
#include "Scene/SceneEngineTest.h"



class GLWidget : public QOpenGLWidget , protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLWidget(QWidget *parent = nullptr);
            ~GLWidget();

//protected:
public:

    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;


    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;

    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;

    void keyPressEvent( QKeyEvent *keyEvent ) Q_DECL_OVERRIDE;
    void keyReleaseEvent( QKeyEvent *keyEvent ) Q_DECL_OVERRIDE;

    void closeEvent(QCloseEvent *event);



    SceneMain *scene();


public slots:



    void Move()
    {
       assert(mScene);
       static_cast<SceneEngineNuzzleGimbal*>(mScene)->gizmoManipulator()->CheckMove();
    }

    void Scale()
    {
        assert(mScene);
        static_cast<SceneEngineNuzzleGimbal*>(mScene)->gizmoManipulator()->CheckScale();
    }

    void Rotate()
    {
        assert(mScene);
        static_cast<SceneEngineNuzzleGimbal*>(mScene)->gizmoManipulator()->CheckRotate();
    }

    void World()
    {
        assert(mScene);
        static_cast<SceneEngineNuzzleGimbal*>(mScene)->gizmoManipulator()->CheckWorld();
    }
    void Local()
    {
        assert(mScene);
        static_cast<SceneEngineNuzzleGimbal*>(mScene)->gizmoManipulator()->CheckLocal();
    }


private:

    QBasicTimer  timer;

    /// scene open GL
    SceneMain* mScene;

    /// Update FPS time
    float mTimeStep;


private slots:


};

#endif // MAINWIDGET_H
