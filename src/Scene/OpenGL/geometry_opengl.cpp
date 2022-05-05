#include "geometry_opengl.h"

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

#include "geometry_opengl.h"


#include <QVector2D>
#include <QVector3D>
#include <QOpenGLFunctions>


//! [0]
geometry_opengl::geometry_opengl()
    : indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    // Generate 2 VBOs
    arrayBuf.create();
    arrayBufUV.create();
    arrayBufNormal.create();
    arrayBufTanget.create();
    indexBuf.create();
}



geometry_opengl::geometry_opengl(const IMesh *i_mesh)
    :indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    // Generate 2 VBOs
    arrayBuf.create();
    arrayBufUV.create();
    arrayBufNormal.create();
    arrayBufTanget.create();
    indexBuf.create();



    arrayBuf.bind();
    arrayBuf.allocate(&i_mesh->VerticesBuffer().front() , i_mesh->VertexCount() * sizeof(Vector3));


    arrayBufUV.bind();
    arrayBufUV.allocate(&i_mesh->UVBuffer().front() , i_mesh->VertexCount() * sizeof(Vector2));


    arrayBufNormal.bind();
    arrayBufNormal.allocate( &i_mesh->NormalsBuffer().front() , i_mesh->VertexCount() * sizeof(Vector3));


    arrayBufTanget.bind();
    arrayBufTanget.allocate( &i_mesh->TangentsBuffer().front() , i_mesh->VertexCount() * sizeof(Vector3));


    indexBuf.bind();
    indexBuf.allocate(&i_mesh->TrianglesBuffer().front(), (mSizeVertex = i_mesh->TriangleCount() * 3) * sizeof(uint));


}

geometry_opengl::~geometry_opengl()
{
    arrayBuf.destroy();
    arrayBufUV.destroy();
    arrayBufNormal.destroy();
    arrayBufTanget.destroy();
    indexBuf.destroy();
}



//! [2]
void geometry_opengl::drawCubeGeometry(QOpenGLShaderProgram *program)
{

    arrayBuf.bind();
     // Tell OpenGL programmable pipeline how to locate vertex position data
     int vertexLocation = program->attributeLocation("a_position");
     program->enableAttributeArray(vertexLocation);
     program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(Vector3));


     arrayBufUV.bind();
     // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
     int texcoordLocation = program->attributeLocation("a_textcoord");
     program->enableAttributeArray(texcoordLocation);
     program->setAttributeBuffer(texcoordLocation, GL_FLOAT, 0, 2, sizeof(Vector2));



     arrayBufNormal.bind();
     // Tell OpenGL programmable normal coordinate data
     int normalLocation = program->attributeLocation("a_normal");
     program->enableAttributeArray(normalLocation);
     program->setAttributeBuffer(normalLocation, GL_FLOAT, 0 , 3, sizeof(Vector3));



     arrayBufTanget.bind();
     // Tell OpenGL programmable normal coordinate data
     int tangetLocation = program->attributeLocation("a_tangent");
     program->enableAttributeArray(tangetLocation);
     program->setAttributeBuffer(tangetLocation, GL_FLOAT, 0 , 3, sizeof(Vector3));



     indexBuf.bind();
     // Draw cube geometry using indices from VBO 1
     glDrawElements(GL_TRIANGLES, mSizeVertex, GL_UNSIGNED_INT, 0);
}
//! [2]


