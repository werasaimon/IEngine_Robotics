#ifndef OPENGLRENDER_H
#define OPENGLRENDER_H

#include "Engine/engine.hpp"


using namespace IEngine;

class OpenGLRender
{
    public:


    OpenGLRender() = default;
    OpenGLRender(const OpenGLRender&) = default;



    static void DrawComponentMeshLineee(const IMesh *ComponentMesh );

    static void DrawComponentMeshPoint(const IMesh *ComponentMesh );


    static void DrawComponentMeshLine(const IMesh *ComponentMesh , const Matrix4& transform = Matrix4::IDENTITY);


    static void DrawComponentMeshFill(const IMesh *ComponentMesh , const Matrix4& transform = Matrix4::IDENTITY);


    static void DrawComponentMeshAABB( const IMesh* ComponentMesh );


    static void DrawComponentAABB(const IAABBox3D &AxisAlignedBoundingBox , const Matrix4& transform = Matrix4::IDENTITY, bool isRotateCenter = false);

    static void DrawComponentOBB(const IAABBox3D &AxisAlignedBoundingBox , const Matrix4& transform = Matrix4::IDENTITY);


    static void DrawLine(const Vector3 &a , const Vector3 &b);

    static void DrawSphere( const Vector3& pos );


};

#endif // OPENGLRENDER_H
