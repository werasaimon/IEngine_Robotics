#include "OpenGLRender.h"

#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/freeglut.h>

namespace
{

    static void DrawCube()
    {
        glBegin(GL_LINE_LOOP);
        // top
        glColor3f(1.0f, 0.0f, 0.0f);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // front
        glColor3f(0.0f, 1.0f, 0.0f);
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // right
        glColor3f(0.0f, 0.0f, 1.0f);
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // left
        glColor3f(0.0f, 0.0f, 0.5f);
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // bottom
        glColor3f(0.5f, 0.0f, 0.0f);
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // back
        glColor3f(0.0f, 0.5f, 0.0f);
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);

        glEnd();
    }

}


void OpenGLRender::DrawComponentMeshLineee(const IMesh *ComponentMesh )
{
    //    glPushMatrix();
    //    glMultMatrixf(ComponentMesh->GetTransformMatrix());
    glLineWidth(3);
    for (Index4i t : ComponentMesh->Edges())
    {
        glBegin(GL_LINES);
        glColor3f(1,1,1);
        glVertex3fv(ComponentMesh->GetVertex(t[0]));
        glVertex3fv(ComponentMesh->GetVertex(t[1]));
        glEnd();
    }
    glLineWidth(1);
    //    glPopMatrix();
}

void OpenGLRender::DrawComponentMeshPoint(const IMesh *ComponentMesh )
{
//    glPushMatrix();
//    glMultMatrixf(ComponentMesh->GetTransformMatrix());
    glLineWidth(3);
    for (Vector3 t : ComponentMesh->Vertices())
    {
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        glVertex3fv(t);
        glEnd();
    }
    glLineWidth(1);
//    glPopMatrix();
}


void OpenGLRender::DrawComponentMeshLine(const IMesh *ComponentMesh, const Matrix4 &transform)
{
    glPushMatrix();
    glMultMatrixf(transform);
    glLineWidth(3);
    for (Index4i t : ComponentMesh->Edges())
    {
        glBegin(GL_LINES);
        glColor3f(1,1,1);
        glVertex3fv(ComponentMesh->GetVertex(t[0]));
        glVertex3fv(ComponentMesh->GetVertex(t[1]));
        glEnd();
    }
    glLineWidth(1);
    glPopMatrix();

}

void OpenGLRender::DrawComponentMeshFill(const IMesh *ComponentMesh, const Matrix4 &transform)
{
    glPushMatrix();
    glMultMatrixf(transform);
    glLineWidth(3);
    for (Index3i t : ComponentMesh->Triangles())
    {
        glBegin(GL_TRIANGLES);
        glVertex3fv(ComponentMesh->GetVertex(t[0]));
        glVertex3fv(ComponentMesh->GetVertex(t[1]));
        glVertex3fv(ComponentMesh->GetVertex(t[2]));
        glEnd();
    }
    glLineWidth(1);
    glPopMatrix();
}

void OpenGLRender::DrawComponentMeshAABB(const IMesh *ComponentMesh)
{
    Vector3 HalfSize =  ComponentMesh->GetAABBTransform().HalfSize();
    Vector3 center   =  ComponentMesh->GetAABBTransform().Center();
    glPushMatrix();
    glTranslatef(center.x,center.y,center.z);
    glScalef(HalfSize.x,HalfSize.y,HalfSize.z);
    DrawCube();
    glPopMatrix();
}

void OpenGLRender::DrawComponentAABB(const IAABBox3D &AxisAlignedBoundingBox, const Matrix4 &transform , bool isRotateCenter)
{
    Vector3 HalfSize =  AxisAlignedBoundingBox.GetAxisAlignedBoxTransform(transform).HalfSize();
    Vector3 center   =  AxisAlignedBoundingBox.GetAxisAlignedBoxTransform(transform).Center();

   // if(isRotateCenter == true) HalfSize = transform.GetRotMatrix().GetInverse().GetTranspose() * HalfSize;

    glPushMatrix();
    glTranslatef(center.x,center.y,center.z);
    glScalef(HalfSize.x,HalfSize.y,HalfSize.z);
    DrawCube();
    glPopMatrix();
}

void OpenGLRender::DrawComponentOBB(const IAABBox3D &AxisAlignedBoundingBox, const Matrix4 &transform)
{
    Vector3 HalfSize = AxisAlignedBoundingBox.GetAxisAlignedBoxTransform().HalfSize();
    glPushMatrix();
    glMultMatrixf(transform);
    glScalef(HalfSize.x,HalfSize.y,HalfSize.z);
    DrawCube();
    glPopMatrix();
}

void OpenGLRender::DrawLine(const Vector3 &a, const Vector3 &b)
{
    glPushMatrix();
    glBegin(GL_LINES);
    glColor3f(1,1,1);
    glVertex3fv(a);
    glVertex3fv(b);
    glEnd();
    glPopMatrix();
}

void OpenGLRender::DrawSphere(const Vector3 &pos)
{
    glPushMatrix();
    glTranslatef( pos.x , pos.y , pos.z );
  //  glutWireSphere( 0.2 , 10 , 10 );
    glPopMatrix();
}
