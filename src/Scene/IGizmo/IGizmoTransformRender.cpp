
#include "IMathGizmo.h"
#include "IGizmoTransformRender.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

namespace IuGizmo
{






//namespace IuGizmo
//{

void CGizmoTransformRender::DrawCircle(const IVector3 &orig,float r,float g,float b,const IVector3 &vtx,const IVector3 &vty)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4f(r,g,b,1);

    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 50 ; i++)
    {
        IVector3 vt;
        vt = vtx * cos((2*M_PI/50)*i);
        vt += vty * sin((2*M_PI/50)*i);
        vt += orig;
        glVertex3f(vt.x,vt.y,vt.z);
    }
    glEnd();
}


void CGizmoTransformRender::DrawCircleHalf(const IVector3 &orig,float r,float g,float b,const IVector3 &vtx,const IVector3 &vty,tplane &camPlan)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4f(r,g,b,1);

    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < 30 ; i++)
    {
        IVector3 vt;
        vt = vtx * cos((M_PI/30)*i);
        vt += vty * sin((M_PI/30)*i);
        vt +=orig;

        IVector3 camPlan3(camPlan.x,camPlan.y,camPlan.z);

        if (camPlan3.Dot(vt-orig) > 0.01 )
            glVertex3f(vt.x,vt.y,vt.z);
    }
    glEnd();
}

void CGizmoTransformRender::DrawAxis(const IVector3 &orig, const IVector3 &axis, const IVector3 &vtx,const IVector3 &vty, float fct,float fct2,const IVector4 &col)
{


    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor4fv(&col.x);
    glBegin(GL_LINES);
    glVertex3fv(&orig.x);
    glVertex3f(orig.x+axis.x,orig.y+axis.y,orig.z+axis.z);
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    for (int i=0;i<=30;i++)
    {
        IVector3 pt;
        pt = vtx * cos(((2*M_PI)/30.0f)*i)*fct;
        pt+= vty * sin(((2*M_PI)/30.0f)*i)*fct;
        pt+=axis*fct2;
        pt+=orig;
        glVertex3fv(&pt.x);
        pt = vtx * cos(((2*M_PI)/30.0f)*(i+1))*fct;
        pt+= vty * sin(((2*M_PI)/30.0f)*(i+1))*fct;
        pt+=axis*fct2;
        pt+=orig;
        glVertex3fv(&pt.x);
        glVertex3f(orig.x+axis.x,orig.y+axis.y,orig.z+axis.z);

    }
    glEnd();

//    glBegin(GL_POINTS);
//     glVertex3f(testPoint.x,testPoint.y,testPoint.z);
//    glEnd();

}

void CGizmoTransformRender::DrawCamem(const IVector3& orig,const IVector3& vtx,const IVector3& vty,float ng)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    int i = 0 ;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);


    glColor4f(1,1,0,0.5f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3fv(&orig.x);
    for (i = 0 ; i <= 50 ; i++)
    {
        IVector3 vt;
        vt = vtx * cos(((ng)/50)*i);
        vt += vty * sin(((ng)/50)*i);
        vt+=orig;
        glVertex3f(vt.x,vt.y,vt.z);
    }
    glEnd();

    glDisable(GL_BLEND);


    glColor4f(1,1,0.2f,1);
    IVector3 vt;
    glBegin(GL_LINE_LOOP);

    glVertex3fv(&orig.x);
    for ( i = 0 ; i <= 50 ; i++)
    {
        IVector3 vt;
        vt = vtx * cos(((ng)/50)*i);
        vt += vty * sin(((ng)/50)*i);
        vt+=orig;
        glVertex3f(vt.x,vt.y,vt.z);
    }

    glEnd();
}

void CGizmoTransformRender::DrawQuad(const IVector3& orig, float size, bool bSelected, const IVector3& axisU, const IVector3 &axisV)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);

    IVector3 pts[4];
    pts[0] = orig;
    pts[1] = orig + (axisU * size);
    pts[2] = orig + (axisU + axisV)*size;
    pts[3] = orig + (axisV * size);

    if (!bSelected)
        glColor4f(1,1,0,0.5f);
    else
        glColor4f(1,1,1,0.6f);

    glBegin(GL_QUADS);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glVertex3fv(&pts[3].x);
    glEnd();

    if (!bSelected)
        glColor4f(1,1,0.2f,1);
    else
        glColor4f(1,1,1,0.6f);

    glBegin(GL_LINE_STRIP);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glVertex3fv(&pts[3].x);
    glEnd();

    glDisable(GL_BLEND);
}


void CGizmoTransformRender::DrawTri(const IVector3& orig, float size, bool bSelected, const IVector3& axisU, const IVector3& axisV)
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);

    IVector3 pts[3];
    pts[0] = orig;

    pts[1] = (axisU );
    pts[2] = (axisV );

    pts[1]*=size;
    pts[2]*=size;
    pts[1]+=orig;
    pts[2]+=orig;

    if (!bSelected)
        glColor4f(1,1,0,0.5f);
    else
        glColor4f(1,1,1,0.6f);

    glBegin(GL_TRIANGLES);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glVertex3fv(&pts[3].x);
    glEnd();

    if (!bSelected)
        glColor4f(1,1,0.2f,1);
    else
        glColor4f(1,1,1,0.6f);

    glBegin(GL_LINE_STRIP);
    glVertex3fv(&pts[0].x);
    glVertex3fv(&pts[1].x);
    glVertex3fv(&pts[2].x);
    glEnd();

    glDisable(GL_BLEND);
}

//}


}

#include "IGizmoTransform.h"

