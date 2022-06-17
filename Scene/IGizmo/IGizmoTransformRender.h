#ifndef IGIZMOTRANSFORMRENDER_H
#define IGIZMOTRANSFORMRENDER_H


namespace IuGizmo
{

    //namespace IuGizmo
    //{

//    class IVector3;
//    class IVector4;

    typedef IVector4 tplane;

    class CGizmoTransformRender
    {
    public:
        CGizmoTransformRender() {}
        virtual ~CGizmoTransformRender() {}

        static void DrawCircle(const IVector3 &orig,float r,float g,float b,const IVector3 &vtx,const IVector3 &vty);
        static void DrawCircleHalf(const IVector3 &orig,float r,float g,float b,const IVector3 &vtx,const IVector3 &vty,tplane &camPlan);
        static void DrawAxis(const IVector3 &orig, const IVector3 &axis, const IVector3 &vtx,const IVector3 &vty, float fct,float fct2,const IVector4 &col);
        static void DrawCamem(const IVector3& orig,const IVector3& vtx,const IVector3& vty,float ng);
        static void DrawQuad(const IVector3& orig, float size, bool bSelected, const IVector3& axisU, const IVector3 &axisV);
        static void DrawTri(const IVector3& orig, float size, bool bSelected, const IVector3& axisU, const IVector3& axisV);

    };

    //}


}


#endif // IGIZMOTRANSFORMRENDER_H
