

#include "IMathGizmo.h"
#include "IGizmoTransformScale.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

namespace IuGizmo
{

extern IVector3 ptd;


IGizmo *CreateScaleGizmo()
{
    return new CGizmoTransformScale;
}


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CGizmoTransformScale::CGizmoTransformScale() : CGizmoTransform()
{
    mScaleType = SCALE_NONE;
}

CGizmoTransformScale::~CGizmoTransformScale()
{

}



int CGizmoTransformScale::RayTracePlaneOrtoAxis(const IVector3 &RayOrigin, const IVector3 &RayDir, const IVector3 &Origin, const IVector3 &Axis_A, const IVector3 &Axis_B, IVector3 &inters)
{
    IVector3 axis_a = Axis_A;
    IVector3 axis_b = Axis_B;
    axis_a.Normalize();
    axis_b.Normalize();

    IVector3 vNormal = Cross(axis_a,axis_b);

    if( vNormal.LengthSquare() < 0.00001f) return 0;

    vNormal.Normalize();

    IPlane plane(vNormal,Origin);
    inters = plane.VIntersectionRayToPlane(RayOrigin,RayDir);

     IVector3 predictC = -Cross(vNormal,axis_b-axis_a);
     IVector3 predictA =  Cross(vNormal,axis_a);
     IVector3 predictB = -Cross(vNormal,axis_b);

     predictA.Normalize();
     predictB.Normalize();
     predictC.Normalize();


      float ScreenFactor = GetScreenFactor();

      IVector3 df = (inters - Origin);


     if( (df).Dot(predictA) / ScreenFactor >= 0 &&
         (df).Dot(predictB) / ScreenFactor >= 0 &&
         (df).Dot(predictC) / ScreenFactor <= 0.5 * (axis_a).Dot(predictC) )

         //        (df).dot(axis_a)/ScreenFactor <= (Lu * 0.5) &&
         //        (df).dot(axis_b)/ScreenFactor <= (Lv * 0.5)
         //        (df).dot(sumV)/ScreenFactor <= sumV.lengthSquare() * 0.25

     {

         if((df).Dot(predictC) / ScreenFactor >= 0.2 * (axis_a).Dot(predictC))
         {
             return 1;
         }
         else
         {
             return 4;
         }

     }
     else
     {

         /**
         IVector3 sumV = (axis_a+axis_b);

         float Lu = (sumV).dot(axis_a);
         float Lv = (sumV).dot(axis_b);

         float len_0 = fabs(df.dot(predictA));
         float len_1 = fabs(df.dot(predictB));

         if( len_0 < len_1 )
         {
             if( fabs(len_0/ScreenFactor) < (0.05) && (df.dot(axis_a)/ScreenFactor) < Lu )
             {
                 return 2;
             }
         }
         else
         {
             if( fabs(len_1/ScreenFactor) < (0.05) && (df.dot(axis_b)/ScreenFactor) < Lv )
             {
                 return 3;
             }

         }
         /**/


         float len_0 = 0;
         float len_1 = 0;

         if( IntersectLineToLine( Origin , Origin+Axis_A , RayOrigin , RayOrigin + RayDir * 1000 , len_0) )
         {
             if( (len_0 / ScreenFactor) < 0.05  )
             {
                 return 2;;
             }
         }

         if( IntersectLineToLine( Origin , Origin+Axis_B , RayOrigin , RayOrigin + RayDir * 1000 , len_1) )
         {
             if( (len_1 / ScreenFactor) < 0.05  && len_1 < len_0)
             {
                 return 3;
             }
         }
     }

    return 0;
}





bool CGizmoTransformScale::GetOpType(SCALETYPE &type, unsigned int x, unsigned int y)
{
    // init
    IVector3 trss(GetTransformedVector(0).Length(),
                  GetTransformedVector(1).Length(),
                  GetTransformedVector(2).Length());

    mLockX = x;
    mLockY = y;
    m_svgMatrix = *m_pMatrix;

    IMatrix4x4 mt;
    mt = *m_pMatrix;

    // ray casting
    IVector3 rayOrigin,rayDir,df2;
    BuildRay(x, y, rayOrigin, rayDir);


    IVector3 origin = m_svgMatrix.GetTranslation();

    IVector3 axisTrX = GetTransformedVector(0);
    IVector3 axisTrY = GetTransformedVector(1);
    IVector3 axisTrZ = GetTransformedVector(2);

    axisTrX *= GetScreenFactor();
    axisTrY *= GetScreenFactor();
    axisTrZ *= GetScreenFactor();

    IVector3 inters;
    float    minimum_distance_nearest = -1;
    int      tp = 0;

    type = SCALE_NONE;

    // plan 1 : X/Z
    if( (tp = RayTracePlaneOrtoAxis(rayOrigin,rayDir,origin,axisTrX,axisTrZ,inters)))
    {
        float len = IAbs(rayDir.Dot(rayOrigin - inters));

        if((len < minimum_distance_nearest || minimum_distance_nearest == -1))
        {
            minimum_distance_nearest = len;

            mPlan = IPlane( Cross(axisTrX,axisTrZ).Normalized() , origin );
            mLockVertex = inters;

            switch (tp)
            {
                case 1 : type = SCALE_XZ; break;
                case 2 : type = SCALE_X; break;
                case 3 : type = SCALE_Z; break;
                case 4 : type = SCALE_XYZ; break;
            }

        }

    }


     //plan 2 : X/Y
    if( (tp = RayTracePlaneOrtoAxis(rayOrigin,rayDir,origin,axisTrX,axisTrY,inters)))
    {
        float len = IAbs(rayDir.Dot(rayOrigin - inters));

        if((len < minimum_distance_nearest || minimum_distance_nearest == -1))
        {
            minimum_distance_nearest = len;

            mPlan = IPlane( Cross(axisTrX,axisTrY).Normalized() , origin);
            mLockVertex = inters;

            switch (tp)
            {
                case 1 : type = SCALE_XY; break;
                case 2 : type = SCALE_X; break;
                case 3 : type = SCALE_Y; break;
                case 4 : type = SCALE_XYZ; break;
            }
        }
    }


    //plan 3: Y/Z
    if( (tp = RayTracePlaneOrtoAxis(rayOrigin,rayDir,origin,axisTrY,axisTrZ,inters)))
    {
        float len = IAbs(rayDir.Dot(rayOrigin - inters));

        if((len < minimum_distance_nearest || minimum_distance_nearest == -1))
        {
            minimum_distance_nearest = len;

            mPlan = IPlane( Cross(axisTrY,axisTrZ).Normalized(), origin);
            mLockVertex = inters;

            switch (tp)
            {
                case 1 : type = SCALE_YZ; break;
                case 2 : type = SCALE_Y; break;
                case 3 : type = SCALE_Z; break;
                case 4 : type = SCALE_XYZ; break;
            }
        }

    }

    return (minimum_distance_nearest>0);

}


bool CGizmoTransformScale::OnMouseDown(unsigned int x, unsigned int y)
{
    mScaleType = SCALE_NONE;
    if (m_pMatrix)
    {
        return GetOpType(mScaleType, x, y);
    }

    mScaleType = SCALE_NONE;
    return false;

}

void CGizmoTransformScale::SnapScale(float &val)
{
    if (mbUseSnap)
    {
        val*=(100.0f);
        SnapIt(val,mScaleSnap);
        val/=(100.0f);
    }
}



void CGizmoTransformScale::OnMouseMove(unsigned int x, unsigned int y)
{

    if (mScaleType != SCALE_NONE)
    {
        IVector3 rayOrigin,rayDir,df, inters, machin;
        IVector3 scVect,scVect2;

        BuildRay(x, y, rayOrigin, rayDir);
        inters = mPlan.VIntersectionRayToPlane(rayOrigin,rayDir);



        IVector3 axeX(1,0,0);
        IVector3 axeY(0,1,0);
        IVector3 axeZ(0,0,1);


        if (mLocation == LOCATE_LOCAL)
        {
            IMatrix3x3 Rot = (m_svgMatrix).GetRotMatrix();
            Rot.OrthoNormalize();

            axeX = axeX * Rot;
            axeY = axeY * Rot;
            axeZ = axeZ * Rot;

            axeX.Normalize();
            axeY.Normalize();
            axeZ.Normalize();

        }


        df = inters-mLockVertex;
        df/=GetScreenFactor();


        IVector3 axeXZ = axeX + axeZ;
        IVector3 axeXY = axeX + axeZ;
        IVector3 axeYZ = axeX + axeZ;
        axeXZ.Normalize();
        axeXY.Normalize();
        axeYZ.Normalize();



        switch (mScaleType)
        {
            case SCALE_XZ:	df = IVector3(df.Dot(axeXZ) , 0,df.Dot(axeXZ));		break;
            case SCALE_X:	df = IVector3(df.Dot(axeX)  , 0,0);					break;
            case SCALE_Z:	df = IVector3(0, 0 , df.Dot(axeZ));		    		break;
            case SCALE_XY:	df = IVector3(df.Dot(axeXY) ,df.Dot(axeXY),0);		break;
            case SCALE_YZ:	df = IVector3(0,df.Dot(axeYZ) ,df.Dot(axeYZ));		break;
            case SCALE_Y:	df = IVector3(0,df.Dot(axeY), 0);					break;
            //case SCALE_XYZ: df = IVector3(df.length(),df.length(),df.length()); break;
        }


        IVector3 worldPoint = m_svgMatrix.GetTranslation();

        if( mScaleType != SCALE_XYZ )
        {

            switch ((int)mScaleType)
            {
                case SCALE_XZ:  IVector3::BiUnitOrthogonalVector(Cross(axeX,axeZ).Normalized() , axeX , axeZ);	 break;
                case SCALE_XY:	IVector3::BiUnitOrthogonalVector(Cross(axeX,axeY).Normalized() , axeX , axeY);   break;
                case SCALE_YZ:  IVector3::BiUnitOrthogonalVector(Cross(axeY,axeZ).Normalized() , axeY , axeZ);	 break;
            }

            //IMatrix3x3 R = IMatrix3x3::IDENTITY;
            //if(mLocation == LOCATE_WORLD) R = (m_svgMatrix).getRotMatrix().getInverse();

            m_SHIFTMatrix =  IMatrix4x4::CreateScaleAroundAxis( axeX , df.x) *
                             IMatrix4x4::CreateScaleAroundAxis( axeY , df.y) *
                             IMatrix4x4::CreateScaleAroundAxis( axeZ , df.z);

            (*m_pMatrix) = m_svgMatrix;
            (*m_pMatrix) = IMatrix4x4::CreateTranslation(worldPoint) *
                           m_SHIFTMatrix *
                           IMatrix4x4::CreateTranslation(-worldPoint) *
                           (*m_pMatrix);

        }
        else
        {
            m_SHIFTMatrix = IMatrix4x4::CreateScale(1.0+df.Length());

            (*m_pMatrix) = m_svgMatrix;
            (*m_pMatrix) = IMatrix4x4::CreateTranslation( worldPoint) * m_SHIFTMatrix *
                           IMatrix4x4::CreateTranslation(-worldPoint) * (*m_pMatrix);
        }

        return;

    }
    else
    {
        // predict move
        if (m_pMatrix)
        {
            GetOpType(mScaleTypePredict, x, y);
        }
    }

}

void CGizmoTransformScale::OnMouseUp(unsigned int x, unsigned int y)
{
    mScaleType = SCALE_NONE;
}

void CGizmoTransformScale::Draw()
{
    if (m_pMatrix)
    {
        ComputeScreenFactor();

        //glDisable(GL_DEPTH_TEST);
        IVector3 orig(m_pMatrix->GetTranslation());

        // axis
        IVector3 axeX(1,0,0),axeY(0,1,0),axeZ(0,0,1);
        if (mLocation == LOCATE_LOCAL)
        {
            IMatrix3x3 Rot = (*m_pMatrix).GetRotMatrix();
            //Rot.OrthoNormalize();

            axeX = axeX * Rot;
            axeY = axeY * Rot;
            axeZ = axeZ * Rot;
            axeX.Normalize();
            axeY.Normalize();
            axeZ.Normalize();
        }

        DrawTri(orig, 0.5f*GetScreenFactor(),((mScaleTypePredict==SCALE_XZ)||(mScaleTypePredict==SCALE_XYZ)), axeX, axeZ);
        DrawTri(orig, 0.5f*GetScreenFactor(),((mScaleTypePredict==SCALE_XY)||(mScaleTypePredict==SCALE_XYZ)), axeX, axeY);
        DrawTri(orig, 0.5f*GetScreenFactor(),((mScaleTypePredict==SCALE_YZ)||(mScaleTypePredict==SCALE_XYZ)), axeY, axeZ);

        axeX*=GetScreenFactor();
        axeY*=GetScreenFactor();
        axeZ*=GetScreenFactor();


        // plan1
        if (mScaleTypePredict != SCALE_X)
            DrawAxis(orig,axeX,axeY,axeZ,0.05f,0.83f,IVector4(1,0,0,1));
        else
            DrawAxis(orig,axeX,axeY,axeZ,0.05f,0.83f,IVector4(1,1,1,1));

        //plan2
        if (mScaleTypePredict != SCALE_Y)
            DrawAxis(orig,axeY,axeX,axeZ,0.05f,0.83f,IVector4(0,1,0,1));
        else
            DrawAxis(orig,axeY,axeX,axeZ,0.05f,0.83f,IVector4(1,1,1,1));

        //plan3
        if (mScaleTypePredict != SCALE_Z)
            DrawAxis(orig,axeZ,axeX,axeY,0.05f,0.83f,IVector4(0,0,1,1));
        else
            DrawAxis(orig,axeZ,axeX,axeY,0.05f,0.83f,IVector4(1,1,1,1));

/*
        // debug
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3fv(&ptd.x);
        glEnd();

        glEnable(GL_DEPTH_TEST);
        */
#if 0
#ifdef WIN32
    GDD->GetD3D9Device()->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    GDD->GetD3D9Device()->SetRenderState(D3DRS_CULLMODE , D3DCULL_NONE );
    GDD->GetD3D9Device()->SetRenderState(D3DRS_ZENABLE , D3DZB_TRUE);
    GDD->GetD3D9Device()->SetRenderState(D3DRS_ALPHATESTENABLE , FALSE);
    GDD->GetD3D9Device()->SetRenderState(D3DRS_ZWRITEENABLE , TRUE);
#endif
    extern RenderingState_t GRenderingState;
    GRenderingState.mAlphaTestEnable = 0;
    GRenderingState.mZWriteEnable = 1;
    GRenderingState.mBlending = 0;
    GRenderingState.mCulling = 0;
    GRenderingState.mZTestType = 1;
#endif
    }


}

void CGizmoTransformScale::ApplyTransform(IVector3& trans, bool bAbsolute)
{
//    if (bAbsolute)
//    {
//        IMatrix4x4 m_InvOrigScale,m_OrigScale;

//        m_OrigScale.Scaling(GetTransformedVector(0).Length(),
//        GetTransformedVector(1).Length(),
//        GetTransformedVector(2).Length());

//        m_InvOrigScale.Inverse(m_OrigScale);
//        m_svgMatrix = *m_pMatrix;

//        IMatrix4x4 mt;
//        mt.Scaling(trans.x/100.0f,trans.y/100.0f,trans.z/100.0f);
//        mt.Multiply(m_InvOrigScale);
//        mt.Multiply(m_svgMatrix);
//        *m_pMatrix=mt;
//    }
//    else
//    {
//        IMatrix4x4 mt,mt2;
//        m_svgMatrix = *m_pMatrix;
//        mt.Scaling(trans.x/100.0f,trans.y/100.0f,trans.z/100.0f);

//        mt2.SetLine(0,GetTransformedVector(0));
//        mt2.SetLine(1,GetTransformedVector(1));
//        mt2.SetLine(2,GetTransformedVector(2));
//        mt2.Translation(0,0,0);
//        mt.Multiply(mt2);
//        mt.Multiply(m_svgMatrix);
//        *m_pMatrix = mt;
//    }

}


}
