
#include "IMathGizmo.h"
#include "IGizmoTransformMove.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif

#include <iostream>


namespace IuGizmo
{




IGizmo *CreateMoveGizmo()
{
    return new CGizmoTransformMove;
}

IVector3 ptd;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CGizmoTransformMove::CGizmoTransformMove() : CGizmoTransform()
{
    mMoveType = MOVE_NONE;
}

CGizmoTransformMove::~CGizmoTransformMove()
{

}


int CGizmoTransformMove::RayTracePlaneOrtoAxis(const IVector3 &RayOrigin, const IVector3 &RayDir, const IVector3 &Origin, const IVector3 &Axis_A, const IVector3 &Axis_B , IVector3& inters)
{

    IVector3 axis_a = Axis_A;
    IVector3 axis_b = Axis_B;
    axis_a.Normalize();
    axis_b.Normalize();

    IVector3 vNormal = Cross(axis_a,axis_b);

    if( vNormal.LengthSquare() < 0.00001f) return 0;

    vNormal.Normalize();

    IPlane plane(vNormal,Origin);
    inters =plane.VIntersectionRayToPlane(RayOrigin,RayDir);

    IVector3 sumV = (axis_a+axis_b);

    //======================================//

    IMatrix3x3 RotInverse = IMatrix3x3::IDENTITY;
    if(mLocation == LOCATION::LOCATE_WORLD )
    {
       RotInverse = IMatrix3x3::IDENTITY;
    }
    else if( mLocation == LOCATE_LOCAL)
    {
       RotInverse = m_pMatrix->GetRotMatrix().GetInverse();
    }

    //=====================================//


    axis_a  = axis_a  * RotInverse;
    axis_b  = axis_b  * RotInverse;
    vNormal = vNormal * RotInverse;

    IVector3 predictA =  Cross(vNormal,axis_a);
    IVector3 predictB = -Cross(vNormal,axis_b);


    IVector3 df = (inters - Origin) * RotInverse;

    sumV = (sumV * RotInverse);

    float Lu = (sumV).Dot(axis_a);
    float Lv = (sumV).Dot(axis_b);

    float ScreenFactor = GetScreenFactor();

    if( (df).Dot(predictA) >= 0 &&
        (df).Dot(predictB) >= 0 &&
        (df).Dot(axis_a)/ScreenFactor <= (Lu * 0.5) &&
        (df).Dot(axis_b)/ScreenFactor <= (Lv * 0.5))
    {
        return 1;
    }
    else
    {
        /**
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



bool CGizmoTransformMove::GetOpType(MOVETYPE &type, unsigned int x, unsigned int y)
{
    IVector3 rayOrigin, rayDir, df;
    BuildRay(x, y, rayOrigin, rayDir);

    m_svgMatrix = *m_pMatrix;

    IVector3 origin = m_svgMatrix.GetTranslation();

    IVector3 axisTrX = GetTransformedVector(0);
    IVector3 axisTrY = GetTransformedVector(1);
    IVector3 axisTrZ = GetTransformedVector(2);

    axisTrX *= GetScreenFactor();
    axisTrY *= GetScreenFactor();
    axisTrZ *= GetScreenFactor();



    IVector3 inters;
    float minimum_distance_nearest = -1;

    int tp = 0;

    type = MOVE_NONE;


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
                 case 1 : type = MOVE_XZ; break;
                 case 2 : type = MOVE_X; break;
                 case 3 : type = MOVE_Z; break;
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

            mPlan = IPlane( Cross(axisTrX,axisTrY).Normalized() , origin );
            mLockVertex = inters;

            switch (tp)
            {
                case 1 : type = MOVE_XY; break;
                case 2 : type = MOVE_X; break;
                case 3 : type = MOVE_Y; break;
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

            mPlan = IPlane( Cross(axisTrY,axisTrZ).Normalized() , origin );
            mLockVertex = inters;

            switch (tp)
            {
                case 1 : type = MOVE_YZ; break;
                case 2 : type = MOVE_Y; break;
                case 3 : type = MOVE_Z; break;
            }
        }
    }


    return (minimum_distance_nearest>0);
}




bool CGizmoTransformMove::OnMouseDown(unsigned int x, unsigned int y)
{
    mMoveType = MOVE_NONE;
    if (m_pMatrix)
    {
        bool b =  GetOpType(mMoveType, x, y);

        if(mMoveType != MOVE_NONE)
        {

        }

        return b;
    }

    mMoveType = MOVE_NONE;
    return false;
}



void CGizmoTransformMove::OnMouseMove(unsigned int x, unsigned int y)
{
    if (mMoveType != MOVE_NONE)
    {
        IVector3 rayOrigin,rayDir,df, inters;

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

        df  = inters - mLockVertex;
        //df /= GetScreenFactor();

        if(mMoveType == MOVE_XY || mMoveType == MOVE_XZ || mMoveType == MOVE_YZ)
        {

            if (mLocation == LOCATE_LOCAL)
            {
                IMatrix3x3 Rot = (m_svgMatrix).GetRotMatrix();
                Rot.OrthoNormalize();
                df = df * Rot.GetInverse();
            }


            IVector3 X =  IVector3::X;
            IVector3 Y =  IVector3::Y;
            IVector3 Z =  IVector3::Z;

            switch (mMoveType)
            {
                case MOVE_XZ:	df = IVector3(df.Dot(X), 0, df.Dot(Z));	break;
                case MOVE_XY:	df = IVector3(df.Dot(X), df.Dot(Y), 0);	break;
                case MOVE_YZ:	df = IVector3(0,df.Dot(Y) , df.Dot(Z));	break;
            }


            if (mbUseSnap)
            {
                SnapIt(df.x,mMoveSnap.x);
                SnapIt(df.y,mMoveSnap.y);
                SnapIt(df.z,mMoveSnap.z);
            }

            IVector3 adf = df.x*axeX +
                           df.y*axeY +
                           df.z*axeZ;

            m_SHIFTMatrix = IMatrix4x4::CreateTranslation(adf);
            *m_pMatrix = m_svgMatrix;
            *m_pMatrix = m_SHIFTMatrix * *m_pMatrix;

        }
        else if( mMoveType == MOVE_X || mMoveType == MOVE_Y || mMoveType == MOVE_Z )
        {


            switch (mMoveType)
            {
                case MOVE_X:	df = IVector3(df.Dot(axeX) , 0,0);					break;
                case MOVE_Z:	df = IVector3(0, 0,df.Dot(axeZ));		    		break;
                case MOVE_Y:	df = IVector3(0,df.Dot(axeY), 0);					break;
            }


            if (mbUseSnap)
            {
                SnapIt(df.x,mMoveSnap.x);
                SnapIt(df.y,mMoveSnap.y);
                SnapIt(df.z,mMoveSnap.z);
            }

            IVector3 adf = df.x*axeX +
                           df.y*axeY +
                           df.z*axeZ;


            m_SHIFTMatrix = IMatrix4x4::CreateTranslation(adf);
            *m_pMatrix = m_svgMatrix;
            *m_pMatrix = m_SHIFTMatrix * *m_pMatrix;

        }

    }
    else
    {
        // predict move
        if (m_pMatrix)
        {
            GetOpType(mMoveTypePredict, x, y);
        }
    }

}

void CGizmoTransformMove::OnMouseUp(unsigned int x, unsigned int y)
{
    mMoveType = MOVE_NONE;
}

void CGizmoTransformMove::Draw()
{

    if (m_pMatrix)
    {

        ComputeScreenFactor();

        glDisable(GL_DEPTH_TEST);
        IVector3 orig = m_pMatrix->GetTranslation();


       // std::cout << orig << std::endl;

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




        DrawQuad(orig, 0.5f*GetScreenFactor(), (mMoveTypePredict == MOVE_XZ), axeX, axeZ);
        DrawQuad(orig, 0.5f*GetScreenFactor(), (mMoveTypePredict == MOVE_XY), axeX, axeY);
        DrawQuad(orig, 0.5f*GetScreenFactor(), (mMoveTypePredict == MOVE_YZ), axeY, axeZ);

        axeX*=GetScreenFactor();
        axeY*=GetScreenFactor();
        axeZ*=GetScreenFactor();

        // plan1
        if (mMoveTypePredict != MOVE_X) DrawAxis(orig,axeX,axeY,axeZ,0.05f,0.83f,IVector4(1,0,0,1));
            else DrawAxis(orig,axeX,axeY,axeZ, 0.05f,0.83f,IVector4(1,1,1,1));

        //plan2
        if (mMoveTypePredict != MOVE_Y) DrawAxis(orig,axeY,axeX,axeZ, 0.05f,0.83f,IVector4(0,1,0,1));
            else DrawAxis(orig,axeY,axeX,axeZ, 0.05f,0.83f,IVector4(1,1,1,1));

        //plan3
        if (mMoveTypePredict != MOVE_Z) DrawAxis(orig,axeZ,axeX,axeY, 0.05f,0.83f,IVector4(0,0,1,1));
            else DrawAxis(orig,axeZ,axeX,axeY, 0.05f,0.83f,IVector4(1,1,1,1));
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
/*

PSM_LVERTEX svVts[2];
    svVts[0].x = ptd.x;
    svVts[0].y = ptd.y;
    svVts[0].z = ptd.z;
    svVts[0].diffuse = 0xFFFFFFFF;

    svVts[1].x = ptd.x+10;
    svVts[1].y = ptd.y+10;
    svVts[1].z = ptd.z+10;
    svVts[1].diffuse = 0xFFFFFFFF;


    IDirect3DDevice9 *pDev = ((PSM_D3D9RenderDevice*)PSM_D3D9RenderDevice::GetInterfacePtr())->d3dDevice;
    pDev->DrawPrimitiveUP(D3DPT_LINESTRIP , 1, svVts, sizeof(PSM_LVERTEX));
    */
}
/*
        // debug
        glPointSize(20);
        glBegin(GL_POINTS);
        glVertex3fv(&ptd.x);
        glEnd();

        glEnable(GL_DEPTH_TEST);
*/

}

void CGizmoTransformMove::ApplyTransform(IVector3& trans, bool bAbsolute)
{
//    if (bAbsolute)
//    {
//        m_pMatrix->mData[12] = trans.x;
//        m_pMatrix->mData[13] = trans.y;
//        m_pMatrix->mData[14] = trans.z;
//    }
//    else
//    {
//        m_pMatrix->mData[12] += trans.x;
//        m_pMatrix->mData[13] += trans.y;
//        m_pMatrix->mData[14] += trans.z;
//    }
}

}


