

#include "IMathGizmo.h"
#include "IGizmoTransformRotate.h"
#ifdef MAC_OS
#import <OpenGL/OpenGL.h>
#else
#include <GL/gl.h>
#endif


#include <iostream>

namespace IuGizmo
{

    //////////////////////////////////////////////////////////////////////
    // Construction/Destruction
    //////////////////////////////////////////////////////////////////////
    extern IVector3 ptd;


    IGizmo *CreateRotateGizmo()
    {
        return new CGizmoTransformRotate;
    }


    CGizmoTransformRotate::CGizmoTransformRotate() : CGizmoTransform()
    {
        mRotateType = ROTATE_NONE;
        mRotateTypePredict = ROTATE_NONE;
        mNg2 = 0;
        mAngleSnap = 0.f;

    }

    CGizmoTransformRotate::~CGizmoTransformRotate()
    {

    }

    bool CGizmoTransformRotate::CheckRotatePlan(const IVector3 &vNorm, float factor, const IVector3 &rayOrig,const IVector3 &rayDir,int id , float& len , const IVector3& q , const IPlane& camPlan , bool isCol)
    {
        IVector3 df, inters;
        mPlan = IPlane(vNorm , m_pMatrix->GetTranslation() );;
        inters = mPlan.VIntersectionRayToPlane(rayOrig,rayDir);
        ptd = inters;
        df = inters - m_pMatrix->GetTranslation();
        df/=GetScreenFactor();

        float length = IAbs(rayDir.Dot(rayOrig - inters));

        IVector3 dir = m_pMatrix->GetTranslation()-mCamSrc;
        dir.Normalize();

        //***********************************************************//
        IVector3 origin = m_pMatrix->GetTranslation();
        IVector3 hit_direct = origin + (inters - origin).Normalized() * GetScreenFactor() * factor;

        length = (hit_direct - mPlan.ClosestPoint(q)).Length();
        length /= GetScreenFactor();
        //***********************************************************//

        if( ((length < len || len == -1) && dir.Dot(origin - hit_direct) >= 0.0) || isCol )
        {
            len = length;

            if( ( length < 0.05 || (((df.Length()/factor) >0.9f) && ( (df.Length()/factor) < 1.1f)))  /*&& m_CamDir.dot(inters - origin) >= -0.01)*/ || isCol )
            {

                mAxis2 = vNorm;
                m_svgMatrix = *m_pMatrix;

                mLockVertex = df;
                mLockVertex.Normalize();

                mVtx = mLockVertex;
                mVty = Cross(mLockVertex,vNorm);
                mVty.Normalize();
                mVtx *= factor;
                mVty *= factor;
                mVtz  = Cross(mVtx,mVty);
                mNg2 = 0;
                if (id!=-1)
                    mAxis = GetTransformedVector(id);

                mOrigScale = IMatrix4x4::CreateScale(GetTransformedVector(0).Length(),
                                                     GetTransformedVector(1).Length(),
                                                     GetTransformedVector(2).Length());

                mInvOrigScale = (mOrigScale.GetInverse());

                return true;
            }
        }


        return false;
    }


    void CGizmoTransformRotate::RotateAroundAxis1(const IVector3& rayOrigin,const IVector3& rayDir)
    {
        IVector3 inters;
        mPlan = IPlane( mAxis2 , m_pMatrix->GetTranslation());
        inters = mPlan.VIntersectionRayToPlane(rayOrigin,rayDir);
        ptd = inters;

        IVector3 df = inters - m_pMatrix->GetTranslation();
        df.Normalize();
        m_LockVertex2 = df;

        float acosng = df.Dot(mLockVertex);
        if ( (acosng<-0.99999f) || (acosng> 0.99999f) )
            mNg2 = 0.f;
        else
            mNg2 = (float)acos(acosng);

        if (df.Dot(mVty)>0)
            mNg2 = -mNg2;

        IMatrix4x4 mt,mt2;

        if (mbUseSnap)
        {
            mNg2*=(360.0f/M_PI);
            SnapIt(mNg2,mAngleSnap);
            mNg2/=(360.0f/M_PI);
        }


        //========================//

          //IVector3 _Axis = mAxis;

          m_SHIFTMatrix = IMatrix4x4::CreateRotationAxis(mAxis,mNg2);

          IVector3 worldPoint = m_svgMatrix.GetTranslation();
          (*m_pMatrix) = m_svgMatrix;
          (*m_pMatrix) = (IMatrix4x4::CreateTranslation( worldPoint)  *  m_SHIFTMatrix *
                          IMatrix4x4::CreateTranslation(-worldPoint)) * (*m_pMatrix);

        //========================//


//        if (mAxis == IVector3::Z)
//        {
//            if (mEditQT)
//            {
//                /*
//                Dans le cadre du jeu, et vu les pb avec les quaternions,
//                le 1er float du quaternion en stockage est l'angle en radian.
//                le stockage reste un quaternion.
//                il y a des pbs de conversion quaternion/matrix
//                */
//    #if USE_QUATERNION
//                tquaternion gth(*m_pMatrix);

//                gth.Normalize();
//                gth.UnitInverse();

//                tquaternion qtg;
//                qtg.RotationAxis(m_Axis,m_Ng2);
//                *mEditQT = gth;//gth+qtg;//tquaternion(mt2);
//                mEditQT->Normalize();
//    #else
//                mEditQT->SetW(mNg2);
//    #endif
//            }
//        }

    }




    bool CGizmoTransformRotate::IntersectRayToSphere(IVector3 rayOrigin, IVector3 rayDir, IVector3 center, float radius, float &t, IVector3 &q)
    {
        IVector3 m = rayOrigin - center;
        float b = Dot(m, rayDir);
        float c = Dot(m, m) - radius * radius;

        // Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0)
        if (c > 0.0f && b > 0.0f) return false;
        float discr = b*b - c;

        // A negative discriminant corresponds to ray missing sphere
        if (discr < 0.0f) return false;

        // Ray now found to intersect sphere, compute smallest t value of intersection
        t = -b - ISqrt(discr);

        // If t is negative, ray started inside sphere so clamp t to zero
        if (t < 0.0f) t = 0.0f;
        q = rayOrigin + t * rayDir;

        return true;
    }




    bool CGizmoTransformRotate::GetOpType(ROTATETYPE &type, unsigned int x, unsigned int y)
    {
        IVector3 rayOrigin,rayDir, axis;
        IVector3 dir = m_pMatrix->GetTranslation()-mCamSrc;
        dir.Normalize();

        BuildRay(x, y, rayOrigin, rayDir);


        //=======================================================//

        IPlane CamPlan( mCamDir , m_svgMatrix.GetTranslation());

        float min_dist = -1;
        type = ROTATE_NONE;


        float t;
        IVector3 q;

        if( IntersectRayToSphere( rayOrigin, rayDir , m_pMatrix->GetTranslation() , GetScreenFactor() * 1.0 , t , q) )
        {

            IVector3 axeX = GetTransformedVector(0);
            IVector3 axeY = GetTransformedVector(1);
            IVector3 axeZ = GetTransformedVector(2);


           // if (mMask&AXIS_TRACKBALL)
            if (CheckRotatePlan(dir,1.0f,rayOrigin,rayDir,-1,min_dist,q,CamPlan,true))
            {
                // m_LockVertex = q;
                // m_Axis2 = m_Axis * mt.getRotMatrix();

                IPlane CamPlane(mCamDir.Normalized(),m_pMatrix->GetTranslation());
                mLockVertex = CamPlane.VIntersectionRayToPlane(rayOrigin,rayDir);

                IMatrix4x4 mt = *m_pMatrix;
                type = ROTATE_TWIN;
            }

            //if (mMask&AXIS_X)
            if (CheckRotatePlan(axeX,1.0f,rayOrigin,rayDir,0,min_dist,q,CamPlan)) { type = ROTATE_X;  mAxis = mAxis2 = axeX; std::cout << " axeX "  << min_dist << std::endl; }

            //if (mMask&AXIS_Y)
            if (CheckRotatePlan(axeY,1.0f,rayOrigin,rayDir,1,min_dist,q,CamPlan)) { type = ROTATE_Y;  mAxis = mAxis2 = axeY; std::cout << " axeY " << min_dist << std::endl; }

            //if (mMask&AXIS_Z)
            if (CheckRotatePlan(axeZ,1.0f,rayOrigin,rayDir,2,min_dist,q,CamPlan)) { type = ROTATE_Z;  mAxis = mAxis2 = axeZ; std::cout << " axeZ " << min_dist << std::endl;  }

        }
        else if(IntersectRayToSphere( rayOrigin, rayDir , m_pMatrix->GetTranslation() , GetScreenFactor() * 1.2 , t , q))
        {
            //if (mMask&AXIS_SCREEN)
            if (CheckRotatePlan(dir,1.2f,rayOrigin,rayDir,-1,min_dist,q, CamPlan , true )) { type = ROTATE_SCREEN;  mAxis = mAxis2 = dir; std::cout << " ROTATE_SCREEN " << min_dist << std::endl;  }
        }


        return (min_dist != -1);

    }

    bool CGizmoTransformRotate::OnMouseDown(unsigned int x, unsigned int y)
    {
        mRotateType = ROTATE_NONE;
        if (m_pMatrix)
        {
            return GetOpType(mRotateType, x, y);
        }

        mRotateType = ROTATE_NONE;
        return false;

    }



    void CGizmoTransformRotate::OnMouseMove(unsigned int x, unsigned int y)
    {
        IVector3 rayOrigin, rayDir, axis;

        BuildRay(x, y, rayOrigin, rayDir);

        if (mRotateType != ROTATE_NONE)
        {
            if (mRotateType == ROTATE_TWIN)
            {
                IVector3 inters;
                IVector3 dir = m_pMatrix->GetTranslation()-mCamSrc;
                dir.Normalize();

                mPlan = IPlane(dir , m_pMatrix->GetTranslation());
                inters =  mPlan.VIntersectionRayToPlane(rayOrigin,rayDir);
                ptd = inters;
                IVector3 df = inters - m_pMatrix->GetTranslation();
                df/=GetScreenFactor();

                float t;
                IVector3 q;
                if( IntersectRayToSphere( rayOrigin, rayDir , m_pMatrix->GetTranslation() , GetScreenFactor() * 1.0 , t , q) )
                {

                    IPlane CamPlane(mCamDir.Normalized(),m_pMatrix->GetTranslation());
                    float angle1 = ((mLockVertex)-CamPlane.VIntersectionRayToPlane(rayOrigin,rayDir)).Dot(mCamRight);
                    float angle2 = ((mLockVertex)-CamPlane.VIntersectionRayToPlane(rayOrigin,rayDir)).Dot(mCamUp);
                    angle1 /= GetScreenFactor();
                    angle2 /= GetScreenFactor();


                    IQuaternion Q1;
                    IQuaternion Q2;
                    Q1 = IQuaternion::FromAngleAxis(mCamUp    , angle1 * M_PI * 0.5f);
                    Q2 = IQuaternion::FromAngleAxis(mCamRight ,-angle2 * M_PI * 0.5f);
                    Q1.Normalize();
                    Q2.Normalize();


                    m_SHIFTMatrix =  Q1.TransformMatrix() *
                                     Q2.TransformMatrix();

                    IVector3 worldPoint = m_svgMatrix.GetTranslation();
                    (*m_pMatrix) = m_svgMatrix;
                    (*m_pMatrix) = IMatrix4x4::CreateTranslation( worldPoint) *  m_SHIFTMatrix *
                                   IMatrix4x4::CreateTranslation(-worldPoint) * (*m_pMatrix);
                }
            }
            else
            {
                RotateAroundAxis1(rayOrigin, rayDir);
            }
        }
        else
        {
            // predict move
            if (m_pMatrix)
            {
                GetOpType(mRotateTypePredict, x, y);
            }
        }
    }

    void CGizmoTransformRotate::OnMouseUp(unsigned int x, unsigned int y)
    {
        mRotateType = ROTATE_NONE;
    }
    /*
                char tmps[512];
                sprintf(tmps, "%5.2f %5.2f %5.2f %5.2f", plCam.x, plCam.y, plCam.z, plCam.w );
                MessageBoxA(NULL, tmps, tmps, MB_OK);
                */
    void CGizmoTransformRotate::Draw()
    {
        if (m_pMatrix)
        {

            ComputeScreenFactor();


            IVector3 right,up,frnt,dir;

            //glDisable(GL_DEPTH_TEST);
            IVector3 orig(m_pMatrix->GetTranslation());

            IVector3 plnorm( mCamSrc-orig );
            plnorm.Normalize();



            tplane plCam = IVector4(plnorm,0);

            dir = orig-mCamSrc;
            dir.Normalize();

            right = Cross(dir,GetTransformedVector(1));
            right.Normalize();

            up = Cross(dir,right);
            up.Normalize();

            right = Cross(dir,up);
            right.Normalize();

            IVector3 axeX(1,0,0),axeY(0,1,0),axeZ(0,0,1);


            if (mLocation == LOCATE_LOCAL)
            {
                axeX = axeX * (*m_pMatrix).GetRotMatrix();
                axeY = axeY * (*m_pMatrix).GetRotMatrix();
                axeZ = axeZ * (*m_pMatrix).GetRotMatrix();
                axeX.Normalize();
                axeY.Normalize();
                axeZ.Normalize();

            }

            // Twin
            if (mMask&AXIS_TRACKBALL)
            {

                if (mRotateTypePredict != ROTATE_TWIN)
                    DrawCircle(orig, 0.2f,0.2f,0.2f,right*GetScreenFactor(),up*GetScreenFactor());
                else
                    DrawCircle(orig, 1,1,1,right*GetScreenFactor(),up*GetScreenFactor());
            }

            // Screen
            if (mMask&AXIS_SCREEN)
            {
                if (mRotateTypePredict != ROTATE_SCREEN)
                    DrawCircle(orig, 1.0f,0.3f,1.0f,up*1.2f*GetScreenFactor(),right*1.2f*GetScreenFactor());
                else
                    DrawCircle(orig, 1,1,1,up*1.2f*GetScreenFactor(),right*1.2f*GetScreenFactor());
            }

            // X
            right = Cross(dir, axeX);
            right.Normalize();
            frnt = Cross(right, axeX);

            frnt.Normalize();

            if (mMask&AXIS_X)
            {
                if (mRotateTypePredict != ROTATE_X)
                    DrawCircleHalf(orig, 1,0,0,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
                else
                    DrawCircleHalf(orig, 1,1,1,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
            }

            // Y

            right = Cross(dir, axeY);
            right.Normalize();
            frnt = Cross(right, axeY);
            frnt.Normalize();

            if (mMask&AXIS_Y)
            {

                if (mRotateTypePredict != ROTATE_Y)
                    DrawCircleHalf(orig, 0,1,0,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
                else
                    DrawCircleHalf(orig, 1,1,1,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
            }

            // Z
            right = Cross(dir, axeZ);
            right.Normalize();
            frnt = Cross(right, axeZ);
            frnt.Normalize();

            if (mMask&AXIS_Z)
            {
                if (mRotateTypePredict != ROTATE_Z)
                    DrawCircleHalf(orig, 0,0,1,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
                else
                    DrawCircleHalf(orig, 1,1,1,right*GetScreenFactor(),frnt*GetScreenFactor(),plCam);
            }
            // camembert
            if ( (mRotateType != ROTATE_NONE) && (mRotateType != ROTATE_TWIN ) )
                DrawCamem(orig,mVtx*GetScreenFactor(),mVty*GetScreenFactor(),-mNg2);
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

    void CGizmoTransformRotate::ApplyTransform(IVector3& trans, bool bAbsolute)
    {
        IMatrix4x4 mt;
        mOrigScale = IMatrix4x4::CreateScale(GetTransformedVector(0).Length(),
                                             GetTransformedVector(1).Length(),
                                             GetTransformedVector(2).Length());

        if (bAbsolute)
        {
            IVector3 translation = m_pMatrix->GetTranslation();

            /**
            //X
            mt.RotationAxis(GetVector(0),((trans.x/360)*ZPI));
            mt.Multiply(m_OrigScale);
            *m_pMatrix=mt;
            //Y
            mt.RotationAxis(GetVector(1),((trans.y/360)*ZPI));
            mt.Multiply(m_OrigScale);
            *m_pMatrix=mt;
            //Z
            mt.RotationAxis(GetVector(2),((trans.z/360)*ZPI));
            mt.Multiply(m_OrigScale);
            *m_pMatrix=mt;
            /**/

            //translate
            m_pMatrix[12] = translation.x;
            m_pMatrix[13] = translation.y;
            m_pMatrix[14] = translation.z;
        }
        else
        {
            IMatrix4x4 mt2;
            mInvOrigScale = (mOrigScale.GetInverse());

            if (trans.x!=0)
            {
                m_svgMatrix = *m_pMatrix;
                /**
                mt.RotationAxis(GetVector(0),((trans.x/360)*ZPI));
                mt.Multiply(m_InvOrigScale);
                mt.Multiply(m_svgMatrix);
                mt2 = m_OrigScale;
                mt2.Multiply(mt);
                /**/
                *m_pMatrix=mt2;
            }
            if (trans.y!=0)
            {
                m_svgMatrix = *m_pMatrix;
                /**
                mt.RotationAxis(GetVector(1),((trans.y/360)*ZPI));
                mt.Multiply(m_InvOrigScale);
                mt.Multiply(m_svgMatrix);
                mt2 = m_OrigScale;
                mt2.Multiply(mt);
                /**/
                *m_pMatrix=mt2;
            }
            if (trans.z!=0)
            {
                m_svgMatrix = *m_pMatrix;
                /**
                mt.RotationAxis(GetVector(2),((trans.z/360)*ZPI));
                mt.Multiply(m_InvOrigScale);
                mt.Multiply(m_svgMatrix);
                mt2 = m_OrigScale;
                mt2.Multiply(mt);
                /**/
                *m_pMatrix=mt2;
            }
        }
    }

}
