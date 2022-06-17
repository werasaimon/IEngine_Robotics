///////////////////////////////////////////////////////////////////////////////////////////////////
// LibGizmo
// File Name :
// Creation : 10/01/2012
// Author : Cedric Guillemet
// Description : LibGizmo
//
///Copyright (C) 2012 Cedric Guillemet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
//of the Software, and to permit persons to whom the Software is furnished to do
///so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
///FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef GIZMOTRANSFORM_H__
#define GIZMOTRANSFORM_H__

#include "IMathGizmo.h"

#include "IGizmoTransformRender.h"
#include "IUGizmo.h"

#include <stdlib.h>


namespace IuGizmo
{


#define EPS 0.0001


class CGizmoTransform : public IGizmo , protected CGizmoTransformRender
{

protected:

    int mScreenWidth;
    int mScreenHeight;

    LOCATION    mLocation;

    IPlane      mPlan;
    IVector3    mLockVertex;
    float       mLng;

    IMatrix4x4 *m_pMatrix;
    IMatrix4x4  m_svgMatrix;
    IMatrix4x4  m_SHIFTMatrix;

    IMatrix4x4  mViewMatrix;
    IMatrix4x4  mProjMatrix;

    IVector3    mCamSrc;
    IVector3    mCamDir;
    IVector3    mCamUp;
    IVector3    mCamRight;

    bool        mbUseSnap;

    float       mScreenFactor;
    float       mDisplayScale;


    IMatrix4x4 mWorkingMatrix; // for dissociated components

//    IVector3    *mEditPos;
//    IVector3    *mEditScale;
//    IQuaternion *mEditQT;
//    //draw helpers


public:


    bool isTransformationChange() const
    {
        return *m_pMatrix != m_svgMatrix;
    }

    virtual bool  isOnMouseDownSelected() const
    {
        return false;
    }

    CGizmoTransform()
    {
        m_pMatrix = nullptr;
        mbUseSnap = false;
//        mEditPos =  mEditScale = nullptr;
//        mEditQT = nullptr;
        mMask = AXIS_ALL;
        mLng = 1.f;
        mScreenHeight = mScreenWidth = 1;
        mScreenFactor = 1;
        mDisplayScale = 1.f;
    }

    virtual ~CGizmoTransform()
    {
    }



    void IndentitySHIFTMatrix()
    {
        m_SHIFTMatrix.SetToIdentity();
    }

    IMatrix4x4 GetMatrix() const
    {
      return *m_pMatrix;
    }

    IMatrix4x4 GetFixMatrix() const
    {
      return *m_svgMatrix;
    }

    IMatrix4x4 GetSHIFTMatrix() const
    {
       return m_SHIFTMatrix;
    }

    virtual void SetEditMatrix(const float *pMatrix)
    {
       m_pMatrix = (IMatrix4x4*)pMatrix;
       m_svgMatrix = *m_pMatrix;
//        mEditPos = mEditScale = nullptr;
//        mEditQT = nullptr;
    }


    void SetEditMatrixMove(float *pMatrix)
    {
        m_pMatrix = (IMatrix4x4*)pMatrix;
        m_svgMatrix = *m_pMatrix;
    }


    virtual void SetDisplayScale( float aScale )
    {
        mDisplayScale = aScale;
    }

    virtual void SetScreenDimension( int screenWidth, int screenHeight)
    {
        mScreenWidth = screenWidth;
        mScreenHeight = screenHeight;
    }

    virtual void SetCameraMatrix(const float *View, const float *Proj)
    {
        mViewMatrix = *(IMatrix4x4*)View;
        mProjMatrix = *(IMatrix4x4*)Proj;

        IMatrix4x4 invModelView = mViewMatrix.GetInverse();

        mCamSrc    = invModelView.GetTranslation();
        mCamRight = IVector3::X * invModelView.GetRotMatrix();
        mCamUp    = IVector3::Y * invModelView.GetRotMatrix();
        mCamDir   = IVector3::Z * invModelView.GetRotMatrix();
    }


    // tools
    void BuildRay(int x, int y, IVector3 &rayOrigin, IVector3 &rayDir)
    {
        float frameX = (float)mScreenWidth;
        float frameY = (float)mScreenHeight;

        IVector3 screen_space;

        // device space to normalized screen space
        screen_space.x =  ((x / frameX ) - 0.5f) * 2.0;
        screen_space.y = -((y / frameY ) - 0.5f) * 2.0;
        screen_space.z = -1.f;

        IVector3   WorldScreen =  screen_space * (mProjMatrix * mViewMatrix).GetInverse();

        // Ray-world
        rayOrigin = mViewMatrix.GetInverse().GetTranslation();;
        rayDir    = (WorldScreen - rayOrigin).Normalized();


    }


    IVector3 GetTransformedVector(int vtID)
    {
        IVector3 vt;
        switch (vtID)
        {
        case 0: vt = IVector3(1,0,0); break;
        case 1: vt = IVector3(0,1,0); break;
        case 2: vt = IVector3(0,0,1); break;
        }
        if (mLocation == LOCATE_LOCAL )
        {
            IMatrix3x3 RotAng = (*m_pMatrix).GetRotMatrix();
            RotAng.OrthoNormalize();
            vt = vt * RotAng;
            vt.Normalize();
        }
        return vt;
    }


    virtual void SetAxisMask(unsigned int mask)
    {
        mMask = mask;
    }

    void ComputeScreenFactor()
    {
        IMatrix4x4 viewproj = mProjMatrix * mViewMatrix;
        IVector4 trf = IVector4( m_pMatrix->GetTranslation(), 1.f);
        trf = trf * viewproj;
        mScreenFactor = mDisplayScale * 0.03f * trf.w;
    }

    float GetScreenFactor()
    {
        return mScreenFactor;
    }

    // snap
    virtual void UseSnap(bool bUseSnap)
    {
        mbUseSnap = bUseSnap;
    }

    virtual bool IsUsingSnap()
    {
        return mbUseSnap;
    }
    //transform
    virtual void ApplyTransform(IVector3& trans, bool bAbsolute) = 0;

    void SetLocation(LOCATION aLocation)
    {
        mLocation = aLocation;
    }

    LOCATION GetLocation()
    {
        return mLocation;
    }




protected:

    unsigned int mMask;
    void SnapIt(float &pos, float &snap)
    {
        float sn = (float)fmod(pos,snap);
        if (fabs(sn)< (snap*0.25f) ) pos-=sn;
        if (fabs(sn)> (snap*0.75f) ) pos=( (pos-sn) + ((sn>0)?snap:-snap) );
    }


    static  bool IntersectLineToLine(IVector3 p1, IVector3 p2, IVector3 p3, IVector3 p4, float& distance )
    {
            IVector3 p13,p43,p21;

            p13 = p1 - p3;
            p43 = p4 - p3;
            p21 = p2 - p1;


            if( fabs(p43.x) < EPS &&
                fabs(p43.y) < EPS &&
                fabs(p43.z) < EPS)
                return false;


            if (fabs(p21.x) < EPS &&
                fabs(p21.y) < EPS &&
                fabs(p21.z) < EPS)
                return false;


            float d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
            float d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
            float d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
            float d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
            float d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

            float denom = d2121 * d4343 - d4321 * d4321;

            if (fabs(denom) < EPS)
                return false;

            float numer = d1343 * d4321 - d1321 * d4343;
            float mua = numer / denom;
            float mub = (d1343 + d4321 * (mua)) / d4343;

            IVector3 ip_a =  p1 + mua * p21;
            IVector3 ip_b =  p3 + mub * p43;

            distance = (ip_a - ip_b).Length();

            // Means we have an intersection
            return (mua >= 0.0 && mua <= 1.0 &&
                    mub >= 0.0 && mub <= 1.0 );

    }

};




}


#endif // !defined(AFX_GIZMOTRANSFORM_H__913D353E_E420_4B1C_95F3_5A0258161651__INCLUDED_)
