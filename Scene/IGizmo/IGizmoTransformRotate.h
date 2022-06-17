#ifndef IGIZMOTRANSFORMROTATE_H
#define IGIZMOTRANSFORMROTATE_H


#include "IGizmoTransform.h"


namespace IuGizmo
{

    class CGizmoTransformRotate : public CGizmoTransform
    {


    protected:

        enum ROTATETYPE
        {
            ROTATE_NONE,
            ROTATE_X,
            ROTATE_Y,
            ROTATE_Z,
            ROTATE_SCREEN,
            ROTATE_TWIN
        };

        ROTATETYPE mRotateType;
        ROTATETYPE mRotateTypePredict;

        IPlane     mPlan;
        IVector3   mLockVertex,m_LockVertex2;
        float      mNg2;
        IVector3   mVtx,mVty,mVtz;
        IVector3   mAxis,mAxis2;
        IMatrix4x4 mOrigScale,mInvOrigScale;
        float      mAngleSnap;

    public:
                 CGizmoTransformRotate();
        virtual ~CGizmoTransformRotate();

        bool  isOnMouseDownSelected() const { return mRotateType != ROTATETYPE::ROTATE_NONE; }

        // return true if gizmo transform capture mouse
        virtual bool OnMouseDown(unsigned int x, unsigned int y);
        virtual void OnMouseMove(unsigned int x, unsigned int y);
        virtual void OnMouseUp(unsigned int x, unsigned int y);

        virtual void Draw();

        virtual void SetSnap(const float snap) {mAngleSnap = snap; }
        virtual void SetSnap(float snapx, float snapy, float snapz) {}
        /*
        void SetAngleSnap(float snap)
        {
            m_AngleSnap = snap;
        }
        */

        float GetAngleSnap()
        {
            return mAngleSnap;
        }

        virtual void ApplyTransform(IVector3& trans, bool bAbsolute);

    protected:

        bool GetOpType(ROTATETYPE &type, unsigned int x, unsigned int y);

        bool CheckRotatePlan(const IVector3 &vNorm, float factor, const IVector3 &rayOrig, const IVector3 &rayDir, int id, float &len, const IVector3 &q,  const IPlane& camPlan , bool isCol=false);

        void RotateAroundAxis1(const IVector3& rayOrigin,const IVector3& rayDir);

        bool IntersectRayToSphere(IVector3 rayOrigin, IVector3 rayDir, IVector3 center, float radius, float &t, IVector3 &q);

    };

}

#endif // IGIZMOTRANSFORMROTATE_H
