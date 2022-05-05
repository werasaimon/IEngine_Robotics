#ifndef IGIZMOTRANSFORMSCALE_H
#define IGIZMOTRANSFORMSCALE_H

#include "IGizmoTransform.h"

namespace IuGizmo
{


class CGizmoTransformScale : public CGizmoTransform
{

protected:

    enum SCALETYPE
    {
        SCALE_NONE,
        SCALE_X,
        SCALE_Y,
        SCALE_Z,
        SCALE_XY,
        SCALE_XZ,
        SCALE_YZ,
        SCALE_XYZ
    };

    SCALETYPE mScaleType;
    SCALETYPE mScaleTypePredict;

    unsigned int mLockX;
    unsigned int mLockY;

    float        mScaleSnap;


public:


             CGizmoTransformScale();
    virtual ~CGizmoTransformScale();


    bool  isOnMouseDownSelected() const { return mScaleType != SCALETYPE::SCALE_NONE; }

    // return true if gizmo transform capture mouse
    virtual bool OnMouseDown(unsigned int x, unsigned int y);
    virtual void OnMouseMove(unsigned int x, unsigned int y);
    virtual void OnMouseUp(unsigned int x, unsigned int y);

    virtual void Draw();

    /*
    void SetScaleSnap(float snap)
    {
        m_ScaleSnap = snap;
    }
    */
    virtual void SetSnap(const float snap)
    {
       mScaleSnap = snap;
    }

    virtual void SetSnap(float snapx, float snapy, float snapz)
    {
       mScaleSnap = snapx + snapy + snapz;
    }

    float GetScaleSnap()
    {
        return mScaleSnap;
    }

    virtual void ApplyTransform(IVector3& trans, bool bAbsolute);

protected:

   void SnapScale(float &val);

   bool GetOpType(SCALETYPE &type, unsigned int x, unsigned int y);

   int  RayTracePlaneOrtoAxis(const IVector3& RayOrigin , const IVector3& RayDir ,  const IVector3& Origin ,  const IVector3& Axis_A ,  const IVector3& Axis_B , IVector3 &inters);



};



}

#endif // IGIZMOTRANSFORMSCALE_H
