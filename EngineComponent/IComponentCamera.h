#ifndef ICOMPONENTCAMERA_H
#define ICOMPONENTCAMERA_H

#include "IComponentAbstract.h"

class IComponentCamera : public IComponentAbstract
{

private:



    IQCamera mCamera2;

    Vector3 mEye;
    Vector3 mCenter;
    Vector3 mUp;



public:

     IComponentCamera();
    ~IComponentCamera();


    void Setup() {}
    void Updatee()
    {
        mCamera2.SetTransformMatrix(mTransformMatrix);
    }


    void InitPerspective(scalar FieldOfView, scalar aspect, scalar NearPlane, scalar FarPlane)
    {
        mCamera2.SetNear(NearPlane);
        mCamera2.SetFar(FarPlane);
        mCamera2.SetAspect(aspect);
        mCamera2.SetAngle(FieldOfView);
    }

    void InitTransform()
    {
        mCamera2.LookAt( mEye , mCenter , mUp );
        SetTransformMatrix(mCamera2.ViewMatrix().GetInverse());
    }


    void TargetVision(const Vector3& target , const Vector3& _up = Vector3::Y)
    {
        mTransformMatrix.SetRotation( Quaternion::LookAtRH( mTransformMatrix.GetTranslation(), target, _up).GetConjugate().GetRotMatrix() );
    }


    void BeginLookAt()
    {
        mCamera2.LookAt( mEye , mCenter , mUp );
    }


    void DollyZoomFOV( float fov )
    {
        mCamera2.DollyZoom(fov);
    }


    Vector3 eye() const;
    Vector3 center() const;
    Vector3 up() const;

    void SetEye(const Vector3 &eye);
    void SetCenter(const Vector3 &center);
    void SetUp(const Vector3 &up);

    scalar getNear() const;
    scalar getFar() const;
    scalar getAspect() const;

    IQCamera getCamera2() const;
};

#endif // ICOMPONENTCAMERA_H
