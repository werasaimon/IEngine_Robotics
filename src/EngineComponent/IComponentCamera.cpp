#include "IComponentCamera.h"





IComponentCamera::IComponentCamera()
    : IComponentAbstract(TYPE_COMPONENT::CAMERA),
      mUp(Vector3::Y)
{

}

IComponentCamera::~IComponentCamera()
{

}


IQCamera IComponentCamera::getCamera2() const
{
    return mCamera2;
}

Vector3 IComponentCamera::center() const
{
    return mCenter;
}

Vector3 IComponentCamera::up() const
{
    return mUp;
}


void IComponentCamera::SetEye(const Vector3 &eye)
{
    mEye = eye;
}

void IComponentCamera::SetCenter(const Vector3 &center)
{
    mCenter = center;
}

void IComponentCamera::SetUp(const Vector3 &up)
{
    mUp = up;
}



