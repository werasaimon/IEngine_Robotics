#include "ISensorLIDAR.h"






ISensorLIDAR::ISensorLIDAR(IDynamicsWorld *_DynamicsWorld,
                           const Vector3 &_direction,
                           const Transform &_transform) :
    m_pDynamicsWorld(_DynamicsWorld),
    mDirection(_direction),
    mTransform(_transform)
{

}

ISensorLIDAR::~ISensorLIDAR()
{

}

Vector3 ISensorLIDAR::WorldDirection() const
{
    return mTransform.GetBasis() * mDirection;
}

scalar ISensorLIDAR::LAngleRotation(const Vector3 &_NormalFace)
{
    Vector3 V1 = mDirection;
    Vector3 V2 = mTransform.GetRotation() * mDirection;
    return Vector3::AngleSigned(V1,V2,_NormalFace);
}



