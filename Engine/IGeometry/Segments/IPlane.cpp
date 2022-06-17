

#include "IPlane.h"


namespace IEngine
{

using namespace  IMath;

IPlane IPlane::operator=(const IPlane &other)
{
    // if same object
    if ( this == &other )
        return *this;

    mNormal = other.mNormal;
    mOffset = other.mOffset;

    return *this;
}

bool IPlane::operator!=(const IPlane &plane) const
{
    return !(plane.mNormal == mNormal &&
             plane.mOffset == mOffset);
}

void IPlane::Set(scalar a, scalar b, scalar c, scalar d)
{
    // normalize for cheap distance checks
    scalar lensq = a*a + b*b + c*c;

    // length of normal had better not be zero
    assert( !IIsZero( lensq ) );

    // recover gracefully
    if ( IIsZero( lensq ) )
    {
        mNormal = Vector3::X;
        mOffset = 0.0f;
    }
    else
    {
        scalar recip = scalar(1.0)/ISqrt(lensq);
        mNormal.SetAllValues( a*recip, b*recip, c*recip );
        mOffset = d*recip;
    }
}

void IPlane::Set(const Vector3 &p0, const Vector3 &p1, const Vector3 &p2)
{
    mNormal = (Vector3::triNormal(p0,p1,p2));
    mOffset = mNormal.Dot(p0);
}

const Vector4 &IPlane::GetVec4()
{
    return Vector4(mNormal,mOffset);
}



IPlane IPlane::Transform(scalar scale, const Quaternion &rotate, const Vector3 &translate) const
{
    IPlane plane;

    // get rotation matrix
    Matrix3  rotmatrix = rotate.GetRotMatrix();

    // transform to get normal
    plane.mNormal = rotmatrix*mNormal/scale;

    // transform to get offset
    Vector3 newTrans = translate*rotmatrix;
    plane.mOffset = -newTrans.Dot( mNormal )/scale + mOffset;

    return plane;
}


IPlane IPlane::Transform(scalar scale, const Matrix3 &rotmatrix, const Vector3 &translate) const
{
    IPlane plane;

    // transform to get normal
    plane.mNormal = rotmatrix*mNormal/scale;

    // transform to get offset
    Vector3 newTrans = translate*rotmatrix;
    plane.mOffset = -newTrans.Dot( mNormal )/scale + mOffset;

    return plane;
}


scalar IPlane::Distance(const IPlane &plane, const Vector3 &point)
{
    return ( plane.Test( point ) );
}

Vector3 IPlane::ClosestPoint(const Vector3 &point) const
{
    return point - Test(point)*mNormal;
}

scalar IPlane::Test(const Vector3 &point) const
{
    return mNormal.Dot(point) - mOffset;
}

scalar IPlane::InvTest(const Vector3 &point) const
{
    return mOffset - mNormal.Dot(point);
}

Vector3 IPlane::VIntersectionLineToPlane(const ILineSegment3D &_edge) const
{
    Vector3 N = mNormal;
    Vector3 P = _edge.GetEndpoint0();
    Vector3 W = _edge.GetDirection();

    scalar  d =  InvTest(P);
    scalar  e =  N.Dot(W);

    if( IAbs(e) < MACHINE_EPSILON  ) return P;

    scalar param = d/e;
    return P + W * param;
}



Vector3 IPlane::VIntersectionRayToPlane(const Vector3 &_ray_origin, const Vector3 &_ray_dir) const
{
    Vector3 N = mNormal;
    Vector3 P = _ray_origin;
    Vector3 W = _ray_dir;

    scalar  d =  InvTest(P);
    scalar  e =  N.Dot(W);

    if( IAbs(e) < MACHINE_EPSILON  ) return P;

    scalar param = d/e;

    return P + W * param;
}


//unsigned int IPlane::AABBIntersect(const Vector3 &aabb_min, const Vector3 &aabb_max) const
//{
//    unsigned int ret =  INTERSECTION_TYPE::INSIDE;

//    Vector3 vMin;
//    Vector3 vMax;

//    // X axis
//    if(GetNormal().x > 0)
//    {
//        vMin.x = aabb_min.x;
//        vMax.x = aabb_max.x;
//    }
//    else
//    {
//        vMin.x = aabb_max.x;
//        vMax.x = aabb_min.x;
//    }
//    // Y axis
//    if(GetNormal().y > 0)
//    {
//        vMin.y = aabb_min.y;
//        vMax.y = aabb_max.y;
//    }
//    else
//    {
//        vMin.y = aabb_max.y;
//        vMax.y = aabb_min.y;
//    }
//    // Z axis
//    if(GetNormal().z > 0)
//    {
//        vMin.z = aabb_min.z;
//        vMax.z = aabb_max.z;
//    }
//    else
//    {
//        vMin.z = aabb_max.z;
//        vMax.z = aabb_min.z;
//    }

//    if( vMin.Dot(GetNormal()) - GetOffset() >  0)  return INTERSECTION_TYPE::OUTSIDE;
//    if( vMax.Dot(GetNormal()) - GetOffset() >= 0) ret = INTERSECTION_TYPE::INTERSECT;

//    return ret;
//}

unsigned int IPlane::AABBIntersect(const Vector3 &aabb_min, const Vector3 &aabb_max) const
{
    unsigned int ret =  INTERSECTION_TYPE::INSIDE;

    Vector3 vMin;
    Vector3 vMax;

    // X axis
    if(GetNormal().x > 0)
    {
        vMin.x = aabb_min.x;
        vMax.x = aabb_max.x;
    }
    else
    {
        vMin.x = aabb_max.x;
        vMax.x = aabb_min.x;
    }
    // Y axis
    if(GetNormal().y > 0)
    {
        vMin.y = aabb_min.y;
        vMax.y = aabb_max.y;
    }
    else
    {
        vMin.y = aabb_max.y;
        vMax.y = aabb_min.y;
    }
    // Z axis
    if(GetNormal().z > 0)
    {
        vMin.z = aabb_min.z;
        vMax.z = aabb_max.z;
    }
    else
    {
        vMin.z = aabb_max.z;
        vMax.z = aabb_min.z;
    }

    if( vMin.Dot(GetNormal()) - GetOffset() >  0)  return INTERSECTION_TYPE::OUTSIDE;
    if( vMax.Dot(GetNormal()) - GetOffset() >= 0) ret = INTERSECTION_TYPE::INTERSECT;

    return ret;
}



}


