
#include "IAABBox3D.h"


#include "../IGeometry/Segments/IPlane.h"


namespace IEngine
{


using namespace IMath;


const IAABBox3D IAABBox3D::Empty = IAABBox3D(false);
const IAABBox3D IAABBox3D::Zero  = IAABBox3D(0);
const IAABBox3D IAABBox3D::UnitPositive = IAABBox3D(1);




IAABBox3D::IAABBox3D()
{
    mMin = Vector3( 1,  1,  1);
    mMax = Vector3(-1, -1, -1);
}

IAABBox3D::IAABBox3D(scalar xmin, scalar ymin, scalar zmin, scalar xmax, scalar ymax, scalar zmax)
{
    mMin = Vector3(xmin, ymin, zmin);
    mMax = Vector3(xmax, ymax, zmax);
}

IAABBox3D::IAABBox3D(scalar fCubeSize)
{
    mMin = Vector3(0, 0, 0);
    mMax = Vector3(fCubeSize, fCubeSize, fCubeSize);
}

IAABBox3D::IAABBox3D(scalar fWidth, scalar fHeight, scalar fDepth)
{
    mMin = Vector3(0, 0, 0);
    mMax = Vector3(fWidth, fHeight, fDepth);
}

IAABBox3D::IAABBox3D(Vector3 &vMin, Vector3 &vMax)
{
    mMin = Vector3(IMin(vMin.x, vMax.x), IMin(vMin.y, vMax.y), IMin(vMin.z, vMax.z));
    mMax = Vector3(IMax(vMin.x, vMax.x), IMax(vMin.y, vMax.y), IMax(vMin.z, vMax.z));
}

IAABBox3D::IAABBox3D(const Vector3 &vMin, const Vector3 &vMax)
{
    mMin = Vector3(IMin(vMin.x, vMax.x), IMin(vMin.y, vMax.y), IMin(vMin.z, vMax.z));
    mMax = Vector3(IMax(vMin.x, vMax.x), IMax(vMin.y, vMax.y), IMax(vMin.z, vMax.z));
}

IAABBox3D::IAABBox3D(const Vector3 &vCenter, scalar fHalfWidth, scalar fHalfHeight, scalar fHalfDepth)
{
    mMin = Vector3(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight, vCenter.z - fHalfDepth);
    mMax = Vector3(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight, vCenter.z + fHalfDepth);
}

IAABBox3D::IAABBox3D(Vector3 &vCenter, scalar fHalfWidth, scalar fHalfHeight, scalar fHalfDepth)
{
    mMin = Vector3(vCenter.x - fHalfWidth, vCenter.y - fHalfHeight, vCenter.z - fHalfDepth);
    mMax = Vector3(vCenter.x + fHalfWidth, vCenter.y + fHalfHeight, vCenter.z + fHalfDepth);
}

IAABBox3D::IAABBox3D(const Vector3 &vCenter, scalar fHalfSize)
{
    mMin = Vector3(vCenter.x - fHalfSize, vCenter.y - fHalfSize, vCenter.z - fHalfSize);
    mMax = Vector3(vCenter.x + fHalfSize, vCenter.y + fHalfSize, vCenter.z + fHalfSize);
}

void IAABBox3D::MergeTwoAABBs(const IAABBox3D &aabb1, const IAABBox3D &aabb2)
{
    mMin.x = IMin(aabb1.mMin.x, aabb2.mMin.x);
    mMin.y = IMin(aabb1.mMin.y, aabb2.mMin.y);
    mMin.z = IMin(aabb1.mMin.z, aabb2.mMin.z);

    mMax.x = IMax(aabb1.mMax.x, aabb2.mMax.x);
    mMax.y = IMax(aabb1.mMax.y, aabb2.mMax.y);
    mMax.z = IMax(aabb1.mMax.z, aabb2.mMax.z);
}

IAABBox3D IAABBox3D::CreateAABBForTriangle(const Vector3 *trianglePoints)
{

    Vector3 minCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);
    Vector3 maxCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);

    if (trianglePoints[1].x < minCoords.x) minCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y < minCoords.y) minCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z < minCoords.z) minCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x < minCoords.x) minCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y < minCoords.y) minCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z < minCoords.z) minCoords.z = trianglePoints[2].z;

    if (trianglePoints[1].x > maxCoords.x) maxCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y > maxCoords.y) maxCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z > maxCoords.z) maxCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x > maxCoords.x) maxCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y > maxCoords.y) maxCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z > maxCoords.z) maxCoords.z = trianglePoints[2].z;

    return  IAABBox3D(minCoords, maxCoords);
}



bool IAABBox3D::TestRayIntersect(const IRay &_ray , const Matrix4& transform , IRaycast* _raycast) const
{

    IRay local_ray( _ray.Origin    * transform.GetInverse(),
                    _ray.Direction * transform.GetRotMatrix().GetInverse());

    Vector3 T_1, T_2; // vectors to hold the T-values for every direction
    double t_near = -__DBL_MAX__; // maximums defined in float.h
    double t_far = __DBL_MAX__;


    for (int i = 0; i < 3; i++)
    {
        //we test slabs in every direction
        if (local_ray.Direction[i] == 0)
        { // ray parallel to planes in this direction
            if ((local_ray.Origin[i] < mMin[i]) || (local_ray.Origin[i] > mMax[i]))
            {
                return false; // parallel AND outside box : no intersection possible
            }
        }
        else
        { // ray not parallel to planes in this direction
            T_1[i] = (mMin[i] - local_ray.Origin[i]) / local_ray.Direction[i];
            T_2[i] = (mMax[i] - local_ray.Origin[i]) / local_ray.Direction[i];

            if(T_1[i] > T_2[i])
            {
                // we want T_1 to hold values for intersection with near plane
                std::swap(T_1,T_2);
            }

            if (T_1[i] > t_near)
            {
                t_near = T_1[i];
            }
            if (T_2[i] < t_far)
            {
                t_far = T_2[i];
            }

            if( (t_near > t_far) || (t_far < 0) )
            {
                return false;
            }
        }
    }

    if(_raycast)
    {
        _raycast->distance = t_near;
    }

    return true;


    //    const Vector3 point2 = local_ray.Origin + local_ray.maxFraction * (local_ray.Direction);
    //    const Vector3 e = mMax - mMin;
    //    const Vector3 d = point2 - ray.Origin;
    //    const Vector3 m = local_ray.Origin + point2 - mMin - mMax;

//    const Vector3 point2 = local_ray.Origin + local_ray.maxFraction * (local_ray.Direction - local_ray.Origin);
//    const Vector3 e = mMax - mMin;
//    const Vector3 d = point2 - local_ray.Origin;
//    const Vector3 m = local_ray.Origin + point2 - mMin - mMax;

//    // Test if the AABB face normals are separating axis
//    scalar adx = IAbs(d.x);
//    if (IAbs(m.x) > e.x + adx) return false;
//    scalar ady = IAbs(d.y);
//    if (IAbs(m.y) > e.y + ady) return false;
//    scalar adz = IAbs(d.z);
//    if (IAbs(m.z) > e.z + adz) return false;

//    // Add in an epsilon term to counteract arithmetic errors when segment is
//    // (near) parallel to a coordinate axis (see text for detail)
//    const scalar epsilon = 0.00001f;
//    adx += epsilon;
//    ady += epsilon;
//    adz += epsilon;

//    // Test if the cross products between face normals and ray direction are
//    // separating axis
//    if (IAbs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return false;
//    if (IAbs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return false;
//    if (IAbs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return false;

//    // No separating axis has been found
//    return true;
}


bool IAABBox3D::TestCollision(const IAABBox3D &aabb) const
{
    if (mMax.x < aabb.mMin.x || aabb.mMax.x < mMin.x) return false;
    if (mMax.y < aabb.mMin.y || aabb.mMax.y < mMin.y) return false;
    if (mMax.z < aabb.mMin.z || aabb.mMax.z < mMin.z) return false;
    return true;
}

scalar IAABBox3D::GetVolume() const
{
    const Vector3 diff = mMax - mMin;
    return (diff.x * diff.y * diff.z);
}

Vector3 IAABBox3D::GetMax() const
{
    return mMax;
}

Vector3 IAABBox3D::GetMin() const
{
    return mMin;
}

void IAABBox3D::SetMin(const Vector3 &min)
{
    mMin = min;
}

void IAABBox3D::SetMax(const Vector3 &max)
{
    mMax = max;
}

Vector3 IAABBox3D::HalfSize() const
{
    Vector3 size;
    for (std::size_t i = 0; i < Vector3::components; ++i)
    {
        size[i] = IAbs(mMax[i] - mMin[i]);
    }
    return size;
}

IAABBox3D::IAABBox3D(const Vector3 &vCenter)
{
    mMin = mMax = vCenter;
}

scalar IAABBox3D::Width() const
{
    return IMax(mMax.x - mMin.x, scalar(0));
}

scalar IAABBox3D::Height() const
{
    return IMax(mMax.y - mMin.y, scalar(0));
}

scalar IAABBox3D::Depth() const
{
    return IMax(mMax.z - mMin.z, scalar(0));
}

scalar IAABBox3D::Volume() const
{
    return Width() * Height() * Depth();
}

scalar IAABBox3D::DiagonalLength() const
{
    return ISqrt((mMax.x - mMin.x) * (mMax.x - mMin.x) +
                     (mMax.y - mMin.y) * (mMax.y - mMin.y) +
                     (mMax.z - mMin.z) * (mMax.z - mMin.z));
}

scalar IAABBox3D::MaxDim() const
{
    return IMax(Width(), IMax(Height(), Depth()));
}

Vector3 IAABBox3D::Diagonal() const
{
    return Vector3(mMax.x - mMin.x, mMax.y - mMin.y, mMax.z-mMin.z);
}

Vector3 IAABBox3D::Extents() const
{
    return Vector3((mMax.x - mMin.x)*scalar(0.5),
                   (mMax.y - mMin.y)*scalar(0.5),
                   (mMax.z - mMin.z)*scalar(0.5));
}

Vector3 IAABBox3D::Center() const
{
    return Vector3(scalar(0.5) * (mMin.x + mMax.x),
                   scalar(0.5) * (mMin.y + mMax.y),
                   scalar(0.5) * (mMin.z + mMax.z));
}

bool IAABBox3D::operator ==(const IAABBox3D &b) const
{
    return mMin == b.mMin && mMax == b.mMax;
}

bool IAABBox3D::operator !=(const IAABBox3D &b)
{
    return mMin != b.mMin || mMax != b.mMax;
}

bool IAABBox3D::Equals(IAABBox3D other)
{
    return this->mMin == other.mMin &&
            this->mMax == other.mMax;
}

int IAABBox3D::GetHashCode() const
{
    // Overflow is fine, just wrap
    int hash = (int) 2166136261;
    hash = (hash * 16777619) ^ mMin.GetHashCode();
    hash = (hash * 16777619) ^ mMax.GetHashCode();
    return hash;
}

Vector3 IAABBox3D::Corner(int i) const
{
    scalar x = (  ((i&1) != 0) ^ ((i&2) != 0) ) ? (mMax.x) : (mMin.x);
    scalar y = ( (i / 2) % 2 == 0 ) ? (mMin.y) : (mMax.y);
    scalar z = (i < 4) ? (mMin.z) : (mMax.z);
    return Vector3(x, y, z);
}

Vector3 IAABBox3D::Point(int xi, int yi, int zi) const
{
    scalar x = (xi < 0) ? mMin.x : ((xi == 0) ? scalar(0.5*(mMin.x + mMax.x)) : mMax.x);
    scalar y = (yi < 0) ? mMin.y : ((yi == 0) ? scalar(0.5*(mMin.y + mMax.y)) : mMax.y);
    scalar z = (zi < 0) ? mMin.z : ((zi == 0) ? scalar(0.5*(mMin.z + mMax.z)) : mMax.z);
    return Vector3(x, y, z);
}

void IAABBox3D::Expand(scalar fRadius)
{
    mMin.x -= fRadius; mMin.y -= fRadius; mMin.z -= fRadius;
    mMax.x += fRadius; mMax.y += fRadius; mMax.z += fRadius;
}

IAABBox3D IAABBox3D::Expanded(scalar fRadius) const
{
    return IAABBox3D(
                mMin.x - fRadius, mMin.y - fRadius, mMin.z - fRadius,
                mMax.x + fRadius, mMax.y + fRadius, mMax.z + fRadius);
}

void IAABBox3D::Contract(scalar fRadius) const
{
    scalar w = 2 * fRadius;
    if ( w > mMax.x-mMin.x )
    {
        mMin.x = mMax.x = scalar(0.5) * (mMin.x + mMax.x);
    }
    else
    {
        mMin.x += fRadius; mMax.x -= fRadius;
    }
    if ( w > mMax.y-mMin.y )
    {
        mMin.y = mMax.y = scalar(0.5) * (mMin.y + mMax.y);
    }
    else
    {
        mMin.y += fRadius; mMax.y -= fRadius;
    }
    if ( w > mMax.z-mMin.z )
    {
        mMin.z = mMax.z = scalar(0.5) * (mMin.z + mMax.z);
    }
    else
    {
        mMin.z += fRadius; mMax.z -= fRadius;
    }
}

IAABBox3D IAABBox3D::Contracted(scalar fRadius) const
{
    IAABBox3D result = IAABBox3D(
                mMin.x + fRadius, mMin.y + fRadius, mMin.z + fRadius,
                mMax.x - fRadius, mMax.y - fRadius, mMax.z - fRadius);

    if (result.mMin.x > result.mMax.x) { result.mMin.x = result.mMax.x = scalar(0.5) * (mMin.x + mMax.x); }
    if (result.mMin.y > result.mMax.y) { result.mMin.y = result.mMax.y = scalar(0.5) * (mMin.y + mMax.y); }
    if (result.mMin.z > result.mMax.z) { result.mMin.z = result.mMax.z = scalar(0.5) * (mMin.z + mMax.z); }

    return result;
}

void IAABBox3D::Scale(scalar sx, scalar sy, scalar sz)
{
    Vector3 c = Center();
    Vector3 e = Extents(); e.x *= sx; e.y *= sy; e.z *= sz;
    mMin = Vector3(c.x - e.x, c.y - e.y, c.z - e.z);
    mMax = Vector3(c.x + e.x, c.y + e.y, c.z + e.z);
}

void IAABBox3D::MergeWithAABB(Vector3 &v)
{
    mMin.x = IMin(mMin.x, v.x);
    mMin.y = IMin(mMin.y, v.y);
    mMin.z = IMin(mMin.z, v.z);
    mMax.x = IMax(mMax.x, v.x);
    mMax.y = IMax(mMax.y, v.y);
    mMax.z = IMax(mMax.z, v.z);
}

void IAABBox3D::MergeWithAABB(const IAABBox3D &box)
{
    mMin.x = IMin(mMin.x, box.mMin.x);
    mMin.y = IMin(mMin.y, box.mMin.y);
    mMin.z = IMin(mMin.z, box.mMin.z);
    mMax.x = IMax(mMax.x, box.mMax.x);
    mMax.y = IMax(mMax.y, box.mMax.y);
    mMax.z = IMax(mMax.z, box.mMax.z);
}

void IAABBox3D::ContainsTwoAABBs(const IAABBox3D &aabb1, const IAABBox3D &aabb2)
{
    mMin.x = IMin(aabb1.mMin.x, aabb2.mMin.x);
    mMin.y = IMin(aabb1.mMin.y, aabb2.mMin.y);
    mMin.z = IMin(aabb1.mMin.z, aabb2.mMin.z);

    mMax.x = IMax(aabb1.mMax.x, aabb2.mMax.x);
    mMax.y = IMax(aabb1.mMax.y, aabb2.mMax.y);
    mMax.z = IMax(aabb1.mMax.z, aabb2.mMax.z);
}

IAABBox3D IAABBox3D::Intersect(const IAABBox3D &box) const
{
    IAABBox3D intersect = IAABBox3D(
                IMax(mMin.x, box.mMin.x), IMax(mMin.y, box.mMin.y), IMax(mMin.z, box.mMin.z),
                IMin(mMax.x, box.mMax.x), IMin(mMax.y, box.mMax.y), IMin(mMax.z, box.mMax.z));
    if (intersect.Height() <= 0 || intersect.Width() <= 0 || intersect.Depth() <= 0)
    {
        return IAABBox3D(false); // Empty;
    }
    else
    {
        return intersect;
    }
}

bool IAABBox3D::TestCollisionTriangleAABB(const Vector3 *trianglePoints) const
{

    if (IMin3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > mMax.x) return false;
    if (IMin3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > mMax.y) return false;
    if (IMin3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > mMax.z) return false;

    if (IMax3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < mMin.x) return false;
    if (IMax3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < mMin.y) return false;
    if (IMax3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < mMin.z) return false;

    return true;
}

bool IAABBox3D::Contains(Vector3 &v) const
{
    return     (mMin.x <= v.x) && (mMin.y <= v.y) && (mMin.z <= v.z)
            && (mMax.x >= v.x) && (mMax.y >= v.y) && (mMax.z >= v.z);
}

bool IAABBox3D::Contains(const Vector3 &v) const
{
    return (mMin.x <= v.x) && (mMin.y <= v.y) && (mMin.z <= v.z) &&
            (mMax.x >= v.x) && (mMax.y >= v.y) && (mMax.z >= v.z);
}

bool IAABBox3D::Contains(IAABBox3D box2) const
{
    return  Contains(box2.mMin) &&
            Contains(box2.mMax);
}

bool IAABBox3D::Contains(const IAABBox3D &box2) const
{
    return Contains(box2.mMin) &&
           Contains(box2.mMax);
}

bool IAABBox3D::Containsss(const IAABBox3D &box2)
{
    return Contains(box2.mMin) &&
           Contains(box2.mMax);
}

bool IAABBox3D::Intersects(const IAABBox3D &box) const
{
    return !((box.mMax.x <= mMin.x) || (box.mMin.x >= mMax.x)
             || (box.mMax.y <= mMin.y) || (box.mMin.y >= mMax.y)
             || (box.mMax.z <= mMin.z) || (box.mMin.z >= mMax.z) );
}

bool IAABBox3D::IntersectsRay(IRay &r)
{

    scalar enter = scalar(0.0f);
    scalar exit  = scalar(1.0f);

    scalar tx1 = (mMin[0] - r.Origin[0]) / r.Direction[0];
    scalar tx2 = (mMax[0] - r.Origin[0]) / r.Direction[0];

    scalar tmin = IMin(tx1, tx2);
    scalar tmax = IMax(tx1, tx2);

    for (std::size_t i = 0; i < Vector3::components; ++i)
    {
        //if(IAbs(r.direction[i]) < T(0.00001) ) continue;


        if (IAbs(r.Direction[i]) < scalar(0.0001))
        {
            // The segment is parallel to slab. No hit if origin not within slab.
            if (r.Origin[i] < mMin[i] || r.Origin[i] > mMax[i])
            {
                return false;
            }
        }
        else
        {
            scalar tx1 = (mMin[i] - r.Origin[i]) / r.Direction[i];
            scalar tx2 = (mMax[i] - r.Origin[i]) / r.Direction[i];

            // Make t1 be intersection with near plane, t2 with far plane.
            if (tx1 > tx2)
            {
                ISwap(tx1, tx2);
            }

            tmin = IMax(tmin, IMin(tx1, tx2));
            tmax = IMin(tmax, IMax(tx1, tx2));

            if(tmin > tmax)
            {
                //ISwap(tmin,tmax);
                return false;
            }

            //Reduce interval based on intersection
            if(tmin > enter) enter = tmin;
            if(tmax < exit)  exit  = tmax;
        }
    }

    r.maxFraction = enter;

    return true;
}

scalar IAABBox3D::DistanceSquared(const Vector3 &v) const
{
    scalar dx = (v.x < mMin.x) ? mMin.x - v.x : (v.x > mMax.x ? v.x - mMax.x : 0);
    scalar dy = (v.y < mMin.y) ? mMin.y - v.y : (v.y > mMax.y ? v.y - mMax.y : 0);
    scalar dz = (v.z < mMin.z) ? mMin.z - v.z : (v.z > mMax.z ? v.z - mMax.z : 0);
    return dx * dx + dy * dy + dz * dz;
}

scalar IAABBox3D::Distance(const Vector3 &v) const
{
    return ISqrt(DistanceSquared(v));
}

scalar IAABBox3D::SignedDistance(const Vector3 &v) const
{
    if ( Contains(v) == false ) {
        return Distance(v);
    } else {
        scalar dx = IMin(IAbs(v.x - mMin.x), IAbs(v.x - mMax.x));
        scalar dy = IMin(IAbs(v.y - mMin.y), IAbs(v.y - mMax.y));
        scalar dz = IMin(IAbs(v.z - mMin.z), IAbs(v.z - mMax.z));
        return -IMin(dx, dy, dz);
    }
}

scalar IAABBox3D::DistanceSquared(const IAABBox3D &box2) const
{
    // compute lensqr( max(0, abs(center1-center2) - (extent1+extent2)) )
    scalar delta_x = IAbs((box2.mMin.x + box2.mMax.x) - (mMin.x + mMax.x)) -
            ((mMax.x - mMin.x) + (box2.mMax.x - box2.mMin.x));
    if ( delta_x < 0 )
        delta_x = 0;
    scalar delta_y = IAbs((box2.mMin.y + box2.mMax.y) - (mMin.y + mMax.y)) -
            ((mMax.y - mMin.y) + (box2.mMax.y - box2.mMin.y));
    if (delta_y < 0)
        delta_y = 0;
    scalar delta_z = IAbs((box2.mMin.z + box2.mMax.z) - (mMin.z + mMax.z)) -
            ((mMax.z - mMin.z) + (box2.mMax.z - box2.mMin.z));
    if (delta_z < 0)
        delta_z = 0;

    return scalar(0.25) * scalar(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
}

IAABBox3D IAABBox3D::GetAxisAlignedBoxTransform(const Matrix4 &_TransformMatrix) const
{
    IAABBox3D box;

    // Get the local bounds in x,y and z direction
    Vector3 minBounds = mMin;
    Vector3 maxBounds = mMax;

    // Scale size AABB local
    minBounds *= scalar(1.04);
    maxBounds *= scalar(1.04);


    // Rotate the local bounds according to the orientation of the body
    Matrix3 worldAxis =  (_TransformMatrix.GetRotMatrix().GetInverse().GetTranspose()).GetAbsoluteMatrix();

    Vector3   worldMinBounds(worldAxis.GetColumn(0).Dot(minBounds),
                             worldAxis.GetColumn(1).Dot(minBounds),
                             worldAxis.GetColumn(2).Dot(minBounds));

    Vector3   worldMaxBounds(worldAxis.GetColumn(0).Dot(maxBounds),
                             worldAxis.GetColumn(1).Dot(maxBounds),
                             worldAxis.GetColumn(2).Dot(maxBounds));


    // Compute the minimum and maximum coordinates of the rotated extents
    Vector3 minCoordinates = _TransformMatrix.GetTranslation() + worldMinBounds;
    Vector3 maxCoordinates = _TransformMatrix.GetTranslation() + worldMaxBounds;


    // Update the AABB with the new minimum and maximum coordinates
    return IAABBox3D(minCoordinates , maxCoordinates);

}

void IAABBox3D::Translate(const Vector3 &vTranslate)
{
    mMin += vTranslate;
    mMax += vTranslate;
}

void IAABBox3D::MoveMin(const Vector3 &vNewMin)
{
    mMax.x = vNewMin.x + (mMax.x - mMin.x);
    mMax.y = vNewMin.y + (mMax.y - mMin.y);
    mMax.z = vNewMin.z + (mMax.z - mMin.z);
    mMin.SetAllValues(vNewMin.x,vNewMin.y,vNewMin.z);
}

void IAABBox3D::MoveMin(scalar fNewX, scalar fNewY, scalar fNewZ)
{
    mMax.x = fNewX + (mMax.x - mMin.x);
    mMax.y = fNewY + (mMax.y - mMin.y);
    mMax.z = fNewZ + (mMax.z - mMin.z);
    mMin.SetAllValues(fNewX, fNewY, fNewZ);
}

void IAABBox3D::Insert(const Vector3 &point)
{
    for (std::size_t i = 0; i < Vector3::components; ++i)
    {
        if (mMin[i] > point[i]) mMin[i] = point[i];
        if (mMax[i] < point[i]) mMax[i] = point[i];
    }
}

void IAABBox3D::Insert(const IAABBox3D &aabb)
{
    for (std::size_t i = 0; i < Vector3::components; ++i)
    {
        if (mMin[i] > aabb.mMin[i]) mMin[i] = aabb.mMin[i];
        if (mMax[i] < aabb.mMax[i]) mMax[i] = aabb.mMax[i];
    }
}

void IAABBox3D::Repair()
{
    for (std::size_t i = 0; i < Vector3::components; ++i)
    {
        if (mMin[i] > mMax[i])
        {
            ISwap(mMin[i], mMax[i]);
        }
    }
}


}














