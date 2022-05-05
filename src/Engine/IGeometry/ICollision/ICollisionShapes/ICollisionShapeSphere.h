#ifndef ICOLLISIONSHAPESPHERE_H
#define ICOLLISIONSHAPESPHERE_H


#include "ICollisionShapeConvex.h"

namespace IEngine
{


// Class SphereShape
/**
 * This class represents a sphere collision shape that is centered
 * at the origin and defined by its radius. This collision shape does not
 * have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, no need to specify an object margin
 * for a sphere shape.
 */
class ICollisionShapeSphere : public ICollisionShapeConvex
{

    protected :


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionShapeSphere(const ICollisionShapeSphere& shape );

        /// Private assignment operator
        ICollisionShapeSphere& operator=(const ICollisionShapeSphere& shape);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 GetLocalSupportPointWithMargin(const Vector3& direction) const;

        /// Return true if a point is inside the collision shape
        virtual bool TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t GetSizeInBytes() const;

    public :


        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionShapeSphere(scalar radius);

        /// Destructor
        virtual ~ICollisionShapeSphere();

        /// Return the radius of the sphere
        scalar GetRadius() const;

        /// Set the scaling vector of the collision shape
        virtual void SetLocalScaling(const Vector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void GetLocalBounds(Vector3& min, Vector3& max) const;

        /// Update the AABB of a body using its collision shape
        void ComputeAABB(IAABBox3D& aabb, const Transform& transform) const;


        /// Return the local inertia tensor of the collision shape
        //virtual void ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const;
        Matrix3 ComputeLocalInertiaTensor2( scalar mass , const Matrix3& transform ) const;
};

// Get the radius of the sphere
/**
 * @return Radius of the sphere (in meters)
 */
SIMD_INLINE scalar ICollisionShapeSphere::GetRadius() const
{
    return mMargin;
}

// Set the scaling vector of the collision shape
SIMD_INLINE void ICollisionShapeSphere::SetLocalScaling(const Vector3& scaling)
{
    mMargin = (mMargin / mScaling.x) * scaling.x;
    ICollisionShape::SetLocalScaling(scaling);
}

// Return the number of bytes used by the collision shape
SIMD_INLINE size_t ICollisionShapeSphere::GetSizeInBytes() const
{
    return sizeof(ICollisionShapeSphere);
}

// Return a local support point in a given direction without the object margin
SIMD_INLINE Vector3 ICollisionShapeSphere::GetLocalSupportPointWithMargin(const Vector3& direction) const
{
//    // Return the center of the sphere (the radius is taken into account in the object margin)
    return  direction.GetUnit() * mMargin;

    // Return the center of the sphere (the radius is taken into account in the object margin)
  //  return Vector3(0.0, 0.0, 0.0);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
SIMD_INLINE void ICollisionShapeSphere::GetLocalBounds(Vector3& min, Vector3& max) const
{

    // Maximum bounds
    max.x = mMargin;
    max.y = mMargin;
    max.z = mMargin;

    // Minimum bounds
    min.x = -mMargin;
    min.y = -mMargin;
    min.z = -mMargin;
}

// Return the local inertia tensor of the sphere
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
//SIMD_INLINE void ICollisionShapeSphere::ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const
//{
//    scalar diag = scalar(0.4) * mass * mMargin * mMargin;

//    float X_scale = diag;
//    float Y_scale = diag;
//    float Z_scale = diag;

//    tensor = Matrix3( X_scale, 0.f, 0.f,
//                         0.f, Y_scale, 0.f,
//                         0.f, 0.f, Z_scale );
//}

SIMD_INLINE Matrix3 ICollisionShapeSphere::ComputeLocalInertiaTensor2(scalar mass, const Matrix3& transform) const
{
        scalar diag = scalar(0.4) * mass * mMargin * mMargin;

        scalar xSquare = diag;
        scalar ySquare = diag;
        scalar zSquare = diag;

        Matrix3 worldAxis = transform;
        xSquare *= worldAxis.GetRow(0).Length();
        ySquare *= worldAxis.GetRow(1).Length();
        zSquare *= worldAxis.GetRow(2).Length();

        Matrix3 tensor( (ySquare + zSquare), 0.0, 0.0,
                        0.0, (xSquare + zSquare), 0.0,
                        0.0, 0.0, (xSquare + ySquare) );
        return tensor;
}


// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
SIMD_INLINE void ICollisionShapeSphere::ComputeAABB(IAABBox3D& aabb, const Transform& transform) const
{
    // Get the local extents in x,y and z direction
    Vector3 extents(mMargin, mMargin, mMargin);

    // Update the AABB with the new minimum and maximum coordinates
    aabb.SetMin(transform.GetPosition() - extents);
    aabb.SetMax(transform.GetPosition() + extents);
}

// Return true if a point is inside the collision shape
SIMD_INLINE bool ICollisionShapeSphere::TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const
{
    return (localPoint.LengthSquare() < mMargin * mMargin);
}


}

#endif // ICOLLISIONSHAPESPHERE_H
