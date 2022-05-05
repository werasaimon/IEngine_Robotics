#ifndef ICOLLISIONSHAPEBOX_H
#define ICOLLISIONSHAPEBOX_H

#include "ICollisionShapeConvex.h"

namespace IEngine
{


// Class BoxShape
/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body will give an orientation and a position to the box. This
 * collision shape uses an extra margin distance around it for collision
 * detection purpose. The default margin is 4cm (if your units are meters,
 * which is recommended). In case, you want to simulate small objects
 * (smaller than the margin distance), you might want to reduce the margin by
 * specifying your own margin distance using the "margin" parameter in the
 * constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 */
    class ICollisionShapeBox: public ICollisionShapeConvex
    {

    protected :

        // -------------------- Attributes -------------------- //

        /// Extent sizes of the box in the x, y and z direction
        Vector3 mExtent;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionShapeBox(const ICollisionShapeBox& shape);

        /// Private assignment operator
        ICollisionShapeBox& operator=(const ICollisionShapeBox& shape);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 GetLocalSupportPointWithMargin(const Vector3& direction ) const;

        /// Return true if a point is inside the collision shape
        virtual bool TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t GetSizeInBytes() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionShapeBox(const Vector3& extent );


        /// Constructor
        ICollisionShapeBox(const Vector3& extent, scalar margin );

        /// Destructor
        virtual ~ICollisionShapeBox();

        /// Return the extents of the box
        Vector3 GetExtent() const;

        /// Set the scaling vector of the collision shape
        virtual void SetLocalScaling(const Vector3& scaling);

        /// Return the local bounds of the shape in x, y and z directions
        virtual void GetLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        //virtual void ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const;
        Matrix3 ComputeLocalInertiaTensor2( scalar mass , const Matrix3& transform ) const;
    };

    // Return the extents of the box
    /**
     * @return The vector with the three extents of the box shape (in meters)
     */
    SIMD_INLINE Vector3 ICollisionShapeBox::GetExtent() const
    {
        return mExtent + Vector3(mMargin, mMargin, mMargin);
    }

    // Set the scaling vector of the collision shape
    SIMD_INLINE void ICollisionShapeBox::SetLocalScaling(const Vector3& scaling)
    {

        //mExtent = (mExtent / mScaling) * scaling;


        ICollisionShape::SetLocalScaling(scaling);
    }

    // Return the local bounds of the shape in x, y and z directions
    /// This method is used to compute the AABB of the box
    /**
     * @param min The minimum bounds of the shape in local-space coordinates
     * @param max The maximum bounds of the shape in local-space coordinates
     */
    SIMD_INLINE void ICollisionShapeBox::GetLocalBounds(Vector3& min, Vector3& max) const
    {

        // Maximum bounds
        max = mExtent + Vector3(mMargin, mMargin, mMargin);

        // Minimum bounds
        min = -max;
    }

    // Return the number of bytes used by the collision shape
    SIMD_INLINE size_t ICollisionShapeBox::GetSizeInBytes() const
    {
        return sizeof(ICollisionShapeBox);
    }

    // Return a local support point in a given direction without the objec margin
    SIMD_INLINE Vector3 ICollisionShapeBox::GetLocalSupportPointWithMargin(const Vector3& direction) const
    {
        return Vector3(direction.x < 0.0f ? -mExtent.x : mExtent.x,
                        direction.y < 0.0f ? -mExtent.y : mExtent.y,
                        direction.z < 0.0f ? -mExtent.z : mExtent.z);

    }


    // Return true if a point is inside the collision shape
    SIMD_INLINE bool ICollisionShapeBox::TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const
    {
        return (localPoint.x < mExtent[0] && localPoint.x > -mExtent[0] &&
                localPoint.y < mExtent[1] && localPoint.y > -mExtent[1] &&
                localPoint.z < mExtent[2] && localPoint.z > -mExtent[2]);
    }

}


#endif // ICOLLISIONSHAPEBOX_H
