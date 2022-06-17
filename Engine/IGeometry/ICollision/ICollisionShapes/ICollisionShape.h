#ifndef ICOLLISIONSHAPE_H
#define ICOLLISIONSHAPE_H

#include "../../IAABBox3D.h"
#include "../IRaycastInfo.h"

namespace IEngine
{

using namespace  IMath;

class IOverlappingPair;

/// Type of the collision shape
enum CollisionShapeType {TRIANGLE,
                         BOX,
                         SPHERE,
                         CONE,
                         CYLINDER,
                         CAPSULE,
                         CONVEX_MESH ,
                         CONVEX_HULL_MESH ,
                         CONCAVE_MESH,
                         CONCAVE_SHAPE,
                         HEIGHTFIELD,
                         TRIANGLE_MESH};


//Extern declarations
class IProxyShape;

// Class CollisionShape
/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ICollisionShape
{

    protected :


        /// Max iterration axis peturbiration
        i32     mNbMaxPeturberationIteration;
        /// Eppsiolon for axis peturbiration
        scalar  mEpsilonPeturberation;

        // -------------------- Attributes ----------------- //

        /// Type of the collision shape
        CollisionShapeType mType;

        /// Scaling vector of the collision shape
        Vector3 mScaling;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionShape(const ICollisionShape& shape);

        /// Private assignment operator
        ICollisionShape& operator=(const ICollisionShape& shape);

        /// Return true if a point is inside the collision shape
        virtual bool TestPointInside(const Vector3& worldPoint, IProxyShape* proxyShape) const = 0;

        /// Raycast method with feedback information
        virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const = 0;

        /// Return the number of bytes used by the collision shape
        virtual size_t GetSizeInBytes() const = 0;




        /// Return a local support point in a given direction with the object margin
        virtual Vector3 GetLocalSupportPointWithMargin(const Vector3& direction) const = 0;


        /// Return a local support point in a given direction without the object margin
        virtual Vector3 GetLocalSupportPointWithoutMargin(const Vector3& direction) const = 0;

    public:


//        virtual ICollisionShape *Clone(){ return nullptr; }


//        virtual void Copy( const ICollisionShape* shape )
//        {
//            /// Max iterration axis peturbiration
//            mNbMaxPeturberationIteration = shape->mNbMaxPeturberationIteration;
//            /// Eppsiolon for axis peturbiration
//            mEpsilonPeturberation = shape->mEpsilonPeturberation;
//        }


        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionShape(CollisionShapeType type);

        /// Destructor
        virtual ~ICollisionShape();

        /// Return the type of the collision shapes
        CollisionShapeType GetType() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool IsConvex() const = 0;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void GetLocalBounds(Vector3& min, Vector3& max) const = 0;

        /// Return the scaling vector of the collision shape
        Vector3 GetScaling() const;

        /// Set the local scaling vector of the collision shape
        virtual void SetLocalScaling(const Vector3& scaling);

        /// Return the local inertia tensor of the collision shapes
        //virtual void ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const = 0;
        virtual Matrix3 ComputeLocalInertiaTensor2( scalar mass , const Matrix3& transform ) const = 0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void ComputeAABB(IAABBox3D& aabb, const Transform& _transform ) const;

        /// Return true if the collision shape type is a convex shape
        static bool IsConvex(CollisionShapeType shapeType);

        /// Return the maximum number of contact manifolds in an overlapping pair given two shape types
        static i32 ComputeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                CollisionShapeType shapeType2);




        //virtual void PairCollisionDetected(IOverlappingPair* collid_info);

        // -------------------- Friendship -------------------- //

        friend class IProxyShape;
        friend class ICollisionWorld;
        friend class ICollisionBody;
        friend class ICollisionShapeInfo;
        friend class IContactGenerator;
};


// Return the type of the collision shape
/**
 * @return The type of the collision shape (box, sphere, cylinder, ...)
 */
SIMD_INLINE CollisionShapeType ICollisionShape::GetType() const
{
    return mType;
}

// Return true if the collision shape type is a convex shape
SIMD_INLINE bool ICollisionShape::IsConvex(CollisionShapeType shapeType)
{
    return shapeType != CONCAVE_MESH && shapeType != HEIGHTFIELD;
}

// Return the scaling vector of the collision shape
SIMD_INLINE Vector3 ICollisionShape::GetScaling() const
{
    return mScaling;
}

// Set the scaling vector of the collision shape
SIMD_INLINE void ICollisionShape::SetLocalScaling(const Vector3& scaling)
{
    mScaling = scaling;
}

// Return the maximum number of contact manifolds allowed in an overlapping
// pair wit the given two collision shape types
SIMD_INLINE i32 ICollisionShape::ComputeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                              CollisionShapeType shapeType2)
{
    // If both shapes are convex
    if (IsConvex(shapeType1) && IsConvex(shapeType2))
    {
        return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
    }   // If there is at least one concave shape
    else
    {
        return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
    }
}


}

#endif // ICOLLISIONSHAPE_H
