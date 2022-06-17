#ifndef IPROXYSHAPE_H
#define IPROXYSHAPE_H

#include "ICollisionShapes/ICollisionShape.h"
#include "IBody/ICollisionBody.h"
#include "../../ICommon/ISettings.h"

namespace IEngine
{

// Class ProxyShape
/**
 * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
 * a unique instance of SphereShape but we need to differentiate between the two instances during
 * the collision detection. They do not have the same position in the world and they do not
 * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
 * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
 * each collision shape attached to the body).
 */
class IProxyShape
{

    protected:

        // -------------------- Attributes -------------------- //

        /// Pointer to the parent body
        ICollisionBody*   mBody;

        /// Internal collision shape
        ICollisionShape*  mCollisionShape;

        /// Local-space to parent body-space transform (does not change over time)
        Transform        mLocalToBodyTransform;

        /// Mass (in kilogramms) of the corresponding collision shape
        scalar            mMass;

        /// Pointer to the next proxy shape of the body (linked list)
        IProxyShape*      mNext;

        /// Broad-phase ID (node ID in the dynamic AABB tree)
        i32               mBroadPhaseID;

        /// Cached collision data
        void*             mCachedCollisionData;

        /// Pointer to user data
        void*             mUserData;




        /// Bits used to define the collision category of this shape.
        /// You can set a single bit to one to define a category value for this
        /// shape. This value is one (0x0001) by default. This variable can be used
        /// together with the mCollideWithMaskBits variable so that given
        /// categories of shapes collide with each other and do not collide with
        /// other categories.
        unsigned short mCollisionCategoryBits;

        /// Bits mask used to state which collision categories this shape can
        /// collide with. This value is 0xFFFF by default. It means that this
        /// proxy shape will collide with every collision categories by default.
        unsigned short mCollideWithMaskBits;




        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IProxyShape(const IProxyShape& proxyShape);

        /// Private assignment operator
        IProxyShape& operator=(const IProxyShape& proxyShape);




    public:


        // -------------------- Methods -------------------- //

        /// Constructor
        IProxyShape(ICollisionBody* body , ICollisionShape* shape , const Transform& transform, scalar mass=0 );

        /// Destructor
        virtual ~IProxyShape();

        /// Return the collision shape
        const ICollisionShape* GetCollisionShape() const;

        /// Return the parent body
        ICollisionBody* GetBody() const;

        /// Return the mass of the collision shape
        scalar GetMass() const;

        /// Return a pointer to the user data attached to this body
        void* GetUserData() const;

        /// Attach user data to this body
        void SetUserData(void* userData);


        /// Return the local to parent body transform
        const Transform& GetLocalToBodyTransform() const;

        /// Set the local to parent body transform
        void SetLocalToBodyTransform(const Transform& transform);

        /// Return the local to world transform
        const Transform GetWorldTransform() const;


        /// Return true if a point is inside the collision shape
        bool TestPointInside(const Vector3& worldPoint);

        /// Raycast method with feedback information
        bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo , bool is_location_inversion = false);


        /// Return the collision bits mask
        unsigned short GetCollideWithMaskBits() const;

        /// Set the collision bits mask
        void SetCollideWithMaskBits(unsigned short collideWithMaskBits);

        /// Return the collision category bits
        unsigned short GetCollisionCategoryBits() const;

        /// Set the collision category bits
        void SetCollisionCategoryBits(unsigned short collisionCategoryBits);

        /// Return the next proxy shape in the linked list of proxy shapes
        IProxyShape* GetNext();

        /// Return the next proxy shape in the linked list of proxy shapes
        const IProxyShape* GetNext() const;

        /// Return the pointer to the cached collision data
        void** GetCachedCollisionData();

        /// Return the local scaling vector of the collision shape
        Vector3 GetLocalScaling() const;

        /// Set the local scaling vector of the collision shape
        virtual void SetLocalScaling(const Vector3& scaling);



        // -------------------- Friendship -------------------- //
        friend class IRigidBody;
        friend class IOverlappingPair;
        friend class ICollisionBody;
        friend class IBroadPhase;
        friend class IDynamicAABBTree;
        friend class IContactManager;
        friend class ICollisionWorld;
        friend class IDynamicsWorld;
        friend class IConvexMeshShape;

};

// Return the pointer to the cached collision data
SIMD_INLINE void** IProxyShape::GetCachedCollisionData()
{
    return &mCachedCollisionData;
}

// Return the collision shape
/**
 * @return Pointer to the internal collision shape
 */
SIMD_INLINE const ICollisionShape* IProxyShape::GetCollisionShape() const
{
    return mCollisionShape;
}



// Return the parent body
/**
 * @return Pointer to the parent body
 */
SIMD_INLINE ICollisionBody* IProxyShape::GetBody() const
{
    return mBody;
}



// Return the mass of the collision shape
/**
 * @return Mass of the collision shape (in kilograms)
 */
SIMD_INLINE scalar IProxyShape::GetMass() const
{
    return mMass;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data stored into the proxy shape
 */
SIMD_INLINE void* IProxyShape::GetUserData() const
{
    return mUserData;
}

// Attach user data to this body
/**
 * @param userData Pointer to the user data you want to store within the proxy shape
 */
SIMD_INLINE void IProxyShape::SetUserData(void* userData)
{
    mUserData = userData;
}

// Return the local to parent body transform
/**
 * @return The transformation that transforms the local-space of the collision shape
 *         to the local-space of the parent body
 */
SIMD_INLINE const Transform& IProxyShape::GetLocalToBodyTransform() const
{
    return mLocalToBodyTransform;
}


// Set the local to parent body transform
SIMD_INLINE void IProxyShape::SetLocalToBodyTransform(const Transform& transform)
{
    mLocalToBodyTransform = transform;
}



// Return the local to world transform
/**
 * @return The transformation that transforms the local-space of the collision
 *         shape to the world-space
 */
SIMD_INLINE const Transform IProxyShape::GetWorldTransform() const
{
    return mBody->mTransform * mLocalToBodyTransform;
}



// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
SIMD_INLINE IProxyShape* IProxyShape::GetNext()
{
    return mNext;
}

// Return the next proxy shape in the linked list of proxy shapes
/**
 * @return Pointer to the next proxy shape in the linked list of proxy shapes
 */
SIMD_INLINE const IProxyShape* IProxyShape::GetNext() const
{
    return mNext;
}

// Return the collision category bits
/**
 * @return The collision category bits mask of the proxy shape
 */
SIMD_INLINE unsigned short IProxyShape::GetCollisionCategoryBits() const
{
    return mCollisionCategoryBits;
}

// Set the collision category bits
/**
 * @param collisionCategoryBits The collision category bits mask of the proxy shape
 */
SIMD_INLINE void IProxyShape::SetCollisionCategoryBits(unsigned short collisionCategoryBits)
{
    mCollisionCategoryBits = collisionCategoryBits;
}

// Return the collision bits mask
/**
 * @return The bits mask that specifies with which collision category this shape will collide
 */
SIMD_INLINE unsigned short IProxyShape::GetCollideWithMaskBits() const
{
    return mCollideWithMaskBits;
}

// Set the collision bits mask
/**
 * @param collideWithMaskBits The bits mask that specifies with which collision category this shape will collide
 */
SIMD_INLINE void IProxyShape::SetCollideWithMaskBits(unsigned short collideWithMaskBits)
{
    mCollideWithMaskBits = collideWithMaskBits;
}

// Return the local scaling vector of the collision shape
/**
 * @return The local scaling vector
 */
SIMD_INLINE Vector3 IProxyShape::GetLocalScaling() const
{
    return mCollisionShape->GetScaling();
}



// Set the local scaling vector of the collision shape
/**
 * @param scaling The new local scaling vector
 */
SIMD_INLINE void IProxyShape::SetLocalScaling(const Vector3& scaling)
{
    // Set the local scaling of the collision shape
    mCollisionShape->SetLocalScaling(scaling);
}

}

#endif // IPROXYSHAPE_H
