#ifndef ICONTACTMANIFOLD_H
#define ICONTACTMANIFOLD_H


#include "IContactPoint.h"
#include "../../ICollision/IProxyShape.h"


namespace IEngine
{


class IContactManifold
{

    private:

        //-------------------- Attributes --------------------//

        /// Pointer to the first proxy shape of the contact
        IProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        IProxyShape* mShape2;


        /// Number of contacts in the cache
        u32 mNbContactPoints;


        /// Contact points in the manifold
        IContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];


        /// Normal direction Id (Unique Id representing the normal direction)
        short int mNormalDirectionId;


        /// First new the contact info manifold
        bool mIsNewContactInfoManiflod;


        /// First friction vector of the contact manifold
        Vector3 mFrictionVector1;

        /// Second friction vector of the contact manifold
        Vector3 mFrictionVector2;



        /// First friction constraint accumulated impulse
        scalar  mAccumulatedFrictionImpulse1;

        /// Second friction constraint accumulated impulse
        scalar  mAccumulatedFrictionImpulse2;

        /// Twist friction constraint accumulated impulse
        scalar  mAccumulatedFrictionTwistImpulse;

        /// Accumulated rolling resistance impulse
        Vector3 mAccumulatedRollingResistanceImpulse;



        /// True if the contact manifold has already been added into an island
        bool mIsAlreadyInIsland;

        /// Extremal Penetration
        scalar mExtremalPenetration;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IContactManifold(const IContactManifold& contactManifold);

        /// Private assignment operator
        IContactManifold& operator=(const IContactManifold& contactManifold);

        /// Return the index of maximum area
        u32 GetMaxArea(scalar area0, scalar area1, scalar area2, scalar area3) const;

        /// Return the index of the contact with the larger penetration depth.
        u32 GetIndexOfDeepestPenetration(IContactPoint* newContact) const;

        /// Return the index that will be removed.
        u32 GetIndexToRemove(u32 indexMaxPenetration, const Vector3& newPoint) const;

        /// Remove a contact point from the manifold
        void removeContactPoint(u32 index);

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
         IContactManifold( IProxyShape* shape1, IProxyShape* shape2 , short int normalDirectionId = 0);

        /// Destructor
        ~IContactManifold();


        /// Return a pointer to the first body of the contact manifold
        ICollisionBody* GetBody1() const;

        /// Return a pointer to the second body of the contact manifold
        ICollisionBody* GetBody2() const;


        /// Return a pointer to the first proxy shape of the contact manifold
        IProxyShape* GetShape1() const;

        /// Return a pointer to the second proxy shape of the contact manifold
        IProxyShape* GetShape2() const;




        /// Return the normal direction Id
        short int GetNormalDirectionId() const;

        /// Add a contact point to the manifold
        void AddContactPoint(IContactPoint* contact);


        /// Update the contact manifold.
        void Update(const Transform& transform1, const Transform& transform2);

        /// Delete the contact manifold.
        void Update_delete_not_uset_contact();

        /// Clear the contact manifold
        void Clear();

        /// Return the number of contact points in the manifold
        u32 GetNbContactPoints() const;

        /// Return the valid contact info manifold
        bool GetIsNewContactInfoManiflod() const;

        /// Set the valid contact info manifold
        void SetIsNewContactInfoManiflod(bool isNewContactInfoManiflod);

        /// Return the first friction vector at the center of the contact manifold
        const Vector3& GetFrictionVector1() const;

        /// Set the first friction vector at the center of the contact manifold
        void SetFrictionVector1(const Vector3& mFrictionVector1);

        /// Return the second friction vector at the center of the contact manifold
        const Vector3& GetFrictionVector2() const;

        /// Set the second friction vector at the center of the contact manifold
        void SetFrictionVector2(const Vector3& mFrictionVector2);





        /// Return the first friction accumulated impulse
        scalar GetAccumulatedFrictionImpulse1() const;

        /// Set the first friction accumulated impulse
        void SetAccumulatedFrictionImpulse1(scalar frictionImpulse1);



        /// Return the second friction accumulated impulse
        scalar GetAccumulatedFrictionImpulse2() const;

        /// Set the second friction accumulated impulse
        void SetAccumulatedFrictionImpulse2(scalar frictionImpulse2);



        /// Return the friction twist accumulated impulse
        scalar GetAccumulatedFrictionTwistImpulse() const;

        /// Set the friction twist accumulated impulse
        void SetAccumulatedFrictionTwistImpulse(scalar frictionTwistImpulse);



        /// Set the accumulated rolling resistance impulse
        Vector3 GetAccumulatedRollingResistanceImpulse() const;


        /// Set the accumulated rolling resistance impulse
        void SetAccumulatedRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);




        /// Set the accumulated rolling resistance split impulse
        Vector3 GetAccumulatedRollingResistanceSplitImpulse() const;


        /// Set the accumulated rolling resistance split impulse
        void SetAccumulatedRollingResistanceSplitImpulse(const Vector3& rollingResistanceImpulse);




        /// Return a contact point of the manifold
        IContactPoint* GetContactPoint(u32 index) const;

        /// Return the normalized averaged normal vector
        Vector3 GetAverageContactNormal() const;

        /// Return the largest depth of all the contact points
        scalar GetLargestContactDepth() const;





        // -------------------- Friendship -------------------- //
        friend class IContactManifoldSet;
        friend class IDynamicsWorld;
        friend class IIsland;
        friend class ICollisionBody;

};


// Return the valid contact info manifold
SIMD_INLINE bool IContactManifold::GetIsNewContactInfoManiflod() const
{
    return mIsNewContactInfoManiflod;
}


// Set the valid contact info manifold
SIMD_INLINE void IContactManifold::SetIsNewContactInfoManiflod(bool isNewContactInfoManiflod)
{
    mIsNewContactInfoManiflod = isNewContactInfoManiflod;
}


// Return a pointer to the first proxy shape of the contact
SIMD_INLINE IProxyShape *IContactManifold::GetShape1() const
{
    return mShape1;
}

// Return a pointer to the second proxy shape of the contact
SIMD_INLINE IProxyShape *IContactManifold::GetShape2() const
{
    return mShape2;
}

// Return a pointer to the first body of the contact manifold
SIMD_INLINE ICollisionBody* IContactManifold::GetBody1() const
{
    return mShape1->GetBody();
}

// Return a pointer to the second body of the contact manifold
SIMD_INLINE ICollisionBody* IContactManifold::GetBody2() const
{
    return mShape2->GetBody();
}




// Return the normal direction Id
SIMD_INLINE short int IContactManifold::GetNormalDirectionId() const
{
    return mNormalDirectionId;
}

// Return the number of contact points in the manifold
SIMD_INLINE u32 IContactManifold::GetNbContactPoints() const
{
    return mNbContactPoints;
}




// Return the first friction vector at the center of the contact manifold
SIMD_INLINE const Vector3& IContactManifold::GetFrictionVector1() const
{
    return mFrictionVector1;
}

// Set the first friction vector at the center of the contact manifold
SIMD_INLINE void IContactManifold::SetFrictionVector1(const Vector3& frictionVector1)
{
    mFrictionVector1 = frictionVector1;
}


// Return the second friction vector at the center of the contact manifold
SIMD_INLINE const Vector3& IContactManifold::GetFrictionVector2() const
{
    return mFrictionVector2;
}

// Set the second friction vector at the center of the contact manifold
SIMD_INLINE void IContactManifold::SetFrictionVector2(const Vector3& frictionVector2)
{
    mFrictionVector2 = frictionVector2;
}




// Return the first friction accumulated impulse
SIMD_INLINE scalar IContactManifold::GetAccumulatedFrictionImpulse1() const
{
    return mAccumulatedFrictionImpulse1;
}

// Set the first friction accumulated impulse
SIMD_INLINE void IContactManifold::SetAccumulatedFrictionImpulse1(scalar frictionImpulse1)
{
    mAccumulatedFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
SIMD_INLINE scalar IContactManifold::GetAccumulatedFrictionImpulse2() const
{
    return mAccumulatedFrictionImpulse2;
}

// Set the second friction accumulated impulse
SIMD_INLINE void IContactManifold::SetAccumulatedFrictionImpulse2(scalar frictionImpulse2)
{
    mAccumulatedFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
SIMD_INLINE scalar IContactManifold::GetAccumulatedFrictionTwistImpulse() const
{
    return mAccumulatedFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
SIMD_INLINE void IContactManifold::SetAccumulatedFrictionTwistImpulse(scalar frictionTwistImpulse)
{
    mAccumulatedFrictionTwistImpulse = frictionTwistImpulse;
}

// Return accumulated rolling resistance impulse
SIMD_INLINE Vector3 IContactManifold::GetAccumulatedRollingResistanceImpulse() const
{
    return mAccumulatedRollingResistanceImpulse;
}

// Set the accumulated rolling resistance impulse
SIMD_INLINE void IContactManifold::SetAccumulatedRollingResistanceImpulse(const Vector3& rollingResistanceImpulse)
{
    mAccumulatedRollingResistanceImpulse = rollingResistanceImpulse;
}


// Return a contact point of the manifold
SIMD_INLINE IContactPoint* IContactManifold::GetContactPoint(u32 index) const
{
    assert(index < mNbContactPoints);
    return mContactPoints[index];
}

// Return true if the contact manifold has already been added into an island
SIMD_INLINE bool IContactManifold::isAlreadyInIsland() const
{
    return mIsAlreadyInIsland;
}

// Return the normalized averaged normal vector
SIMD_INLINE Vector3 IContactManifold::GetAverageContactNormal() const
{
    Vector3 averageNormal;

    for (u32 i=0; i<mNbContactPoints; i++)
    {
        averageNormal += mContactPoints[i]->GetNormal();
    }

    return averageNormal.GetUnit();
}

// Return the largest depth of all the contact points
SIMD_INLINE scalar IContactManifold::GetLargestContactDepth() const
{
    scalar largestDepth = 0.0f;

    for (u32 i=0; i<mNbContactPoints; i++)
    {
        scalar depth = mContactPoints[i]->GetPenetration();
        if (depth > largestDepth)
        {
            largestDepth = depth;
        }
    }

    return largestDepth;
}


}

#endif // ICONTACTMANIFOLD_H
