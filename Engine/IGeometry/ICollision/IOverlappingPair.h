#ifndef IOVERLAPPINGPAIR_H
#define IOVERLAPPINGPAIR_H

#include "ICollisionContact/IContactManifoldSet.h"
#include "../ICollision/IProxyShape.h"
#include <vector>



namespace IEngine
{

// Type for the overlapping pair ID
typedef std::pair<u32, u32> overlappingpairid;

/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class IOverlappingPair
{

    private:

       //bool isFakeCollision;

        // -------------------- Attributes -------------------- //

        /// Set of persistent contact manifolds
        IContactManifoldSet mContactManifoldSet;

        IProxyShape* mShape1;
        IProxyShape* mShape2;


        /// Cached previous separating axis
        Vector3 mCachedSeparatingAxis;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IOverlappingPair(const IOverlappingPair& pair);

        /// Private assignment operator
        IOverlappingPair& operator=(const IOverlappingPair& pair);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IOverlappingPair( IProxyShape* shape1, IProxyShape* shape2 , i32 nbMaxContactManifolds = 1);

        /// Destructor
        ~IOverlappingPair();

        /// Return the pointer to first proxy collision shape
        IProxyShape* GetShape1() const;

        /// Return the pointer to second body
        IProxyShape* GetShape2() const;


        /// Return the cached separating axis
        Vector3 GetCachedSeparatingAxis() const;

        /// Set the cached separating axis
        void SetCachedSeparatingAxis(const Vector3& axis);



        /// Add a contact to the contact cache
        void AddContact(IContactPoint* contact);

        /// Update the contact cache
        void Update();

        /// Delete the contact cache.
        void Update_delete_not_uset_contact();


        /// Clear the contact points of the contact manifold
        void ClearContactPoints();


        /// Return the number of contacts in the cache
        i32 GetNbContactManifolds() const;

        /// Return the a reference to the contact manifold set
        const IContactManifoldSet& GetContactManifoldSet() const;


        /// Return the a contact points
        std::vector<IContactPoint> GetContactPoints()
        {
            std::vector<IContactPoint> contacts(0);
            for ( u32  i = 0; i < mContactManifoldSet.GetNbContactManifolds(); ++i)
            {
                 IContactManifold* manifold = mContactManifoldSet.GetContactManifold(i);
                 for ( u32 j = 0; j < manifold->GetNbContactPoints(); ++j)
                 {
                     contacts.push_back( *manifold->GetContactPoint(j) );
                 }
            }
            return contacts;
        }

//        /// Return the a contact manifolds
//        std::vector<rpContactManifold*> GetContactManifolds()
//        {
//           std::vector<rpContactManifold*> manifolds(0);
//           for ( u32  i = 0; i < mContactManifoldSet.getNbContactManifolds(); ++i)
//           {
//              manifolds.push_back(mContactManifoldSet.getContactManifold(i));
//           }
//           return manifolds;
//        }


        /// Return the pair of bodies index
        static overlappingpairid ComputeID(IProxyShape* shape1,
                                           IProxyShape* shape2);

        /// Return the pair of bodies index of the pair
        static bodyindexpair ComputeBodiesIndexPair( ICollisionBody* body1,
                                                     ICollisionBody* body2);

        // -------------------- Friendship -------------------- //

        friend class IContactManager;
        friend class ICollisionWorld;
        friend class IDynamicsWorld;


};

// Return the pointer to first body
SIMD_INLINE  IProxyShape* IOverlappingPair::GetShape1() const
{
    return mShape1;
}
// Return the pointer to first body
SIMD_INLINE  IProxyShape* IOverlappingPair::GetShape2() const
{
    return mShape2;
}

// Return the cached separating axis
SIMD_INLINE  Vector3 IOverlappingPair::GetCachedSeparatingAxis() const
{
    return mCachedSeparatingAxis;
}

// Set the cached separating axis
SIMD_INLINE  void IOverlappingPair::SetCachedSeparatingAxis(const Vector3& axis)
{
    mCachedSeparatingAxis = axis;
}



SIMD_INLINE i32 IOverlappingPair::GetNbContactManifolds() const
{
    return mContactManifoldSet.GetNbContactManifolds();
}

SIMD_INLINE const IContactManifoldSet& IOverlappingPair::GetContactManifoldSet() const
{
     return mContactManifoldSet;
}



// Return the pair of bodies index
SIMD_INLINE  bodyindexpair IOverlappingPair::ComputeBodiesIndexPair(ICollisionBody* body1,
                                                                    ICollisionBody* body2)
{
    // Construct the pair of body index
    bodyindexpair indexPair =    body1->GetID() < body2->GetID() ?
                                 std::make_pair(body1->GetID(), body2->GetID()) :
                                 std::make_pair(body2->GetID(), body1->GetID());
    assert(indexPair.first != indexPair.second);
    return indexPair;
}


// Return the pair of bodies index
SIMD_INLINE   overlappingpairid IOverlappingPair::ComputeID(IProxyShape* shape1,
                                                            IProxyShape* shape2)
{

    assert(shape1->mBroadPhaseID >= 0 && shape2->mBroadPhaseID >= 0);

    // Construct the pair of body index
    overlappingpairid pairID =  shape1->mBroadPhaseID < shape2->mBroadPhaseID ?
                                std::make_pair(shape1->mBroadPhaseID, shape2->mBroadPhaseID) :
                                std::make_pair(shape2->mBroadPhaseID, shape1->mBroadPhaseID);
    assert(pairID.first != pairID.second);
    return pairID;
}






}

#endif // IOVERLAPPINGPAIR_H
