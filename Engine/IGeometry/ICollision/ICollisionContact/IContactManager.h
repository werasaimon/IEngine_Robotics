#ifndef ICONTACTMANAGER_H
#define ICONTACTMANAGER_H

// Libraries
#include <map>
#include <set>

#include "IContactManifold.h"
#include "IContactManifoldSet.h"
#include "../IOverlappingPair.h"
#include "../../ICollision/IBroadPhase/IBroadPhase.h"
#include "IContactGenerator.h"

//// Declarations
//class IBroadPhase;
//class ICollisionWorld;


namespace IEngine
{


// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class IContactManager : public AbstractBroadPhaseNotifyOverlappingPair
{

    private :


        // -------------------- Attributes -------------------- //

        /// Delete is overlapping pairs
        std::map<overlappingpairid, bool> mPairsIsRemoves;

        /// Set of pair of bodies that cannot collide between each other
        std::set<bodyindexpair>                        mNoCollisionPairs;

        /// Broad-phase overlapping pairs
        std::map<overlappingpairid, IOverlappingPair*> mOverlappingPairs;

        /// Real overlapping pairs
        std::map<overlappingpairid, IOverlappingPair*> mContactOverlappingPairs;

        /// Broad-phase algorithm
        IBroadPhase                                    mBroadPhaseAlgorithm;


        /// True if some collision shapes have been added previously
        bool mIsCollisionShapesAdded;



        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IContactManager(const IContactManager& collisionDetection);

        /// Private assignment operator
        IContactManager& operator=(const IContactManager& collisionDetection);

        /// Compute the broad-phase collision detection
        void ComputeBroadPhase();

        /// Compute the narrow-phase collision detection
        void ComputeNarrowPhase();



         /// Compute the collision contact
        void ComputeContacts(const IProxyShape* ,const IProxyShape* , IOverlappingPair* ,IContactVertex* , u32 , bool);


        /// Add all the contact manifold of colliding pairs to their bodies
        void AddAllContactManifoldsToBodies();


        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void AddContactManifoldToBody(IOverlappingPair* pair);


        void BroadPhaseNotifyOverlappingPair(void *_shape1 ,
                                             void *_shape2 );


    public :



        // -------------------- Methods -------------------- //

        /// Constructor
         IContactManager();

        /// Destructor
        ~IContactManager();



        /// Compute find contacts
        void FindNewContacts();



        /// Create point
        void CreateContact(IOverlappingPair* overlappingPair , IContactPoint *contact);

        /// Delete all the contact points in the currently overlapping pairs
        void ClearContactPoints();


        /// Add a proxy collision shape to the collision detection
        void AddProxyCollisionShape(IProxyShape* proxyShape, const IAABBox3D& aabb);

        /// Remove a proxy collision shape from the collision detection
        void RemoveProxyCollisionShape(IProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void UpdateProxyCollisionShape(IProxyShape* shape, const IAABBox3D& aabb,  const Vector3& displacement = Vector3(0, 0, 0) , bool forceReinsert = false );

        /// Add a pair of bodies that cannot collide with each other
        void AddNoCollisionPair(ICollisionBody* body1, ICollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void RemoveNoCollisionPair(ICollisionBody* body1, ICollisionBody* body2);

        /// Ask for a collision shape to be tested again during broad-phase.
        void AskForBroadPhaseCollisionCheck(IProxyShape* shape);

        /// Ray casting method
        void Raycast(IRaycastCallback* raycastCallback, const IRay& ray, unsigned short raycastWithCategoryMaskBits) const;

        /// All collision pairs
        std::map<overlappingpairid, IOverlappingPair*> ContactOverlappingPairs() const;



        // -------------------- Friendships -------------------- //

        friend class IDynamicsWorld;
        friend class ICollisionWorld;
        friend class IConvexShape;
        friend class IBroadPhase;


};




}

#endif // ICONTACTMANAGER_H
