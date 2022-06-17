#ifndef ICOLLISIONWORLD_H
#define ICOLLISIONWORLD_H

// Libraries
#include <vector>
#include <set>
#include <list>
#include <algorithm>

#include "ICollisionContact/IContactManager.h"
#include "IBody/ICollisionBody.h"

namespace IEngine
{

// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class ICollisionWorld
{

    protected:

        //----------------- Attributes ------------------//

        /// Reference to the collision detection
        IContactManager            mCollisionDetection;

        /// All the bodies (rigid and soft) of the world
        std::set<ICollisionBody*>  mBodies;

        /// Current body ID
        bodyindex                  mCurrentBodyID;

        /// List of free ID for rigid bodies
        std::vector<luint>         mFreeBodiesIDs;


        //-------------------- Methods ------------------//

        /// Private copy-constructor
        ICollisionWorld(const ICollisionWorld& world);

        /// Private assignment operator
        ICollisionWorld& operator=(const ICollisionWorld& world);



        /// Return the next available body ID
        bodyindex computeNextAvailableBodyID();

        /// Reset all the contact manifolds linked list of each body
        void resetContactManifoldListsOfBodies();


    public:
        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionWorld();

        /// Destructor
        virtual ~ICollisionWorld();

        /// Return an iterator to the beginning of the bodies of the physics world
        std::set<ICollisionBody*>::iterator getBodiesBeginIterator();

        /// Return an iterator to the end of the bodies of the physics world
        std::set<ICollisionBody*>::iterator getBodiesEndIterator();

        /// Create a collision body
        ICollisionBody* createCollisionBody(const Transform& transform);

        /// Destroy a collision body
        void destroyCollisionBody(ICollisionBody* collisionBody);

        /// Find all collision pairs body
        void FindNewContacts();

        /// Ray cast method
        void raycast(const IRay& ray, IRaycastCallback* raycastCallback , unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;

        /// Contact manager
        IContactManager *GetCollisionDetection();

        /// Add no collision pair
        void addNoCollisionPair(ICollisionBody* body1, ICollisionBody* body2)
        {
            mCollisionDetection.AddNoCollisionPair( body1 , body2 );
        }




        // -------------------- Friendship -------------------- //

        friend class IDynamicsWorld;
        friend class IContactManager;
        friend class ICollisionBody;
        friend class IRigidBody;
        friend class IConvexMeshShape;

};




// Return an iterator to the beginning of the bodies of the physics world
/**
 * @return An starting iterator to the set of bodies of the world
 */
SIMD_INLINE std::set<ICollisionBody*>::iterator ICollisionWorld::getBodiesBeginIterator()
{
    return mBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
/**
 * @return An ending iterator to the set of bodies of the world
 */
SIMD_INLINE std::set<ICollisionBody*>::iterator ICollisionWorld::getBodiesEndIterator()
{
    return mBodies.end();
}


}


#endif // ICOLLISIONWORLD_H
