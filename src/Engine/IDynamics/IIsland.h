#ifndef IISLAND_H
#define IISLAND_H


#include "IRigidBody.h"
#include "IJoints/IJoint.h"
#include "../IGeometry/ICollision/ICollisionContact/IContactManifold.h"
#include "../IGeometry/ICollision/ICollisionContact/IContactManifoldSet.h"
#include "../IGeometry/ICollision/IOverlappingPair.h"



namespace IEngine
{

class IIsland
{

  private:

     //-------------------- Attributes -----------------//

     /// Array with all the bodies of the island
     IRigidBody** mBodies;

     /// Current number of bodies in the island
     u32 mNbBodies;



     /// Array with all the contact manifolds between bodies of the island
     IContactManifold** mContactManifolds;

     /// Current number of contact manifold in the island
     u32 mNbContactManifolds;



     /// Array with all the joints between bodies of the island
     IJoint** mJoints;

     /// Current number of joints in the island
     u32 mNbJoints;

     //-------------------- Methods -------------------//

     /// Private assignment operator
     IIsland& operator=(const IIsland& island);

     /// Private copy-constructor
     IIsland(const IIsland& island);



  public:


    //-------------------- Methods --------------------//

      /// Constructor
      IIsland(u32 nbMaxBodies , u32 nbMaxContactManifolds , u32 nbMaxJoints );

     /// Destructor
     ~IIsland();



     /// Add a body into the island
     void AddBody(IRigidBody* body);

     /// Add a contact manifold into the island
     void AddContactManifold(IContactManifold* contactManifold);

     /// Add a joint into the island
     void AddJoint(IJoint* joint);





     /// Return the number of bodies in the island
     u32 GetNbBodies() const;

     /// Return the number of contact manifolds in the island
     u32 GetNbContactManifolds() const;

     /// Return the number of joints in the island
     u32 GetNbJoints() const;


     /// Return a pointer to the array of bodies
     IRigidBody** GetBodies();

     /// Return a pointer to the array of contact manifolds
     IContactManifold** GetContactManifold();

     /// Return a pointer to the array of joints
     IJoint** GetJoints();



     //-------------------- Friendship --------------------//
     friend class IDynamicsWorld;

};


// Add a body into the island
SIMD_INLINE void IIsland::AddBody(IRigidBody* body)
{
    assert(!body->IsSleeping());
    mBodies[mNbBodies] = body;
    mNbBodies++;
}

// Add a contact manifold into the island
SIMD_INLINE void IIsland::AddContactManifold(IContactManifold* contactManifold)
{
    mContactManifolds[mNbContactManifolds] = contactManifold;
    mNbContactManifolds++;
}

SIMD_INLINE void IIsland::AddJoint(IJoint *joint)
{
    mJoints[mNbJoints] = joint;
    mNbJoints++;
}


// Return the number of bodies in the island
SIMD_INLINE u32 IIsland::GetNbBodies() const
{
    return mNbBodies;
}

// Return the number of contact manifolds in the island
SIMD_INLINE u32 IIsland::GetNbContactManifolds() const
{
    return mNbContactManifolds;
}


// Return a pointer to the array of bodies
SIMD_INLINE IRigidBody** IIsland::GetBodies()
{
    return mBodies;
}

// Return a pointer to the array of contact manifolds
SIMD_INLINE IContactManifold** IIsland::GetContactManifold()
{
    return mContactManifolds;
}


// Return the number of joints in the island
SIMD_INLINE  u32 IIsland::GetNbJoints() const
{
   return mNbJoints;
}


// Return a pointer to the array of joints
SIMD_INLINE  IJoint** IIsland::GetJoints()
{
   return mJoints;
}

}

#endif // IISLAND_H
