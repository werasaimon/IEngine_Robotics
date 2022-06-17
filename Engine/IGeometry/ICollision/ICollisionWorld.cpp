#include "ICollisionWorld.h"


namespace IEngine
{

// Constructor
ICollisionWorld::ICollisionWorld()
 : mCollisionDetection(),
   mCurrentBodyID(0)
{

}

// Destructor
ICollisionWorld::~ICollisionWorld()
{
    // Destroy all the collision bodies that have not been removed
    for (auto itBodies = mBodies.begin(); itBodies != mBodies.end(); )
    {
        std::set<ICollisionBody*>::iterator itToRemove = itBodies;
        ++itBodies;
        destroyCollisionBody(*itToRemove);
    }


    assert(mBodies.empty());
}

// Create a collision body and add it to the world
/**
 * @param transform Transformation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
ICollisionBody* ICollisionWorld::createCollisionBody(const Transform &transform)
{
    // Get the next available body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<bodyindex>::max());

    // Create the collision body
    ICollisionBody* collisionBody = new ICollisionBody( transform, &mCollisionDetection , bodyID);

    assert(collisionBody != NULL);

    // Add the collision body to the world
    mBodies.insert(collisionBody);

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
/**
 * @param collisionBody Pointer to the body to destroy
 */
void ICollisionWorld::destroyCollisionBody(ICollisionBody* collisionBody)
{

    // Remove all the collision shapes of the body
    collisionBody->RemoveAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(collisionBody->GetID());

    // Call the destructor of the collision body
    delete collisionBody;

    // Remove the collision body from the list of bodies
    mBodies.erase(collisionBody);

}

// Return the next available body ID
IContactManager *ICollisionWorld::GetCollisionDetection()
{
    return &mCollisionDetection;
}

bodyindex ICollisionWorld::computeNextAvailableBodyID()
{

    // Compute the body ID
    bodyindex bodyID;
    if (!mFreeBodiesIDs.empty())
    {
        bodyID = mFreeBodiesIDs.back();
        mFreeBodiesIDs.pop_back();
    }
    else
    {
        bodyID = mCurrentBodyID;
        mCurrentBodyID++;
    }

    return bodyID;
}



// Reset all the contact manifolds linked list of each body
void ICollisionWorld::resetContactManifoldListsOfBodies()
{
    // For each rigid body of the world
    for (auto it = mBodies.begin(); it != mBodies.end(); ++it)
    {
        // Reset the contact manifold list of the body
        (*it)->ResetContactManifoldsList();
    }
}


// Update collision tohet all bodies
void ICollisionWorld::FindNewContacts()
{

    resetContactManifoldListsOfBodies();

    /**
    for( auto itBodies = mBodies.begin(); itBodies != mBodies.end(); ++itBodies )
    {
        (*itBodies)->updateBroadPhaseState();
    }
    /**/



    mCollisionDetection.FindNewContacts();



    /********************************************************************
    for( auto pair : mCollisionDetection.mContactOverlappingPairs )
    {
        const int NbSize =  pair.second->getContactManifoldSet().getNbContactManifolds();

        for (int i = 0; i < NbSize; ++i)
        {
            const rpContactManifold* maniflod = pair.second->getContactManifoldSet().getContactManifold(i);
        }
    }
    /********************************************************************/

}


// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *                                    bodies to be raycasted
 */
void ICollisionWorld::raycast(const IRay& ray, IRaycastCallback* raycastCallback , unsigned short raycastWithCategoryMaskBits) const
{
    mCollisionDetection.Raycast(raycastCallback, ray , raycastWithCategoryMaskBits);
}


}
