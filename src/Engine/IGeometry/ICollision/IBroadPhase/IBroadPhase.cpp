#include "IBroadPhase.h"

#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <cstring>

#include "../../../ICommon/IMemory/IMem.h"
#include "../IProxyShape.h"

namespace IEngine
{

using namespace IMath;

// Constructor
IBroadPhase::IBroadPhase(AbstractBroadPhaseNotifyOverlappingPair* collisionDetection)
    :mDynamicAABBTree(DYNAMIC_TREE_AABB_GAP),
     mNbMovedShapes(0),
     mNbAllocatedMovedShapes(8),
     mNbNonUsedMovedShapes(0),
     mNbPotentialPairs(0),
     mNbAllocatedPotentialPairs(8),
     mContactManager(collisionDetection)
{

    // Allocate memory for the array of non-static proxy shapes IDs
    mMovedShapes = (i32*) IAlloc(mNbAllocatedMovedShapes * sizeof(i32));
    assert(mMovedShapes != NULL);

    // Allocate memory for the array of potential overlapping pairs
    mPotentialPairs = (IBroadPhasePair*) IAlloc(mNbAllocatedPotentialPairs * sizeof(IBroadPhasePair));
    assert(mPotentialPairs != NULL);
}

// Destructor
IBroadPhase::~IBroadPhase()
{
    // Release the memory for the array of non-static proxy shapes IDs
    IFree(mMovedShapes);

    // Release the memory for the array of potential overlapping pairs
    IFree(mPotentialPairs);
}



// Add a collision shape in the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
void IBroadPhase::AddMovedCollisionShape(i32 broadPhaseID)
{
    // Allocate more elements in the array of shapes that have moved if necessary
    if (mNbAllocatedMovedShapes == mNbMovedShapes)
    {
        mNbAllocatedMovedShapes *= 2;
        i32* oldArray = mMovedShapes;
        mMovedShapes = (i32*) IAlloc(mNbAllocatedMovedShapes * sizeof(i32));
        assert(mMovedShapes != NULL);
        memcpy(mMovedShapes, oldArray, mNbMovedShapes * sizeof(i32));
        IFree(oldArray);
    }

    // Store the broad-phase ID into the array of shapes that have moved
    assert(mNbMovedShapes < mNbAllocatedMovedShapes);
    assert(mMovedShapes != NULL);
    mMovedShapes[mNbMovedShapes] = broadPhaseID;
    mNbMovedShapes++;
}

// Remove a collision shape from the array of shapes that have moved in the last simulation step
// and that need to be tested again for broad-phase overlapping.
void IBroadPhase::RemoveMovedCollisionShape(i32 broadPhaseID)
{
    //assert(mNbNonUsedMovedShapes <= mNbMovedShapes);

    // If less than the quarter of allocated elements of the non-static shapes IDs array
    // are used, we release some allocated memory
    if ((mNbMovedShapes - mNbNonUsedMovedShapes) < mNbAllocatedMovedShapes / 4 && mNbAllocatedMovedShapes > 8)
    {

        mNbAllocatedMovedShapes /= 2;
        i32* oldArray = mMovedShapes;
        mMovedShapes = (i32*) IAlloc(mNbAllocatedMovedShapes * sizeof(i32));
        assert(mMovedShapes != NULL);
        u32 nbElements = 0;
        for (u32 i=0; i<mNbMovedShapes; i++)
        {
            if (oldArray[i] != -1)
            {
                mMovedShapes[nbElements] = oldArray[i];
                nbElements++;
            }
        }
        mNbMovedShapes = nbElements;
        mNbNonUsedMovedShapes = 0;
        IFree(oldArray);
    }



    // Remove the broad-phase ID from the array
    for (u32 i=0; i<mNbMovedShapes; i++)
    {
        if (mMovedShapes[i] == broadPhaseID)
        {
            mMovedShapes[i] = -1;
            mNbNonUsedMovedShapes++;
            break;
        }
    }
}


// Compute all the overlapping pairs of collision shapes
void IBroadPhase::ComputeOverlappingPairs()
{

    // Reset the potential overlapping pairs
    mNbPotentialPairs = 0;

    // For all collision shapes that have moved (or have been created) during the
    // last simulation step
    for (u32 i=0; i<mNbMovedShapes; i++)
    {
        i32 shapeID = mMovedShapes[i];

        if (shapeID == -1) continue;

        IAABBOverlapCallback callback(*this, shapeID);

        // Get the AABB of the shape
        const IAABBox3D& shapeAABB = mDynamicAABBTree.GetFatAABB(shapeID);

        // Ask the dynamic AABB tree to report all collision shapes that overlap with
        // this AABB. The method BroadPhase::notifiyOverlappingPair() will be called
        // by the dynamic AABB tree for each potential overlapping pair.
        mDynamicAABBTree.ReportAllShapesOverlappingWithAABB(shapeAABB, callback);
    }

    // Reset the array of collision shapes that have move (or have been created) during the
    // last simulation step
    mNbMovedShapes = 0;

    // Sort the array of potential overlapping pairs in order to remove duplicate pairs
    std::sort(mPotentialPairs, mPotentialPairs + mNbPotentialPairs, IBroadPhasePair::SmallerThan);


    // Check all the potential overlapping pairs avoiding duplicates to report unique
    // overlapping pairs
    u32 i=0;
    while (i < mNbPotentialPairs)
    {

        // Get a potential overlapping pair
        IBroadPhasePair* pair = mPotentialPairs + i;
        i++;

        assert(pair->collisionShape1ID != pair->collisionShape2ID);

        // Get the two collision shapes of the pair
        void* shape1 = (mDynamicAABBTree.GetNodeDataPointer(pair->collisionShape1ID));
        void* shape2 = (mDynamicAABBTree.GetNodeDataPointer(pair->collisionShape2ID));

        // Notify the collision detection about the overlapping pair
        mContactManager->BroadPhaseNotifyOverlappingPair( shape1 , shape2 );

        // Skip the duplicate overlapping pairs
        while (i < mNbPotentialPairs)
        {

            // Get the next pair
            IBroadPhasePair* nextPair = mPotentialPairs + i;

            // If the next pair is different from the previous one, we stop skipping pairs
            if (nextPair->collisionShape1ID != pair->collisionShape1ID ||
                nextPair->collisionShape2ID != pair->collisionShape2ID)
            {
                break;
            }
            i++;
        }
    }

    // If the number of potential overlapping pairs is less than the quarter of allocated
    // number of overlapping pairs
    if (mNbPotentialPairs < mNbAllocatedPotentialPairs / 4 && mNbPotentialPairs > 8)
    {
        // Reduce the number of allocated potential overlapping pairs
        IBroadPhasePair* oldPairs = mPotentialPairs;
        mNbAllocatedPotentialPairs /= 2;
        mPotentialPairs = (IBroadPhasePair*) IAlloc(mNbAllocatedPotentialPairs * sizeof(IBroadPhasePair));
        assert(mPotentialPairs);
        memcpy(mPotentialPairs, oldPairs, mNbPotentialPairs * sizeof(IBroadPhasePair));
        IFree(oldPairs);
    }
}

// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
void IBroadPhase::NotifyOverlappingNodes(i32 node1ID, i32 node2ID)
{

    // If both the nodes are the same, we do not create store the overlapping pair
    if (node1ID == node2ID) return;

    // If we need to allocate more memory for the array of potential overlapping pairs
    if (mNbPotentialPairs == mNbAllocatedPotentialPairs)
    {
        // Allocate more memory for the array of potential pairs
        IBroadPhasePair* oldPairs = mPotentialPairs;
        mNbAllocatedPotentialPairs *= 2;
        mPotentialPairs = (IBroadPhasePair*) IAlloc(mNbAllocatedPotentialPairs * sizeof(IBroadPhasePair));
        assert(mPotentialPairs);
        memcpy(mPotentialPairs, oldPairs, mNbPotentialPairs * sizeof(IBroadPhasePair));
        IFree(oldPairs);
    }

    // Add the new potential pair into the array of potential overlapping pairs
    mPotentialPairs[mNbPotentialPairs].collisionShape1ID = IMin(node1ID, node2ID);
    mPotentialPairs[mNbPotentialPairs].collisionShape2ID = IMax(node1ID, node2ID);
    mNbPotentialPairs++;
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
void IAABBOverlapCallback::NotifyOverlappingNode(i32 nodeId)
{
    mBroadPhaseAlgorithm.NotifyOverlappingNodes(mReferenceNodeId, nodeId);
}

// Called for a broad-phase shape that has to be tested for raycast
scalar IBroadPhaseRaycastCallback::RaycastBroadPhaseShape(i32 nodeId, const IRay& ray)
{
        scalar hitFraction = scalar(-1.0);

        // Get the proxy shape from the node
        IProxyShape* proxyShape = static_cast<IProxyShape*>(mDynamicAABBTree.GetNodeDataPointer(nodeId));

        // Check if the raycast filtering mask allows raycast against this shape
        //if ((mRaycastWithCategoryMaskBits != 0) || true  /*&& proxyShape->getCollisionCategoryBits()*/ )
        if ((mRaycastWithCategoryMaskBits & proxyShape->GetCollisionCategoryBits()) != 0)
        {

            // Ask the collision detection to perform a ray cast test against
            // the proxy shape of this node because the ray is overlapping
            // with the shape in the broad-phase
            hitFraction = mRaycastTest.raycastAgainstShape(proxyShape, ray);
        }

        return hitFraction;
}

}
