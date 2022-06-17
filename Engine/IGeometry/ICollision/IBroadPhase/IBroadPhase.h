#ifndef IBROADPHASE_H
#define IBROADPHASE_H

#include "IDynamicAABBTree.h"

namespace IEngine
{

class IBroadPhase;
class AbstractBroadPhaseNotifyOverlappingPair
{
    public:

      virtual void BroadPhaseNotifyOverlappingPair( void* shape1 , void* shape2 ) = 0;
        virtual ~AbstractBroadPhaseNotifyOverlappingPair() {}
};


// Structure BroadPhasePair
/**
 * This structure represent a potential overlapping pair during the
 * broad-phase collision detection.
 */
struct IBroadPhasePair
{
    // -------------------- Attributes -------------------- //

    /// Broad-phase ID of the first collision shape
    i32 collisionShape1ID;

    /// Broad-phase ID of the second collision shape
    i32 collisionShape2ID;

    // -------------------- Methods -------------------- //

    /// Method used to compare two pairs for sorting algorithm
    static bool SmallerThan(const IBroadPhasePair& pair1, const IBroadPhasePair& pair2);
};







// class AABBOverlapCallback
class IAABBOverlapCallback : public IDynamicAABBTreeOverlapCallback
{

    private:

        IBroadPhase& mBroadPhaseAlgorithm;
        i32 mReferenceNodeId;

    public:

        // Constructor
        IAABBOverlapCallback(IBroadPhase& broadPhaseAlgo, i32 referenceNodeId)
         : mBroadPhaseAlgorithm(broadPhaseAlgo),
           mReferenceNodeId(referenceNodeId)
        {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void NotifyOverlappingNode(i32 nodeId);

};




// Class BroadPhaseRaycastCallback
/**
 * Callback called when the AABB of a leaf node is hit by a ray the
 * broad-phase Dynamic AABB Tree.
 */
class IBroadPhaseRaycastCallback : public IDynamicAABBTreeRaycastCallback
{

    private :

        const IDynamicAABBTree& mDynamicAABBTree;
        unsigned short mRaycastWithCategoryMaskBits;
        IRaycastTest& mRaycastTest;

    public:

        // Constructor
        IBroadPhaseRaycastCallback(const IDynamicAABBTree& dynamicAABBTree, unsigned short raycastWithCategoryMaskBits, IRaycastTest& raycastTest)
            : mDynamicAABBTree(dynamicAABBTree),
              mRaycastWithCategoryMaskBits(raycastWithCategoryMaskBits),
              mRaycastTest(raycastTest)
        {

        }

        // Called for a broad-phase shape that has to be tested for raycast
        virtual scalar RaycastBroadPhaseShape(i32 nodeId, const IRay& ray);

};



// Class BroadPhaseAlgorithm
/**
 * This class represents the broad-phase collision detection. The
 * goal of the broad-phase collision detection is to compute the pairs of proxy shapes
 * that have their AABBs overlapping. Only those pairs of bodies will be tested
 * later for collision during the narrow-phase collision detection. A dynamic AABB
 * tree data structure is used for fast broad-phase collision detection.
 */
class IBroadPhase
{

    protected :

        // -------------------- Attributes -------------------- //

        /// Dynamic AABB tree
        IDynamicAABBTree mDynamicAABBTree;

        /// Array with the broad-phase IDs of all collision shapes that have moved (or have been
        /// created) during the last simulation step. Those are the shapes that need to be tested
        /// for overlapping in the next simulation step.
        i32* mMovedShapes;

        /// Number of collision shapes in the array of shapes that have moved during the last
        /// simulation step.
        u32 mNbMovedShapes;

        /// Number of allocated elements for the array of shapes that have moved during the last
        /// simulation step.
        u32 mNbAllocatedMovedShapes;

        /// Number of non-used elements in the array of shapes that have moved during the last
        /// simulation step.
        u32 mNbNonUsedMovedShapes;

        /// Temporary array of potential overlapping pairs (with potential duplicates)
        IBroadPhasePair* mPotentialPairs;

        /// Number of potential overlapping pairs
        u32 mNbPotentialPairs;

        /// Number of allocated elements for the array of potential overlapping pairs
        u32 mNbAllocatedPotentialPairs;

        /// Reference to the collision contact meneger
        AbstractBroadPhaseNotifyOverlappingPair *mContactManager;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IBroadPhase(const IBroadPhase& algorithm);

        /// Private assignment operator
        IBroadPhase& operator=(const IBroadPhase& algorithm);

    public :

        // -------------------- Methods -------------------- //

         /// Constructor
         IBroadPhase(AbstractBroadPhaseNotifyOverlappingPair *collisionDetection);

         /// Destructor
        ~IBroadPhase();



        /// Add a proxy collision shape into the broad-phase collision detection
        template<class T> void AddProxyCollisionShape(T* proxyShape, const IAABBox3D& aabb);

        /// Remove a proxy collision shape from the broad-phase collision detection
        template<class T> void RemoveProxyCollisionShape(T* proxyShape);

        /// Notify the broad-phase that a collision shape has moved and need to be updated
        template<class T> void UpdateProxyCollisionShape(T* proxyShape, const IAABBox3D& aabb, const Vector3& displacement, bool forceReinsert = false);

        /// Add a collision shape in the array of shapes that have moved in the last simulation step
        /// and that need to be tested again for broad-phase overlapping.
        void AddMovedCollisionShape(i32 broadPhaseID);

        /// Remove a collision shape from the array of shapes that have moved in the last simulation
        /// step and that need to be tested again for broad-phase overlapping.
        void RemoveMovedCollisionShape(i32 broadPhaseID);

        /// Notify the broad-phase about a potential overlapping pair in the dynamic AABB tree
        void NotifyOverlappingNodes(i32 broadPhaseId1, i32 broadPhaseId2);

        /// Compute all the overlapping pairs of collision shapes
        void ComputeOverlappingPairs();

        /// Return true if the two broad-phase collision shapes are overlapping
        template<class T> bool TestOverlappingShapes(const T* shape1, const T* shape2) const;

        /// Ray casting method
        void Raycast(const IRay& ray, IRaycastTest& raycastTest , unsigned short raycastWithCategoryMaskBits) const;
};

// Method used to compare two pairs for sorting algorithm
SIMD_INLINE bool IBroadPhasePair::SmallerThan(const IBroadPhasePair& pair1, const IBroadPhasePair& pair2)
{

    if (pair1.collisionShape1ID <  pair2.collisionShape1ID) return true;
    if (pair1.collisionShape1ID == pair2.collisionShape1ID)
    {
        return pair1.collisionShape2ID < pair2.collisionShape2ID;
    }
    return false;
}


// Add a proxy collision shape into the broad-phase collision detection
template<class T>
SIMD_INLINE void IBroadPhase::AddProxyCollisionShape(T* proxyShape, const IAABBox3D& aabb)
{

    // Add the collision shape into the dynamic AABB tree and get its broad-phase ID
    i32 nodeId = mDynamicAABBTree.AddObject(aabb, proxyShape);

    // Set the broad-phase ID of the proxy shape
    proxyShape->mBroadPhaseID = nodeId;

    // Add the collision shape into the array of bodies that have moved (or have been created)
    // during the last simulation step
    AddMovedCollisionShape(proxyShape->mBroadPhaseID);
}

// Remove a proxy collision shape from the broad-phase collision detection
template<class T>
SIMD_INLINE void IBroadPhase::RemoveProxyCollisionShape(T *proxyShape)
{
    i32 broadPhaseID = proxyShape->mBroadPhaseID;

    proxyShape->mBroadPhaseID = -1;

    // Remove the collision shape from the dynamic AABB tree
    mDynamicAABBTree.RemoveObject(broadPhaseID);

    // Remove the collision shape into the array of shapes that have moved (or have been created)
    // during the last simulation step
    RemoveMovedCollisionShape(broadPhaseID);
}

// Notify the broad-phase that a collision shape has moved and need to be updated
template<class T>
SIMD_INLINE void IBroadPhase::UpdateProxyCollisionShape(T* proxyShape, const IAABBox3D& aabb, const Vector3& displacement, bool forceReinsert)
{
    i32 broadPhaseID = proxyShape->mBroadPhaseID;

    assert(broadPhaseID >= 0);

    // Update the dynamic AABB tree according to the movement of the collision shape
    bool hasBeenReInserted = mDynamicAABBTree.UpdateObject(broadPhaseID, aabb, displacement, forceReinsert);

    // If the collision shape has moved out of its fat AABB (and therefore has been reinserted
    // into the tree).
    if (hasBeenReInserted)
    {
        // Add the collision shape into the array of shapes that have moved (or have been created)
        // during the last simulation step
        AddMovedCollisionShape(broadPhaseID);
    }
}

// Return true if the two broad-phase collision shapes are overlapping
template<class T>
SIMD_INLINE bool IBroadPhase::TestOverlappingShapes(const T* shape1,
                                                    const T* shape2) const
{
    // Get the two AABBs of the collision shapes
    const IAABBox3D& aabb1 = mDynamicAABBTree.GetFatAABB(shape1->mBroadPhaseID);
    const IAABBox3D& aabb2 = mDynamicAABBTree.GetFatAABB(shape2->mBroadPhaseID);

    // Check if the two AABBs are overlapping
    return aabb1.TestCollision(aabb2);
}

// Ray casting method
SIMD_INLINE void IBroadPhase::Raycast(const IRay &ray, IRaycastTest& raycastTest , unsigned short raycastWithCategoryMaskBits) const
{
    IBroadPhaseRaycastCallback broadPhaseRaycastCallback(mDynamicAABBTree, raycastWithCategoryMaskBits, raycastTest);
    mDynamicAABBTree.Raycast( ray , broadPhaseRaycastCallback );
}



}

#endif // IBROADPHASE_H
