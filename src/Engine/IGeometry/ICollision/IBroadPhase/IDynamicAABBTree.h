#ifndef IDYNAMICAABBTREE_H
#define IDYNAMICAABBTREE_H

#include "../../../ICommon/ISettings.h"
#include "../../IAABBox3D.h"

namespace IEngine
{


using namespace IEngine;

// Declarations
//class IBroadPhaseAlgorithm;
//class IBroadPhaseRaycastTestCallback;
class IDynamicAABBTreeOverlapCallback;
class IRaycastTest;

// Structure TreeNode
/**
 * This structure represents a node of the dynamic AABB tree.
 */
class ITreeNode
{

    public:

        // -------------------- Constants -------------------- //

        /// Null tree node constant
        const static i32 NULL_TREE_NODE;

        // -------------------- Attributes -------------------- //

        // A node is either in the tree (has a parent) or in the free nodes list
        // (has a next node)
        union
        {

          /// Parent node ID
          i32 parentID;

          /// Next allocated node ID
          i32 nextNodeID;
        };

        // A node is either a leaf (has data) or is an internal node (has children)
        union
        {

            /// Left and right child of the node (children[0] = left child)
            i32 children[2];

            /// Two pieces of data stored at that node (in case the node is a leaf)
            union
            {
                i32   dataInt[2];
                void* dataPointer;
            };
        };

        /// Height of the node in the tree
        i16 height;

        /// Fat axis aligned bounding box (AABB) corresponding to the node
        IAABBox3D aabb;

        // -------------------- Methods -------------------- //

        /// Return true if the node is a leaf of the tree
        bool isLeaf() const;
};




// Class DynamicAABBTreeOverlapCallback
/**
 * Overlapping callback method that has to be used as parameter of the
 * reportAllShapesOverlappingWithNode() method.
 */
class IDynamicAABBTreeOverlapCallback
{

    public :

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void NotifyOverlappingNode(i32 nodeId)=0;
};




// Class DynamicAABBTreeRaycastCallback
/**
 * Raycast callback in the Dynamic AABB Tree called when the AABB of a leaf
 * node is hit by the ray.
 */
class IDynamicAABBTreeRaycastCallback
{

    public:

        // Called when the AABB of a leaf node is hit by a ray
        virtual scalar RaycastBroadPhaseShape(i32 nodeId, const IRay& ray)=0;

};

// Class DynamicAABBTree
/**
 * This class implements a dynamic AABB tree that is used for broad-phase
 * collision detection. This data structure is inspired by Nathanael Presson's
 * dynamic tree implementation in BulletPhysics. The following implementation is
 * based on the one from Erin Catto in Box2D as described in the book
 * "Introduction to Game Physics with Box2D" by Ian Parberry.
 */
class IDynamicAABBTree
{

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the memory location of the nodes of the tree
        ITreeNode* mNodes;

        /// ID of the root node of the tree
        i32 mRootNodeID;

        /// ID of the first node of the list of free (allocated) nodes in the tree that we can use
        i32 mFreeNodeID;

        /// Number of allocated nodes in the tree
        i32 mNbAllocatedNodes;

        /// Number of nodes in the tree
        i32 mNbNodes;

        /// Extra AABB Gap used to allow the collision shape to move a little bit
        /// without triggering a large modification of the tree which can be costly
        scalar mExtraAABBGap;

        // -------------------- Methods -------------------- //

        /// Allocate and return a node to use in the tree
        i32 AllocateNode();

        /// Release a node
        void ReleaseNode(i32 nodeID);

        /// Insert a leaf node in the tree
        void InsertLeafNode(i32 nodeID);

        /// Remove a leaf node from the tree
        void RemoveLeafNode(i32 nodeID);

        /// Balance the sub-tree of a given node using left or right rotations.
        i32 BalanceSubTreeAtNode(i32 nodeID);

        /// Compute the height of a given node in the tree
        i32 ComputeHeight(i32 nodeID);

        /// Internally add an object into the tree
        i32 AddObjectInternal(const IAABBox3D& aabb);

        /// Initialize the tree
        void Init();

#ifndef NDEBUG

        /// Check if the tree structure is valid (for debugging purpose)
        void Check() const;

        /// Check if the node structure is valid (for debugging purpose)
        void CheckNode(i32 nodeID) const;

#endif

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IDynamicAABBTree(scalar extraAABBGap = scalar(0.0));

        /// Destructor
        ~IDynamicAABBTree();

        /// Add an object into the tree (where node data are two integers)
        i32 AddObject(const IAABBox3D& aabb, i32 data1, i32 data2);

        /// Add an object into the tree (where node data is a pointer)
        i32 AddObject(const IAABBox3D& aabb, void* data);

        /// Remove an object from the tree
        void RemoveObject(u32 nodeID);

        /// Update the dynamic tree after an object has moved.
        bool UpdateObject(i32 nodeID, const IAABBox3D& newAABB, const Vector3& displacement, bool forceReinsert = false);

        /// Return the fat AABB corresponding to a given node ID
        const IAABBox3D& GetFatAABB(i32 nodeID) const;

        /// Return the pointer to the data array of a given leaf node of the tree
        i32* GetNodeDataInt(i32 nodeID) const;

        /// Return the data pointer of a given leaf node of the tree
        void* GetNodeDataPointer(i32 nodeID) const;

        /// Report all shapes overlapping with the AABB given in parameter.
        void ReportAllShapesOverlappingWithAABB(const IAABBox3D& aabb, IDynamicAABBTreeOverlapCallback& callback) const;

        /// Ray casting method
        void Raycast(const IRay& ray, IDynamicAABBTreeRaycastCallback& callback ) const;

        /// Compute the height of the tree
        i32 ComputeHeight();

        /// Return the root AABB of the tree
        IAABBox3D GetRootAABB() const;

        /// Clear all the nodes and reset the tree
        void Reset();
};

// Return true if the node is a leaf of the tree
SIMD_INLINE bool ITreeNode::isLeaf() const
{
    return (height == 0);
}

// Return the fat AABB corresponding to a given node ID
SIMD_INLINE const IAABBox3D& IDynamicAABBTree::GetFatAABB(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    return mNodes[nodeID].aabb;
}

// Return the pointer to the data array of a given leaf node of the tree
SIMD_INLINE i32* IDynamicAABBTree::GetNodeDataInt(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataInt;
}

// Return the pointer to the data pointer of a given leaf node of the tree
SIMD_INLINE void* IDynamicAABBTree::GetNodeDataPointer(i32 nodeID) const
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    return mNodes[nodeID].dataPointer;
}

// Return the root AABB of the tree
SIMD_INLINE IAABBox3D IDynamicAABBTree::GetRootAABB() const
{
    return GetFatAABB(mRootNodeID);
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
SIMD_INLINE i32 IDynamicAABBTree::AddObject(const IAABBox3D& aabb, i32 data1, i32 data2)
{
    i32 nodeId = AddObjectInternal(aabb);

    mNodes[nodeId].dataInt[0] = data1;
    mNodes[nodeId].dataInt[1] = data2;

    return nodeId;
}

// Add an object into the tree. This method creates a new leaf node in the tree and
// returns the ID of the corresponding node.
SIMD_INLINE i32 IDynamicAABBTree::AddObject(const IAABBox3D& aabb, void* data)
{
    i32 nodeId = AddObjectInternal(aabb);
    mNodes[nodeId].dataPointer = data;
    return nodeId;
}



}
#endif // IDYNAMICAABBTREE_H
