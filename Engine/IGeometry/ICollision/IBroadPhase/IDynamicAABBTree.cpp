#include "IDynamicAABBTree.h"

#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <cstring>

#include "../../../ICommon/IMemory/IStack.h"


namespace IEngine
{


using namespace IMath;

// Initialization of static variables
const i32 ITreeNode::NULL_TREE_NODE = -1;


// Constructor
IDynamicAABBTree::IDynamicAABBTree(scalar extraAABBGap) : mExtraAABBGap(extraAABBGap)
{
    Init();
}

// Destructor
IDynamicAABBTree::~IDynamicAABBTree()
{
    // Free the allocated memory for the nodes
    free(mNodes);
}

// Initialize the tree
void IDynamicAABBTree::Init()
{

    mRootNodeID = ITreeNode::NULL_TREE_NODE;
    mNbNodes = 0;
    mNbAllocatedNodes = 8;

    // Allocate memory for the nodes of the tree
    mNodes = (ITreeNode*) IAlloc(mNbAllocatedNodes * sizeof(ITreeNode));
    assert(mNodes);
    memset(mNodes, 0, mNbAllocatedNodes * sizeof(ITreeNode));

    // Initialize the allocated nodes
    for (i32 i=0; i<mNbAllocatedNodes - 1; i++)
    {
        mNodes[i].nextNodeID = i + 1;
        mNodes[i].height = -1;
    }
    mNodes[mNbAllocatedNodes - 1].nextNodeID = ITreeNode::NULL_TREE_NODE;
    mNodes[mNbAllocatedNodes - 1].height = -1;
    mFreeNodeID = 0;
}

// Clear all the nodes and reset the tree
void IDynamicAABBTree::Reset()
{
    // Free the allocated memory for the nodes
    IFree(mNodes);

    // Initialize the tree
    Init();
}



// Allocate and return a new node in the tree
i32 IDynamicAABBTree::AllocateNode()
{

    // If there is no more allocated node to use
    if (mFreeNodeID == ITreeNode::NULL_TREE_NODE)
    {

        assert(mNbNodes == mNbAllocatedNodes);

        // Allocate more nodes in the tree
        mNbAllocatedNodes *= 2;
        ITreeNode* oldNodes = mNodes;
        mNodes = (ITreeNode*) IAlloc(mNbAllocatedNodes * sizeof(ITreeNode));
        assert(mNodes);
        memcpy(mNodes, oldNodes, mNbNodes * sizeof(ITreeNode));
        free(oldNodes);

        // Initialize the allocated nodes
        for (i32 i=mNbNodes; i<mNbAllocatedNodes - 1; i++)
        {
            mNodes[i].nextNodeID = i + 1;
            mNodes[i].height = -1;
        }
        mNodes[mNbAllocatedNodes - 1].nextNodeID = ITreeNode::NULL_TREE_NODE;
        mNodes[mNbAllocatedNodes - 1].height = -1;
        mFreeNodeID = mNbNodes;
    }

    // Get the next free node
    i32 freeNodeID = mFreeNodeID;
    mFreeNodeID = mNodes[freeNodeID].nextNodeID;
    mNodes[freeNodeID].parentID = ITreeNode::NULL_TREE_NODE;
    mNodes[freeNodeID].height = 0;
    mNbNodes++;

    return freeNodeID;
}

// Release a node
void IDynamicAABBTree::ReleaseNode(i32 nodeID)
{

    assert(mNbNodes > 0);
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].height >= 0);
    mNodes[nodeID].nextNodeID = mFreeNodeID;
    mNodes[nodeID].height = -1;
    mFreeNodeID = nodeID;
    mNbNodes--;
}

// Internally add an object into the tree
i32 IDynamicAABBTree::AddObjectInternal(const IAABBox3D& aabb)
{

    // Get the next available node (or allocate new ones if necessary)
    u32 nodeID = AllocateNode();

    // Create the fat aabb to use in the tree
    const Vector3 gap(mExtraAABBGap, mExtraAABBGap, mExtraAABBGap);
    mNodes[nodeID].aabb.SetMin(aabb.GetMin() - gap);
    mNodes[nodeID].aabb.SetMax(aabb.GetMax() + gap);

    // Set the height of the node in the tree
    mNodes[nodeID].height = 0;

    // Insert the new leaf node in the tree
    InsertLeafNode(nodeID);
    assert(mNodes[nodeID].isLeaf());

    assert(nodeID >= 0);

    // Return the Id of the node
    return nodeID;
}

// Remove an object from the tree
void IDynamicAABBTree::RemoveObject(u32 nodeID)
{
    assert(nodeID >= 0 && nodeID < (unsigned int)mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());

    // Remove the node from the tree
    RemoveLeafNode(nodeID);
    ReleaseNode(nodeID);
}

// Update the dynamic tree after an object has moved.
/// If the new AABB of the object that has moved is still inside its fat AABB, then
/// nothing is done. Otherwise, the corresponding node is removed and reinserted into the tree.
/// The method returns true if the object has been reinserted into the tree. The "displacement"
/// argument is the linear velocity of the AABB multiplied by the elapsed time between two
/// frames. If the "forceReinsert" parameter is true, we force a removal and reinsertion of the node
/// (this can be useful if the shape AABB has become much smaller than the previous one for instance).
bool IDynamicAABBTree::UpdateObject(i32 nodeID, const IAABBox3D& newAABB, const Vector3& displacement, bool forceReinsert)
{

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());
    assert(mNodes[nodeID].height >= 0);

    // If the new AABB is still inside the fat AABB of the node
    if (!forceReinsert && mNodes[nodeID].aabb.Containsss(newAABB))
    {
        return false;
    }

    // If the new AABB is outside the fat AABB, we remove the corresponding node
    RemoveLeafNode(nodeID);

    // Compute the fat AABB by inflating the AABB with a constant gap
    mNodes[nodeID].aabb = newAABB;
    const Vector3 gap(mExtraAABBGap, mExtraAABBGap, mExtraAABBGap);


//    mNodes[nodeID].aabb.mMin -= gap;
//    mNodes[nodeID].aabb.mMax += gap;

//    // Inflate the fat AABB in direction of the linear motion of the AABB
//    if (displacement.x < scalar(0.0))
//    {
//        mNodes[nodeID].aabb.mMin.x += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.x;
//    }
//    else
//    {
//        mNodes[nodeID].aabb.mMax.x += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.x;
//    }
//    if (displacement.y < scalar(0.0))
//    {
//        mNodes[nodeID].aabb.mMin.y += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.y;
//    }
//    else
//    {
//        mNodes[nodeID].aabb.mMax.y += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.y;
//    }
//    if (displacement.z < scalar(0.0))
//    {
//        mNodes[nodeID].aabb.mMin.z += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.z;
//    }
//    else
//    {
//        mNodes[nodeID].aabb.mMax.z += DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER *displacement.z;
//    }

//    assert(mNodes[nodeID].aabb.Contains(newAABB));

    // Reinsert the node into the tree
    InsertLeafNode(nodeID);

    return true;
}

// Insert a leaf node in the tree. The process of inserting a new leaf node
// in the dynamic tree is described in the book "Introduction to Game Physics
// with Box2D" by Ian Parberry.
void IDynamicAABBTree::InsertLeafNode(i32 nodeID)
{
    // If the tree is empty
    if (mRootNodeID == ITreeNode::NULL_TREE_NODE)
    {
        mRootNodeID = nodeID;
        mNodes[mRootNodeID].parentID = ITreeNode::NULL_TREE_NODE;
        return;
    }

    assert(mRootNodeID != ITreeNode::NULL_TREE_NODE);

    // Find the best sibling node for the new node
    IAABBox3D newNodeAABB = mNodes[nodeID].aabb;
    i32 currentNodeID = mRootNodeID;
    while (!mNodes[currentNodeID].isLeaf())
    {

        i32 leftChild = mNodes[currentNodeID].children[0];
        i32 rightChild = mNodes[currentNodeID].children[1];

        // Compute the merged AABB
        scalar volumeAABB = mNodes[currentNodeID].aabb.GetVolume();
        IAABBox3D mergedAABBs;
        mergedAABBs.MergeTwoAABBs(mNodes[currentNodeID].aabb, newNodeAABB);
        scalar mergedVolume = mergedAABBs.GetVolume();

        // Compute the cost of making the current node the sibbling of the new node
        scalar costS = scalar(2.0) * mergedVolume;

        // Compute the minimum cost of pushing the new node further down the tree (inheritance cost)
        scalar costI = scalar(2.0) * (mergedVolume - volumeAABB);

        // Compute the cost of descending into the left child
        scalar costLeft;
        IAABBox3D currentAndLeftAABB;
        currentAndLeftAABB.MergeTwoAABBs(newNodeAABB, mNodes[leftChild].aabb);
        if (mNodes[leftChild].isLeaf())
        {   // If the left child is a leaf
            costLeft = currentAndLeftAABB.GetVolume() + costI;
        }
        else
        {
            scalar leftChildVolume = mNodes[leftChild].aabb.GetVolume();
            costLeft = costI + currentAndLeftAABB.GetVolume() - leftChildVolume;
        }

        // Compute the cost of descending into the right child
        scalar costRight;
        IAABBox3D currentAndRightAABB;
        currentAndRightAABB.MergeTwoAABBs(newNodeAABB, mNodes[rightChild].aabb);
        if (mNodes[rightChild].isLeaf())
        {   // If the right child is a leaf
            costRight = currentAndRightAABB.GetVolume() + costI;
        }
        else
        {
            scalar rightChildVolume = mNodes[rightChild].aabb.GetVolume();
            costRight = costI + currentAndRightAABB.GetVolume() - rightChildVolume;
        }

        // If the cost of making the current node a sibbling of the new node is smaller than
        // the cost of going down into the left or right child
        if (costS < costLeft && costS < costRight) break;

        // It is cheaper to go down into a child of the current node, choose the best child
        if (costLeft < costRight)
        {
            currentNodeID = leftChild;
        }
        else
        {
            currentNodeID = rightChild;
        }
    }

    i32 siblingNode = currentNodeID;

    // Create a new parent for the new node and the sibling node
    i32 oldParentNode = mNodes[siblingNode].parentID;
    i32 newParentNode = AllocateNode();
    mNodes[newParentNode].parentID = oldParentNode;
    mNodes[newParentNode].aabb.MergeTwoAABBs(mNodes[siblingNode].aabb, newNodeAABB);
    mNodes[newParentNode].height = mNodes[siblingNode].height + 1;
    assert(mNodes[newParentNode].height > 0);

    // If the sibling node was not the root node
    if (oldParentNode != ITreeNode::NULL_TREE_NODE)
    {
        assert(!mNodes[oldParentNode].isLeaf());
        if (mNodes[oldParentNode].children[0] == siblingNode)
        {
            mNodes[oldParentNode].children[0] = newParentNode;
        }
        else
        {
            mNodes[oldParentNode].children[1] = newParentNode;
        }
        mNodes[newParentNode].children[0] = siblingNode;
        mNodes[newParentNode].children[1] = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
    }
    else
    {  // If the sibling node was the root node
        mNodes[newParentNode].children[0] = siblingNode;
        mNodes[newParentNode].children[1] = nodeID;
        mNodes[siblingNode].parentID = newParentNode;
        mNodes[nodeID].parentID = newParentNode;
        mRootNodeID = newParentNode;
    }

    // Move up in the tree to change the AABBs that have changed
    currentNodeID = mNodes[nodeID].parentID;
    assert(!mNodes[currentNodeID].isLeaf());
    while (currentNodeID != ITreeNode::NULL_TREE_NODE)
    {

        // Balance the sub-tree of the current node if it is not balanced
        currentNodeID = BalanceSubTreeAtNode(currentNodeID);
        assert(mNodes[nodeID].isLeaf());

        assert(!mNodes[currentNodeID].isLeaf());
        i32 leftChild = mNodes[currentNodeID].children[0];
        i32 rightChild = mNodes[currentNodeID].children[1];
        assert(leftChild != ITreeNode::NULL_TREE_NODE);
        assert(rightChild != ITreeNode::NULL_TREE_NODE);

        // Recompute the height of the node in the tree
        mNodes[currentNodeID].height = IMax(mNodes[leftChild].height,
                                            mNodes[rightChild].height) + 1;
        assert(mNodes[currentNodeID].height > 0);

        // Recompute the AABB of the node
        mNodes[currentNodeID].aabb.MergeTwoAABBs(mNodes[leftChild].aabb, mNodes[rightChild].aabb);

        currentNodeID = mNodes[currentNodeID].parentID;
    }

    assert(mNodes[nodeID].isLeaf());
}

// Remove a leaf node from the tree
void IDynamicAABBTree::RemoveLeafNode(i32 nodeID)
{

    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    assert(mNodes[nodeID].isLeaf());

    // If we are removing the root node (root node is a leaf in this case)
    if (mRootNodeID == nodeID)
    {
        mRootNodeID = ITreeNode::NULL_TREE_NODE;
        return;
    }

    i32 parentNodeID = mNodes[nodeID].parentID;
    i32 grandParentNodeID = mNodes[parentNodeID].parentID;
    i32 siblingNodeID;
    if (mNodes[parentNodeID].children[0] == nodeID)
    {
        siblingNodeID = mNodes[parentNodeID].children[1];
    }
    else
    {
        siblingNodeID = mNodes[parentNodeID].children[0];
    }

    // If the parent of the node to remove is not the root node
    if (grandParentNodeID != ITreeNode::NULL_TREE_NODE)
    {

        // Destroy the parent node
        if (mNodes[grandParentNodeID].children[0] == parentNodeID)
        {
            mNodes[grandParentNodeID].children[0] = siblingNodeID;
        }
        else
        {
            assert(mNodes[grandParentNodeID].children[1] == parentNodeID);
            mNodes[grandParentNodeID].children[1] = siblingNodeID;
        }
        mNodes[siblingNodeID].parentID = grandParentNodeID;
        ReleaseNode(parentNodeID);

        // Now, we need to recompute the AABBs of the node on the path back to the root
        // and make sure that the tree is still balanced
        i32 currentNodeID = grandParentNodeID;
        while(currentNodeID != ITreeNode::NULL_TREE_NODE)
        {

            // Balance the current sub-tree if necessary
            currentNodeID = BalanceSubTreeAtNode(currentNodeID);

            assert(!mNodes[currentNodeID].isLeaf());

            // Get the two children of the current node
            i32 leftChildID  = mNodes[currentNodeID].children[0];
            i32 rightChildID = mNodes[currentNodeID].children[1];

            // Recompute the AABB and the height of the current node
            mNodes[currentNodeID].aabb.MergeTwoAABBs(mNodes[leftChildID].aabb,
                                                     mNodes[rightChildID].aabb);

            mNodes[currentNodeID].height = IMax(mNodes[leftChildID].height,
                                                mNodes[rightChildID].height) + 1;
            assert(mNodes[currentNodeID].height > 0);

            currentNodeID = mNodes[currentNodeID].parentID;
        }
    }
    else
    { // If the parent of the node to remove is the root node

        // The sibling node becomes the new root node
        mRootNodeID = siblingNodeID;
        mNodes[siblingNodeID].parentID =ITreeNode::NULL_TREE_NODE;
        ReleaseNode(parentNodeID);
    }
}

// Balance the sub-tree of a given node using left or right rotations.
/// The rotation schemes are described in the book "Introduction to Game Physics
/// with Box2D" by Ian Parberry. This method returns the new root node ID.
i32 IDynamicAABBTree::BalanceSubTreeAtNode(i32 nodeID)
{

    assert(nodeID != ITreeNode::NULL_TREE_NODE);

    ITreeNode* nodeA = mNodes + nodeID;

    // If the node is a leaf or the height of A's sub-tree is less than 2
    if (nodeA->isLeaf() || nodeA->height < 2)
    {
        // Do not perform any rotation
        return nodeID;
    }

    // Get the two children nodes
    i32 nodeBID = nodeA->children[0];
    i32 nodeCID = nodeA->children[1];
    assert(nodeBID >= 0 && nodeBID < mNbAllocatedNodes);
    assert(nodeCID >= 0 && nodeCID < mNbAllocatedNodes);
    ITreeNode* nodeB = mNodes + nodeBID;
    ITreeNode* nodeC = mNodes + nodeCID;

    // Compute the factor of the left and right sub-trees
    i32 balanceFactor = nodeC->height - nodeB->height;

    // If the right node C is 2 higher than left node B
    if (balanceFactor > 1)
    {

        assert(!nodeC->isLeaf());

        i32 nodeFID = nodeC->children[0];
        i32 nodeGID = nodeC->children[1];
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        ITreeNode* nodeF = mNodes + nodeFID;
        ITreeNode* nodeG = mNodes + nodeGID;

        nodeC->children[0] = nodeID;
        nodeC->parentID = nodeA->parentID;
        nodeA->parentID = nodeCID;

        if (nodeC->parentID != ITreeNode::NULL_TREE_NODE)
        {

            if (mNodes[nodeC->parentID].children[0] == nodeID)
            {
                mNodes[nodeC->parentID].children[0] = nodeCID;
            }
            else
            {
                assert(mNodes[nodeC->parentID].children[1] == nodeID);
                mNodes[nodeC->parentID].children[1] = nodeCID;
            }
        }
        else
        {
            mRootNodeID = nodeCID;
        }

        assert(!nodeC->isLeaf());
        assert(!nodeA->isLeaf());

        // If the right node C was higher than left node B because of the F node
        if (nodeF->height > nodeG->height)
        {

            nodeC->children[1] = nodeFID;
            nodeA->children[1] = nodeGID;
            nodeG->parentID = nodeID;

            // Recompute the AABB of node A and C
            nodeA->aabb.MergeTwoAABBs(nodeB->aabb, nodeG->aabb);
            nodeC->aabb.MergeTwoAABBs(nodeA->aabb, nodeF->aabb);

            // Recompute the height of node A and C
            nodeA->height = IMax(nodeB->height, nodeG->height) + 1;
            nodeC->height = IMax(nodeA->height, nodeF->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeC->height > 0);
        }
        else
        {  // If the right node C was higher than left node B because of node G
            nodeC->children[1] = nodeGID;
            nodeA->children[1] = nodeFID;
            nodeF->parentID = nodeID;

            // Recompute the AABB of node A and C
            nodeA->aabb.MergeTwoAABBs(nodeB->aabb, nodeF->aabb);
            nodeC->aabb.MergeTwoAABBs(nodeA->aabb, nodeG->aabb);

            // Recompute the height of node A and C
            nodeA->height = IMax(nodeB->height, nodeF->height) + 1;
            nodeC->height = IMax(nodeA->height, nodeG->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeC->height > 0);
        }

        // Return the new root of the sub-tree
        return nodeCID;
    }

    // If the left node B is 2 higher than right node C
    if (balanceFactor < -1)
    {

        assert(!nodeB->isLeaf());

        i32 nodeFID = nodeB->children[0];
        i32 nodeGID = nodeB->children[1];
        assert(nodeFID >= 0 && nodeFID < mNbAllocatedNodes);
        assert(nodeGID >= 0 && nodeGID < mNbAllocatedNodes);
        ITreeNode* nodeF = mNodes + nodeFID;
        ITreeNode* nodeG = mNodes + nodeGID;

        nodeB->children[0] = nodeID;
        nodeB->parentID = nodeA->parentID;
        nodeA->parentID = nodeBID;

        if (nodeB->parentID != ITreeNode::NULL_TREE_NODE)
        {

            if (mNodes[nodeB->parentID].children[0] == nodeID)
            {
                mNodes[nodeB->parentID].children[0] = nodeBID;
            }
            else
            {
                assert(mNodes[nodeB->parentID].children[1] == nodeID);
                mNodes[nodeB->parentID].children[1] = nodeBID;
            }
        }
        else
        {
            mRootNodeID = nodeBID;
        }

        assert(!nodeB->isLeaf());
        assert(!nodeA->isLeaf());

        // If the left node B was higher than right node C because of the F node
        if (nodeF->height > nodeG->height)
        {

            nodeB->children[1] = nodeFID;
            nodeA->children[0] = nodeGID;
            nodeG->parentID = nodeID;

            // Recompute the AABB of node A and B
            nodeA->aabb.MergeTwoAABBs(nodeC->aabb, nodeG->aabb);
            nodeB->aabb.MergeTwoAABBs(nodeA->aabb, nodeF->aabb);

            // Recompute the height of node A and B
            nodeA->height = IMax(nodeC->height, nodeG->height) + 1;
            nodeB->height = IMax(nodeA->height, nodeF->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeB->height > 0);
        }
        else
        {  // If the left node B was higher than right node C because of node G
            nodeB->children[1] = nodeGID;
            nodeA->children[0] = nodeFID;
            nodeF->parentID = nodeID;

            // Recompute the AABB of node A and B
            nodeA->aabb.MergeTwoAABBs(nodeC->aabb, nodeF->aabb);
            nodeB->aabb.MergeTwoAABBs(nodeA->aabb, nodeG->aabb);

            // Recompute the height of node A and B
            nodeA->height = IMax(nodeC->height, nodeF->height) + 1;
            nodeB->height = IMax(nodeA->height, nodeG->height) + 1;
            assert(nodeA->height > 0);
            assert(nodeB->height > 0);
        }

        // Return the new root of the sub-tree
        return nodeBID;
    }

    // If the sub-tree is balanced, return the current root node
    return nodeID;
}

/// Report all shapes overlapping with the AABB given in parameter.
void IDynamicAABBTree::ReportAllShapesOverlappingWithAABB(const IAABBox3D& aabb, IDynamicAABBTreeOverlapCallback& callback) const
{

    // Create a stack with the nodes to visit
    IStack<i32, 64> stack;
    stack.Push(mRootNodeID);

    // While there are still nodes to visit
    while(stack.Count() > 0)
    {

        // Get the next node ID to visit
        i32 nodeIDToVisit = stack.Pop();

        // Skip it if it is a null node
        if (nodeIDToVisit == ITreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const ITreeNode* nodeToVisit = mNodes + nodeIDToVisit;

        // If the AABB in parameter overlaps with the AABB of the node to visit
        if (aabb.TestCollision(nodeToVisit->aabb))
        {
            // If the node is a leaf
            if (nodeToVisit->isLeaf())
            {
                // Notify the broad-phase about a new potential overlapping pair
                callback.NotifyOverlappingNode(nodeIDToVisit);
            }
            else
            {  // If the node is not a leaf

                // We need to visit its children
                stack.Push(nodeToVisit->children[0]);
                stack.Push(nodeToVisit->children[1]);
            }
        }
    }
}

// Ray casting method
void IDynamicAABBTree::Raycast(const IRay& ray, IDynamicAABBTreeRaycastCallback &callback ) const
{
    scalar maxFraction = ray.maxFraction;

    IStack<i32, 128> stack;
    stack.Push(mRootNodeID);


    // Walk through the tree from the root looking for proxy shapes
    // that overlap with the ray AABB
    while (stack.Count() > 0)
    {
        // Get the next node in the stack
        i32 nodeID = stack.Pop();

        // If it is a null node, skip it
        if (nodeID == ITreeNode::NULL_TREE_NODE) continue;

        // Get the corresponding node
        const ITreeNode* node = mNodes + nodeID;

        IRay rayTemp(ray.Origin, ray.Direction, maxFraction);

        // Test if the ray intersects with the current node AABB
        if (!node->aabb.TestRayIntersect(rayTemp)) continue;


        // If the node is a leaf of the tree
        if (node->isLeaf())
        {

            // Call the callback that will raycast again the broad-phase shape
            scalar hitFraction = callback.RaycastBroadPhaseShape(nodeID, rayTemp);

            // If the user returned a hitFraction of zero, it means that
            // the raycasting should stop here
//            if (hitFraction == scalar(0.0))
//            {
//                return;
//            }

            // If the user returned a positive fraction
            if (hitFraction > scalar(0.0))
            {
                // We update the maxFraction value and the ray
                // AABB using the new maximum fraction
                if (hitFraction < maxFraction)
                {
                    maxFraction = hitFraction;
                }
            }

            // If the user returned a negative fraction, we continue
            // the raycasting as if the proxy shape did not exist
        }
        else
        {   // If the node has children

            // Push its children in the stack of nodes to explore
            stack.Push(node->children[0]);
            stack.Push(node->children[1]);
        }
    }
}



// Check if the tree structure is valid (for debugging purpose)
void IDynamicAABBTree::Check() const
{
    // Recursively check each node
    CheckNode(mRootNodeID);

    i32 nbFreeNodes = 0;
    i32 freeNodeID = mFreeNodeID;

    // Check the free nodes
    while(freeNodeID != ITreeNode::NULL_TREE_NODE)
    {
        assert(0 <= freeNodeID && freeNodeID < mNbAllocatedNodes);
        freeNodeID = mNodes[freeNodeID].nextNodeID;
        nbFreeNodes++;
    }

    assert(mNbNodes + nbFreeNodes == mNbAllocatedNodes);
}

// Check if the node structure is valid (for debugging purpose)
void IDynamicAABBTree::CheckNode(i32 nodeID) const
{

    if (nodeID == ITreeNode::NULL_TREE_NODE) return;

    // If it is the root
    if (nodeID == mRootNodeID)
    {
        assert(mNodes[nodeID].parentID == ITreeNode::NULL_TREE_NODE);
    }

    // Get the children nodes
    ITreeNode* pNode = mNodes + nodeID;
    assert(!pNode->isLeaf());
    i32 leftChild = pNode->children[0];
    i32 rightChild = pNode->children[1];

    assert(pNode->height >= 0);
    assert(pNode->aabb.GetVolume() > 0);

    // If the current node is a leaf
    if (pNode->isLeaf())
    {

        // Check that there are no children
        assert(leftChild == ITreeNode::NULL_TREE_NODE);
        assert(rightChild == ITreeNode::NULL_TREE_NODE);
        assert(pNode->height == 0);
    }
    else
    {

        // Check that the children node IDs are valid
        assert(0 <= leftChild && leftChild < mNbAllocatedNodes);
        assert(0 <= rightChild && rightChild < mNbAllocatedNodes);

        // Check that the children nodes have the correct parent node
        assert(mNodes[leftChild].parentID == nodeID);
        assert(mNodes[rightChild].parentID == nodeID);

        // Check the height of node
        i32 height = 1 + IMax(mNodes[leftChild].height, mNodes[rightChild].height);
        assert(mNodes[nodeID].height == height);

        // Check the AABB of the node
        IAABBox3D aabb;
        aabb.MergeTwoAABBs(mNodes[leftChild].aabb, mNodes[rightChild].aabb);
        assert(aabb.GetMin() == mNodes[nodeID].aabb.GetMin());
        assert(aabb.GetMax() == mNodes[nodeID].aabb.GetMax());

        // Recursively check the children nodes
        CheckNode(leftChild);
        CheckNode(rightChild);
    }
}

// Compute the height of the tree
i32 IDynamicAABBTree::ComputeHeight()
{
   return ComputeHeight(mRootNodeID);
}

// Compute the height of a given node in the tree
i32 IDynamicAABBTree::ComputeHeight(i32 nodeID)
{
    assert(nodeID >= 0 && nodeID < mNbAllocatedNodes);
    ITreeNode* node = mNodes + nodeID;

    // If the node is a leaf, its height is zero
    if (node->isLeaf())
    {
        return 0;
    }

    // Compute the height of the left and right sub-tree
    i32 leftHeight = ComputeHeight(node->children[0]);
    i32 rightHeight = ComputeHeight(node->children[1]);

    // Return the height of the node
    return 1 + IMax(leftHeight, rightHeight);
}

}
