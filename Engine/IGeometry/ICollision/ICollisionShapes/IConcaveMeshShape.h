#ifndef ICONCAVEMESHSHAPE_H
#define ICONCAVEMESHSHAPE_H

// Libraries
#include "IConcaveShape.h"
#include "../IBroadPhase/IDynamicAABBTree.h"



namespace IEngine
{
//// Declarations
class ConcaveMeshShape;
//class Profiler;
class TriangleShape;
class TriangleMesh;

// class ConvexTriangleAABBOverlapCallback
class ConvexTriangleAABBOverlapCallback : public IDynamicAABBTreeOverlapCallback {

    private:

        TriangleCallback& mTriangleTestCallback;

        // Reference to the concave mesh shape
        const ConcaveMeshShape& mConcaveMeshShape;

        // Reference to the Dynamic AABB tree
        const IDynamicAABBTree& mDynamicAABBTree;

    public:

        // Constructor
        ConvexTriangleAABBOverlapCallback(TriangleCallback& triangleCallback,
                                          const ConcaveMeshShape& concaveShape,
                                          const IDynamicAABBTree& dynamicAABBTree)
          : mTriangleTestCallback(triangleCallback),
            mConcaveMeshShape(concaveShape),
            mDynamicAABBTree(dynamicAABBTree)
        {

        }

        // Called when a overlapping node has been found during the call to
        // DynamicAABBTree:reportAllShapesOverlappingWithAABB()
        virtual void NotifyOverlappingNode(int nodeId) override;

};

/// Class ConcaveMeshRaycastCallback
class ConcaveMeshRaycastCallback : public IDynamicAABBTreeRaycastCallback {

    private :

        List<i32> mHitAABBNodes;
        const IDynamicAABBTree& mDynamicAABBTree;
        const ConcaveMeshShape& mConcaveMeshShape;
        IProxyShape* mProxyShape;
        IRaycastInfo& mRaycastInfo;
        const IRay& mRay;
        bool mIsHit;
 //       MemoryAllocator& mAllocator;

#ifdef IS_PROFILING_ACTIVE

        /// Pointer to the profiler
        Profiler* mProfiler;

#endif

    public:

        // Constructor
        ConcaveMeshRaycastCallback(const IDynamicAABBTree& dynamicAABBTree,
                                   const ConcaveMeshShape& concaveMeshShape,
                                   IProxyShape* proxyShape, IRaycastInfo& raycastInfo, const IRay& ray)
            : mDynamicAABBTree(dynamicAABBTree),
              mConcaveMeshShape(concaveMeshShape),
              mProxyShape(proxyShape),
              mRaycastInfo(raycastInfo), mRay(ray), mIsHit(false)
        {

        }

        /// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
        virtual scalar RaycastBroadPhaseShape(i32 nodeId, const IRay& ray) override;

        /// Raycast all collision shapes that have been collected
        void raycastTriangles();

        /// Return true if a raycast hit has been found
        bool getIsHit() const {
            return mIsHit;
        }

#ifdef IS_PROFILING_ACTIVE

        /// Set the profiler
        void setProfiler(Profiler* profiler) {
            mProfiler = profiler;
        }

#endif
};

// Class ConcaveMeshShape
/**
 * This class represents a static concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

    protected:

        // -------------------- Attributes -------------------- //

        /// Triangle mesh
        TriangleMesh* mTriangleMesh;

        /// Dynamic AABB tree to accelerate collision with the triangles
        IDynamicAABBTree mDynamicAABBTree;

        /// Array with computed vertices normals for each TriangleVertexArray of the triangle mesh (only
        /// if the user did not provide its own vertices normals)
        Vector3** mComputedVerticesNormals;

        /// Scaling
        const Vector3 mScaling;

        // -------------------- Methods -------------------- //

        /// Raycast method with feedback information
        virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t GetSizeInBytes() const override;

        /// Insert all the triangles into the dynamic AABB tree
        void initBVHTree();

        /// Compute the shape Id for a given triangle of the mesh
        u32 computeTriangleShapeId(u32 subPart, u32 triangleIndex) const;

    public:

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh, const Vector3& scaling = Vector3(1, 1, 1));

        /// Destructor
        virtual ~ConcaveMeshShape() override = default;

        /// Deleted copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape) = delete;

        /// Deleted assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape) = delete;

        /// Return the scaling vector
        const Vector3& getScaling() const;

        /// Return the number of sub parts contained in this mesh
        u32 getNbSubparts() const;

        /// Return the number of triangles in a sub part of the mesh
        u32 getNbTriangles(u32 subPart) const;

        /// Return the indices of the three vertices of a given triangle in the array
        void getTriangleVerticesIndices(u32 subPart, u32 triangleIndex, u32* outVerticesIndices) const;

        /// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
        void getTriangleVertices(u32 subPart, u32 triangleIndex, Vector3* outTriangleVertices) const;

        /// Return the three vertex normals (in the array outVerticesNormals) of a triangle
        void getTriangleVerticesNormals(u32 subPart, u32 triangleIndex, Vector3* outVerticesNormals) const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void GetLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape
        virtual Matrix3 ComputeLocalInertiaTensor2( scalar mass , const Matrix3& transform ) const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const IAABBox3D& localAABB) const override;

//        /// Return the string representation of the shape
//        virtual std::string to_string() const override;

//#ifdef IS_PROFILING_ACTIVE

//        /// Set the profiler
//        virtual void setProfiler(Profiler* profiler) override;

//#endif

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
};

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::GetSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Return the scaling vector
inline const Vector3& ConcaveMeshShape::getScaling() const
{
    return mScaling;
}



// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConcaveMeshShape::GetLocalBounds(Vector3 &min, Vector3 &max) const
{
    // Get the AABB of the whole tree
    IAABBox3D treeAABB = mDynamicAABBTree.GetRootAABB();

    min = treeAABB.GetMin();
    max = treeAABB.GetMax();
}


// Return the local inertia tensor of the shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline Matrix3 ConcaveMeshShape::ComputeLocalInertiaTensor2(scalar mass, const Matrix3 &transform) const
{
    // Default inertia tensor
    // Note that this is not very realistic for a concave triangle mesh.
    // However, in most cases, it will only be used static bodies and therefore,
    // the inertia tensor is not used.
    return Matrix3(mass, 0, 0,
                   0, mass, 0,
                   0, 0, mass);
}

// Called when a overlapping node has been found during the call to
// DynamicAABBTree:reportAllShapesOverlappingWithAABB()
inline void ConvexTriangleAABBOverlapCallback::NotifyOverlappingNode(int nodeId) {

    // Get the node data (triangle index and mesh subpart index)
    i32* data = mDynamicAABBTree.GetNodeDataInt(nodeId);

    // Get the triangle vertices for this node from the concave mesh shape
    Vector3 trianglePoints[3];
    mConcaveMeshShape.getTriangleVertices(data[0], data[1], trianglePoints);

    // Get the vertices normals of the triangle
    Vector3 verticesNormals[3];
    mConcaveMeshShape.getTriangleVerticesNormals(data[0], data[1], verticesNormals);

    // Call the callback to test narrow-phase collision with this triangle
    mTriangleTestCallback.testTriangle(trianglePoints, verticesNormals, mConcaveMeshShape.computeTriangleShapeId(data[0], data[1]));
}
}

#endif // ICONCAVEMESHSHAPE_H
