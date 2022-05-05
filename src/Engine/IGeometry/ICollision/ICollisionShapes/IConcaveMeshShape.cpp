#include "IConcaveMeshShape.h"
#include "ITriangleVertexArray.h"
#include "ITriangleShape.h"
#include "ITriangleMesh.h"


namespace IEngine
{

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh, const Vector3& scaling)
                 : ConcaveShape(), mDynamicAABBTree(),
                   mScaling(scaling)
{
    mTriangleMesh = triangleMesh;
    mRaycastTestType = TriangleRaycastSide::FRONT;

    // Insert all the triangles into the dynamic AABB tree
    initBVHTree();
}

// Insert all the triangles into the dynamic AABB tree
void ConcaveMeshShape::initBVHTree() {

    // TODO : Try to randomly add the triangles into the tree to obtain a better tree

    // For each sub-part of the mesh
    for (u32 subPart=0; subPart<mTriangleMesh->getNbSubparts(); subPart++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

        // For each triangle of the concave mesh
        for (u32 triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

            Vector3 trianglePoints[3];

            // Get the triangle vertices
            triangleVertexArray->getTriangleVertices(triangleIndex, trianglePoints);

            // Apply the scaling factor to the vertices
            trianglePoints[0].x *= mScaling.x;
            trianglePoints[0].y *= mScaling.y;
            trianglePoints[0].z *= mScaling.z;
            trianglePoints[1].x *= mScaling.x;
            trianglePoints[1].y *= mScaling.y;
            trianglePoints[1].z *= mScaling.z;
            trianglePoints[2].x *= mScaling.x;
            trianglePoints[2].y *= mScaling.y;
            trianglePoints[2].z *= mScaling.z;


            // Create the AABB for the triangle
            IAABBox3D aabb = IAABBox3D::CreateAABBForTriangle(trianglePoints);

            // Add the AABB with the index of the triangle into the dynamic AABB tree
            mDynamicAABBTree.AddObject(aabb, subPart, triangleIndex);
        }
    }
}

// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
void ConcaveMeshShape::getTriangleVertices(u32 subPart, u32 triangleIndex,
                                           Vector3* outTriangleVertices) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices coordinates of the triangle
    triangleVertexArray->getTriangleVertices(triangleIndex, outTriangleVertices);

    // Apply the scaling factor to the vertices
    outTriangleVertices[0].x *= mScaling.x;
    outTriangleVertices[0].y *= mScaling.y;
    outTriangleVertices[0].z *= mScaling.z;
    outTriangleVertices[1].x *= mScaling.x;
    outTriangleVertices[1].y *= mScaling.y;
    outTriangleVertices[1].z *= mScaling.z;
    outTriangleVertices[2].x *= mScaling.x;
    outTriangleVertices[2].y *= mScaling.y;
    outTriangleVertices[2].z *= mScaling.z;
}

// Return the three vertex normals (in the array outVerticesNormals) of a triangle
void ConcaveMeshShape::getTriangleVerticesNormals(u32 subPart, u32 triangleIndex, Vector3* outVerticesNormals) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices normals of the triangle
    triangleVertexArray->getTriangleVerticesNormals(triangleIndex, outVerticesNormals);
}

// Return the indices of the three vertices of a given triangle in the array
void ConcaveMeshShape::getTriangleVerticesIndices(u32 subPart, u32 triangleIndex, u32* outVerticesIndices) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices normals of the triangle
    triangleVertexArray->getTriangleVerticesIndices(triangleIndex, outVerticesIndices);
}

// Return the number of sub parts contained in this mesh
u32 ConcaveMeshShape::getNbSubparts() const
{
    return mTriangleMesh->getNbSubparts();
}

// Return the number of triangles in a sub part of the mesh
u32 ConcaveMeshShape::getNbTriangles(u32 subPart) const
{
    assert(mTriangleMesh->getSubpart(subPart));
    return mTriangleMesh->getSubpart(subPart)->getNbTriangles();
}

// Use a callback method on all triangles of the concave shape inside a given AABB
void ConcaveMeshShape::testAllTriangles(TriangleCallback& callback, const IAABBox3D& localAABB) const {

    ConvexTriangleAABBOverlapCallback overlapCallback(callback, *this, mDynamicAABBTree);

    // Ask the Dynamic AABB Tree to report all the triangles that are overlapping
    // with the AABB of the convex shape.
    mDynamicAABBTree.ReportAllShapesOverlappingWithAABB(localAABB, overlapCallback);
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool ConcaveMeshShape::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

   // RP3D_PROFILE("ConcaveMeshShape::raycast()", mProfiler);

    // Create the callback object that will compute ray casting against triangles
    ConcaveMeshRaycastCallback raycastCallback(mDynamicAABBTree, *this, proxyShape, raycastInfo, ray);

#ifdef IS_PROFILING_ACTIVE

    // Set the profiler
    raycastCallback.setProfiler(mProfiler);

#endif

    // Ask the Dynamic AABB Tree to report all AABB nodes that are hit by the ray.
    // The raycastCallback object will then compute ray casting against the triangles
    // in the hit AABBs.
    mDynamicAABBTree.Raycast(ray, raycastCallback);

    raycastCallback.raycastTriangles();

    return raycastCallback.getIsHit();
}

// Compute the shape Id for a given triangle of the mesh
u32 ConcaveMeshShape::computeTriangleShapeId(u32 subPart, u32 triangleIndex) const
{

    u32 shapeId = 0;

    u32 i=0;
    while (i < subPart)
    {

        shapeId += mTriangleMesh->getSubpart(i)->getNbTriangles();

        i++;
    }

    return shapeId + triangleIndex;
}

// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
scalar ConcaveMeshRaycastCallback::RaycastBroadPhaseShape(i32 nodeId, const IRay &ray)
{
    // Add the id of the hit AABB node into
    mHitAABBNodes.add(nodeId);

    return ray.maxFraction;
}

// Raycast all collision shapes that have been collected
void ConcaveMeshRaycastCallback::raycastTriangles()
{

    List<int>::Iterator it;
    scalar smallestHitFraction = mRay.maxFraction;

    for (it = mHitAABBNodes.begin(); it != mHitAABBNodes.end(); ++it) {

        // Get the node data (triangle index and mesh subpart index)
        i32* data = mDynamicAABBTree.GetNodeDataInt(*it);

        // Get the triangle vertices for this node from the concave mesh shape
        Vector3 trianglePoints[3];
        mConcaveMeshShape.getTriangleVertices(data[0], data[1], trianglePoints);

        // Get the vertices normals of the triangle
        Vector3 verticesNormals[3];
        mConcaveMeshShape.getTriangleVerticesNormals(data[0], data[1], verticesNormals);

        // Create a triangle collision shape
        TriangleShape triangleShape(trianglePoints, verticesNormals, mConcaveMeshShape.computeTriangleShapeId(data[0], data[1]));
        triangleShape.setRaycastTestType(mConcaveMeshShape.getRaycastTestType());

#ifdef IS_PROFILING_ACTIVE

        // Set the profiler to the triangle shape
        triangleShape.setProfiler(mProfiler);

#endif

        // Ray casting test against the collision shape
        IRaycastInfo raycastInfo;
        bool isTriangleHit = triangleShape.Raycast(mRay, raycastInfo, mProxyShape);

        // If the ray hit the collision shape
        if (isTriangleHit && raycastInfo.hitFraction <= smallestHitFraction)
        {

            assert(raycastInfo.hitFraction >= scalar(0.0));

            mRaycastInfo.body = raycastInfo.body;
            mRaycastInfo.proxyShape = raycastInfo.proxyShape;
            mRaycastInfo.hitFraction = raycastInfo.hitFraction;
            mRaycastInfo.worldPoint = raycastInfo.worldPoint;
            mRaycastInfo.worldNormal = raycastInfo.worldNormal;
            mRaycastInfo.meshSubpart = data[0];
            mRaycastInfo.triangleIndex = data[1];

            smallestHitFraction = raycastInfo.hitFraction;
            mIsHit = true;
        }

    }
}

}
