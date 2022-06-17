#ifndef ITRIANGLEMESH_H
#define ITRIANGLEMESH_H


// Libraries
#include <cassert>
#include "IHalfEdgeStructure.h"


namespace IEngine
{
// Declarations
class TriangleVertexArray;
class MemoryManager;

// Class TriangleMesh
/**
 * This class represents a mesh made of triangles. A TriangleMesh contains
 * one or several parts. Each part is a set of triangles represented in a
 * TriangleVertexArray object describing all the triangles vertices of the part.
 * A TriangleMesh object can be used to create a ConcaveMeshShape from a triangle
 * mesh for instance.
 */
class TriangleMesh
{

    protected:

        /// All the triangle arrays of the mesh (one triangle array per part)
        List<TriangleVertexArray*> mTriangleArrays;

    public:

        /// Constructor
         TriangleMesh();

        /// Destructor
        ~TriangleMesh() = default;

        /// Add a subpart of the mesh
        void addSubpart(TriangleVertexArray* triangleVertexArray);

        /// Return a pointer to a given subpart (triangle vertex array) of the mesh
        TriangleVertexArray* getSubpart(u32 indexSubpart) const;

        /// Return the number of subparts of the mesh
        u32 getNbSubparts() const;
};

// Add a subpart of the mesh
/**
 * @param triangleVertexArray Pointer to the TriangleVertexArray to add into the mesh
 */
inline void TriangleMesh::addSubpart(TriangleVertexArray* triangleVertexArray)
{
    mTriangleArrays.add(triangleVertexArray );
}

// Return a pointer to a given subpart (triangle vertex array) of the mesh
/**
 * @param indexSubpart The index of the sub-part of the mesh
 * @return A pointer to the triangle vertex array of a given sub-part of the mesh
 */
inline TriangleVertexArray* TriangleMesh::getSubpart(u32 indexSubpart) const
{
   assert(indexSubpart < mTriangleArrays.size());
   return mTriangleArrays[indexSubpart];
}

// Return the number of sub-parts of the mesh
/**
 * @return The number of sub-parts of the mesh
 */
inline u32 TriangleMesh::getNbSubparts() const
{
    return mTriangleArrays.size();
}

}

#endif // ITRIANGLEMESH_H
