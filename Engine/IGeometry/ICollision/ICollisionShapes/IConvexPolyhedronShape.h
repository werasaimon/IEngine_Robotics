#ifndef ICONVEXPOLYHEDRONSHAPE_H
#define ICONVEXPOLYHEDRONSHAPE_H

#include "ICollisionShapeConvex.h"
#include "IHalfEdgeStructure.h"

namespace IEngine
{

// Class ConvexPolyhedronShape
/**
 * This abstract class represents a convex polyhedron collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConvexPolyhedronShape : public ICollisionShapeConvex {

    protected :

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexPolyhedronShape();

        /// Destructor
        virtual ~ConvexPolyhedronShape() override = default;

        /// Deleted copy-constructor
        ConvexPolyhedronShape(const ConvexPolyhedronShape& shape) = delete;

        /// Deleted assignment operator
        ConvexPolyhedronShape& operator=(const ConvexPolyhedronShape& shape) = delete;

        /// Return the number of faces of the polyhedron
        virtual u32 getNbFaces() const=0;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(u32 faceIndex) const=0;

        /// Return the number of vertices of the polyhedron
        virtual u32 getNbVertices() const=0;

        /// Return a given vertex of the polyhedron
        virtual HalfEdgeStructure::Vertex getVertex(u32 vertexIndex) const=0;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(u32 vertexIndex) const=0;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(u32 faceIndex) const=0;

        /// Return the number of half-edges of the polyhedron
        virtual u32 getNbHalfEdges() const=0;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(u32 edgeIndex) const=0;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const;// override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const=0;

        /// Find and return the index of the polyhedron face with the most anti-parallel face
        /// normal given a direction vector
        u32 findMostAntiParallelFace(const Vector3& direction) const;
};

// Return true if the collision shape is a polyhedron
inline bool ConvexPolyhedronShape::isPolyhedron() const {
    return true;
}

}

#endif // ICONVEXPOLYHEDRONSHAPE_H
