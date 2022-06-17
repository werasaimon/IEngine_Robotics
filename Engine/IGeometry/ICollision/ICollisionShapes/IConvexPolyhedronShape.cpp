#include "IConvexPolyhedronShape.h"

namespace IEngine
{


// Constructor
ConvexPolyhedronShape::ConvexPolyhedronShape()
            : ICollisionShapeConvex(CONCAVE_MESH, OBJECT_MARGIN)
{

}

// Find and return the index of the polyhedron face with the most anti-parallel face
// normal given a direction vector. This is used to find the incident face on
// a polyhedron of a given reference face of another polyhedron
u32 ConvexPolyhedronShape::findMostAntiParallelFace(const Vector3& direction) const {

    scalar minDotProduct = DECIMAL_LARGEST;
    u32 mostAntiParallelFace = 0;

    // For each face of the polyhedron
    for (u32 i=0; i < getNbFaces(); i++)
    {

        // Get the face normal
        scalar dotProduct = getFaceNormal(i).Dot(direction);
        if (dotProduct < minDotProduct) {
            minDotProduct = dotProduct;
            mostAntiParallelFace = i;
        }
    }

    return mostAntiParallelFace;
}

}
