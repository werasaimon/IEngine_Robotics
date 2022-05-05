#ifndef ICONCAVESHAPE_H
#define ICONCAVESHAPE_H

// Libraries
#include "ICollisionShape.h"
#include "ITriangleShape.h"

namespace IEngine
{

// Class TriangleCallback
/**
 * This class is used to encapsulate a callback method for
 * a single triangle of a ConcaveMesh.
 */
class TriangleCallback {

    public:

        /// Destructor
        virtual ~TriangleCallback() = default;

        /// Report a triangle
        virtual void testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, u32 shapeId)=0;

};


// Class ConcaveShape
/**
 * This abstract class represents a concave collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConcaveShape : public ICollisionShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Raycast test type for the triangle (front, back, front-back)
        TriangleRaycastSide mRaycastTestType;

        // -------------------- Methods -------------------- //

        /// Return true if a point is inside the collision shape
        virtual bool TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveShape();

        /// Destructor
        virtual ~ConcaveShape() override = default;

        /// Deleted copy-constructor
        ConcaveShape(const ConcaveShape& shape) = delete;

        /// Deleted assignment operator
        ConcaveShape& operator=(const ConcaveShape& shape) = delete;

        /// Return the raycast test type (front, back, front-back)
        TriangleRaycastSide getRaycastTestType() const;

        // Set the raycast test type (front, back, front-back)
        void setRaycastTestType(TriangleRaycastSide testType);

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const;// override;

        /// Return true if the collision shape is a polyhedron
        virtual bool isPolyhedron() const;// override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const IAABBox3D& localAABB) const=0;
};

// Return true if the collision shape is convex, false if it is concave
inline bool ConcaveShape::isConvex() const {
    return false;
}

// Return true if the collision shape is a polyhedron
inline bool ConcaveShape::isPolyhedron() const {
    return true;
}

// Return true if a point is inside the collision shape
inline bool ConcaveShape::TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const {
    return false;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
    return mRaycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void ConcaveShape::setRaycastTestType(TriangleRaycastSide testType) {
    mRaycastTestType = testType;
}

}

#endif // ICONCAVESHAPE_H
