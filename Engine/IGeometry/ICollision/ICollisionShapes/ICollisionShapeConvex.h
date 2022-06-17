#ifndef ICOLLISIONSHAPECONVEX_H
#define ICOLLISIONSHAPECONVEX_H

#include "ICollisionShape.h"

namespace IEngine
{


/// Object margin for collision detection in meters (for the GJK-EPA Algorithm)
const scalar OBJECT_MARGIN = scalar(0.002);

// Class ConvexShape
/**
* This abstract class represents a convex collision shape associated with a
* body that is used during the narrow-phase collision detection.
*/
class ICollisionShapeConvex : public ICollisionShape
{

   protected :

       // -------------------- Attributes -------------------- //

       /// Margin used for the GJK collision detection algorithm
       scalar mMargin;

       // -------------------- Methods -------------------- //

       /// Private copy-constructor
       ICollisionShapeConvex(const ICollisionShapeConvex& shape);

       /// Private assignment operator
       ICollisionShapeConvex& operator=(const ICollisionShapeConvex& shape);


   public:

       /// Return a local support point in a given direction with the object margin
       virtual Vector3 GetLocalSupportPointWithMargin(const Vector3& direction) const = 0;

       /// Return a local support point in a given direction without the object margin
       Vector3 GetLocalSupportPointWithoutMargin(const Vector3& direction) const;

       /// Return true if a point is inside the collision shape
       virtual bool TestPointInside(const Vector3& worldPoint, IProxyShape* proxyShape) const=0;

   public :


       // -------------------- Methods -------------------- //

       /// Constructor
       ICollisionShapeConvex(CollisionShapeType type, scalar margin);

       /// Destructor
       virtual ~ICollisionShapeConvex();

       /// Return the current object margin
       scalar GetMargin() const;

       /// Return true if the collision shape is convex, false if it is concave
       virtual bool IsConvex() const;

       // -------------------- Friendship -------------------- //

       friend class  ICollisionBody;
};

/// Return true if the collision shape is convex, false if it is concave
SIMD_INLINE bool ICollisionShapeConvex::IsConvex() const
{
   return true;
}

// Return the current collision shape margin
/**
* @return The margin (in meters) around the collision shape
*/
SIMD_INLINE scalar ICollisionShapeConvex::GetMargin() const
{
   return mMargin;
}


}

#endif // ICOLLISIONSHAPECONVEX_H
