#ifndef ICOLLISIONSHAPEINFO_H
#define ICOLLISIONSHAPEINFO_H


#include "../IProxyShape.h"


namespace IEngine
{


class IOverlappingPair;
class ICollisionShape;

// Class rpCollisionShapeInfo
/**
 * This structure regroups different things about a collision shape. This is
 * used to pass information about a collision shape to a collision algorithm.
 */
struct ICollisionShapeInfo
{

    public:

    /// Broadphase overlapping pair
    //IOverlappingPair* overlappingPair;

    /// Proxy shape
    //IProxyShape* proxyShape;

    /// Pointer to the collision shape
    const ICollisionShape* collisionShape;

    /// Transform that maps from collision shape local-space to world-space
    const Transform        shapeToWorldTransform;

    /// Cached collision data of the proxy shape
    void** cachedCollisionData;

    /// Constructor
    ICollisionShapeInfo(const ICollisionShape* _CollisionShape,
                        const Transform& shapeLocalToWorldTransform,
                        void** cachedData)
        : collisionShape(_CollisionShape),
          shapeToWorldTransform(shapeLocalToWorldTransform) ,
          cachedCollisionData(cachedData)
    {


    }


    // Return a local support point in a given direction with the object margin
    Vector3 GetLocalSupportPointWithMargin(const Vector3 &direction ) const
    {
        return collisionShape->GetLocalSupportPointWithMargin(direction);
    }

    /**

    // Return a local support point in a given direction with the object margin
    Vector3 getLocalSupportPointWithoutMargin(const Vector3 &direction ) const
    {
          Vector3 supportPoint = collisionShape->getLocalSupportPointWithoutMargin(direction,NULL);

          // Add the margin to the support point
          Vector3 unitVec(0.0, -1.0, 0.0);
          if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON)
          {
              unitVec = direction.getUnit();
          }
          supportPoint += unitVec * 0.08;


          return collisionShape->getLocalSupportPointWithoutMargin(direction,NULL);
    }

    /**/


     // Return a world support point in a given direction
    Vector3 GetWorldSupportPointWithMargin(const Vector3 &direction) const
    {
        return shapeToWorldTransform * GetLocalSupportPointWithMargin(shapeToWorldTransform.GetBasis().GetTranspose() * direction );
    }


    const Transform& GetWorldTransform() const
    {
        return shapeToWorldTransform;
    }


    Vector3 getScale() const
    {
        return collisionShape->GetScaling();
    }
};


}


#endif // ICOLLISIONSHAPEINFO_H
