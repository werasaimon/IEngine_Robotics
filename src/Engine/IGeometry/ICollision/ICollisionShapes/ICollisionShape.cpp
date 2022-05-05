#include "ICollisionShape.h"

//#include <GL/gl.h>
//#include <GL/glu.h>

namespace IEngine
{

// Constructor
ICollisionShape::ICollisionShape(CollisionShapeType type)
: mType(type),
  mScaling(1.0, 1.0, 1.0)
{
//    mNbMaxPeturberationIteration = 10;
//    mEpsilonPeturberation = 0.001;
}

// Destructor
ICollisionShape::~ICollisionShape()
{

}



// Compute the world-space AABB of the collision shape given a transform
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
void ICollisionShape::ComputeAABB( IAABBox3D& aabb, const Transform &_transform ) const
{

    Transform transform = _transform;


    // Get the local bounds in x,y and z direction
    Vector3 minBounds;
    Vector3 maxBounds;
    GetLocalBounds(minBounds, maxBounds);


    /// Scale size AABB local
    minBounds *= 1.05f;
    maxBounds *= 1.05f;


  //      minBounds = minBounds * mScaling;
  //      maxBounds = maxBounds * mScaling;



    // Rotate the local bounds according to the orientation of the body
    Matrix3   worldAxis =  (transform.GetBasis()).GetAbsoluteMatrix();
    Vector3   worldMinBounds(worldAxis.GetRow(0).Dot(minBounds),
                              worldAxis.GetRow(1).Dot(minBounds),
                              worldAxis.GetRow(2).Dot(minBounds));
    Vector3   worldMaxBounds(worldAxis.GetRow(0).Dot(maxBounds),
                              worldAxis.GetRow(1).Dot(maxBounds),
                              worldAxis.GetRow(2).Dot(maxBounds));





    // Compute the minimum and maximum coordinates of the rotated extents
     Vector3 minCoordinates = transform.GetPosition() + worldMinBounds;
     Vector3 maxCoordinates = transform.GetPosition() + worldMaxBounds;


//     Vector3 pos = transform.GetPosition();
//     Vector3 halfSize = minCoordinates - maxCoordinates;
//     glPushMatrix();
//     glColor3f(0,1,0);
//     glTranslatef( pos.x , pos.y , pos.z );
//     glScalef( halfSize.x , halfSize.y , halfSize.z );
//     glutWireCube(1.0);
//     glPopMatrix();

    // Update the AABB with the new minimum and maximum coordinates
    aabb.SetMin(minCoordinates);
    aabb.SetMax(maxCoordinates);
}

//void ICollisionShape::PairCollisionDetected(IOverlappingPair *collid_info)
//{
//}


}
