#include "ICollisionShapeBox.h"
#include "../IProxyShape.h"

#include "../../../ICommon/ISettings.h"

namespace IEngine
{

// Default initilization
#define DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS	      4
#define DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT   0.08

// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */

ICollisionShapeBox::ICollisionShapeBox(const Vector3 &extent)
    : ICollisionShapeConvex(BOX, OBJECT_MARGIN) ,
      mExtent(extent * 0.5 - Vector3(OBJECT_MARGIN,OBJECT_MARGIN,OBJECT_MARGIN))
{

    assert(extent.x > scalar(0.0) && extent.x > OBJECT_MARGIN);
    assert(extent.y > scalar(0.0) && extent.y > OBJECT_MARGIN);
    assert(extent.z > scalar(0.0) && extent.z > OBJECT_MARGIN);

    mNbMaxPeturberationIteration = DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation =  DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}


// Constructor
/**
 * @param extent The vector with the three extents of the box (in meters)
 * @param margin The collision margin (in meters) around the collision shape
 */
ICollisionShapeBox::ICollisionShapeBox(const Vector3& extent, scalar margin)
: ICollisionShapeConvex(BOX, margin) ,
  mExtent(extent * 0.5 - Vector3(margin, margin, margin))
{

    assert(extent.x > scalar(0.0) && extent.x > margin);
    assert(extent.y > scalar(0.0) && extent.y > margin);
    assert(extent.z > scalar(0.0) && extent.z > margin);

    mNbMaxPeturberationIteration = DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation =  DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}

// Destructor
ICollisionShapeBox::~ICollisionShapeBox()
{

}

// Return the local inertia tensor of the collision shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
//void ICollisionShapeBox::ComputeLocalInertiaTensor(Matrix3 &tensor, scalar mass) const
//{
//    scalar factor = (scalar(1.0) / scalar(3.0)) * mass;
//    Vector3 realExtent = mExtent + Vector3(mMargin, mMargin, mMargin);
//    scalar xSquare = realExtent.x * realExtent.x;
//    scalar ySquare = realExtent.y * realExtent.y;
//    scalar zSquare = realExtent.z * realExtent.z;
//    tensor = Matrix3( factor * (ySquare + zSquare), 0.0, 0.0,
//                         0.0, factor * (xSquare + zSquare), 0.0,
//                         0.0, 0.0, factor * (xSquare + ySquare) );
//}

SIMD_INLINE Matrix3 ICollisionShapeBox::ComputeLocalInertiaTensor2(scalar mass, const Matrix3& transform) const
{

    scalar factor = (scalar(1.0) / scalar(3.0)) * mass;
    Vector3 realExtent = mExtent + Vector3(mMargin, mMargin, mMargin);
    scalar xSquare = realExtent.x * realExtent.x;
    scalar ySquare = realExtent.y * realExtent.y;
    scalar zSquare = realExtent.z * realExtent.z;

    Matrix3 worldAxis = transform;
    xSquare *= worldAxis.GetRow(0).Length();
    ySquare *= worldAxis.GetRow(1).Length();
    zSquare *= worldAxis.GetRow(2).Length();

    Matrix3 tensor( factor * (ySquare + zSquare), 0.0, 0.0,
                       0.0, factor * (xSquare + zSquare), 0.0,
                       0.0, 0.0, factor * (xSquare + ySquare) );
    return tensor;

}



bool ICollisionShapeBox::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

      Vector3 min = -mExtent;
      Vector3 max =  mExtent;

       Vector3 T_1, T_2; // vectors to hold the T-values for every direction
       double t_min = -DBL_MAX; // maximums defined in float.h
       double t_max = DBL_MAX;

       for (int i = 0; i < 3; i++)
       {
           //we test slabs in every direction
           if (ray.Direction[i] == 0)
           { // ray parallel to planes in this direction
               if ((ray.Origin[i] < min[i]) || (ray.Origin[i] > max[i]))
               {
                   return false; // parallel AND outside box : no intersection possible
               }
           }
           else
           { // ray not parallel to planes in this direction
               T_1[i] = (min[i] - ray.Origin[i]) / ray.Direction[i];
               T_2[i] = (max[i] - ray.Origin[i]) / ray.Direction[i];

               if(T_1[i] > T_2[i])
               {
                   // we want T_1 to hold values for intersection with near plane
                   std::swap(T_1,T_2);
               }

               if (T_1[i] > t_min)
               {
                   t_min = T_1[i];
               }

               if (T_2[i] < t_max)
               {
                   t_max = T_2[i];
               }

               if( (t_min > t_max) || (t_max < 0) )
               {
                   return false;
               }
           }
       }

      scalar tMin = t_min;
      scalar tMax = t_max;





    /**

    Vector3 rayDirection = ray.Direction;
    scalar tMin = 0;//DECIMAL_SMALLEST;
    scalar tMax = 0;//DECIMAL_LARGEST;
    Vector3 normalDirection(scalar(0), scalar(0), scalar(0));
    Vector3 currentNormal;

    // For each of the three slabs
    for (i32 i=0; i<3; i++)
    {
        // If ray is parallel to the slab
        if (IAbs(rayDirection[i]) < MACHINE_EPSILON)
        {
            // If the ray's origin is not inside the slab, there is no hit
            if (ray.Origin[i] > mExtent[i] || ray.Origin[i] < -mExtent[i]) return false;
        }
        else
        {

            // Compute the intersection of the ray with the near and far plane of the slab
            scalar oneOverD = scalar(1.0) / rayDirection[i];
            scalar t1 = (-mExtent[i] - ray.Origin[i]) * oneOverD;
            scalar t2 = (mExtent[i] - ray.Origin[i]) * oneOverD;
            currentNormal[0] = (i == 0) ? -mExtent[i] : scalar(0.0);
            currentNormal[1] = (i == 1) ? -mExtent[i] : scalar(0.0);
            currentNormal[2] = (i == 2) ? -mExtent[i] : scalar(0.0);

            // Swap t1 and t2 if need so that t1 is intersection with near plane and
            // t2 with far plane
            if (t1 > t2)
            {
                ISwap(t1, t2);
                currentNormal = -currentNormal;
            }

            // Compute the intersection of the of slab intersection interval with previous slabs
            if (t1 > tMin)
            {
                tMin = t1;
                normalDirection = currentNormal;
            }
            tMax = IMin(tMax, t2);

            // If tMin is larger than the maximum raycasting fraction, we return no hit
            if (tMin > ray.maxFraction) return false;

            // If the slabs intersection is empty, there is no hit
            if (tMin > tMax) return false;
        }
    }


    /**/
    // If tMin is negative, we return no hit
   // if (tMin < scalar(0.0) || tMin > ray.maxFraction) return false;

    // The ray intersects the three slabs, we compute the hit point
    Vector3 localHitPoint = ray.Origin + tMin * ray.Direction;

    /**/

    raycastInfo.body        = proxyShape->GetBody();
    raycastInfo.proxyShape  = proxyShape;
    raycastInfo.hitFraction = tMin;
    raycastInfo.worldPoint  = localHitPoint;
    raycastInfo.worldNormal = ray.Direction;

    return true;
}



#undef DEFAULT_BOX_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_BOX_EPS_PETURBERATION_ANGLES_COFFICIENT

}
