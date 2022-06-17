#include "ICollisionShapeHull.h"
#include "../INarrowPhase/GJK/IGJKIntersection.h"

namespace IEngine
{



namespace
{
   static IGJKIntersection GJKAIntersection;
}


// Default initilization
#define DEFAULT_HULL_MAX_PETURBERATION_ITERATIONS	       10
#define DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT   0.06


ICollisionShapeHull::ICollisionShapeHull(const IHullDescriptor &_HullDescriptor , scalar margin)
: ICollisionShapeHull(_HullDescriptor.NbVertices , _HullDescriptor.Vertices ,
                      _HullDescriptor.NbIndexes  , _HullDescriptor.Indexes , margin)
{
    mNbMaxPeturberationIteration = DEFAULT_HULL_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation = DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation

}

ICollisionShapeHull::ICollisionShapeHull(u32 _NbVertices, const Vector3 *_Vertices,
                                         u32 _NbIndexes, lu32 *_Indexes, scalar margin)
: ICollisionShapeConvex( CONVEX_HULL_MESH , margin ) ,
     mNbVertices(_NbVertices) , mVertices(_Vertices) ,
     mNbIndexes(_NbIndexes) , mIndexes(_Indexes)
{
    mNbMaxPeturberationIteration = DEFAULT_HULL_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation = DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}


ICollisionShapeHull::~ICollisionShapeHull()
{
    mNbVertices = 0;
    mNbIndexes  = 0;

    if(mVertices != NULL )
    {
        delete mVertices;
        mVertices = NULL;
    }

    if(mIndexes != NULL )
    {
        delete mIndexes;
        mIndexes = NULL;
    }

}

size_t ICollisionShapeHull::GetSizeInBytes() const
{
   return sizeof (ICollisionShapeHull);
}



Vector3 ICollisionShapeHull::GetLocalSupportPointWithMargin(const Vector3 &direction) const
{
    unsigned int index = 0;
    scalar max = (mVertices[0].Dot(direction));

    for (unsigned int i = 1; i < mNbVertices; i++)
    {
        scalar d = (mVertices[i].Dot(direction));
        if (d > max)
        {
            max = d;
            index = i;
        }
    }

    return mVertices[index];
}




bool ICollisionShapeHull::TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const
{
    return GJKAIntersection.testPointInside(localPoint, proxyShape);
}


bool ICollisionShapeHull::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{
    return GJKAIntersection.raycast(ray, raycastInfo, proxyShape);
}



void ICollisionShapeHull::SetLocalScaling(const Vector3& scaling)
{
  mScaling = scaling;
}


void ICollisionShapeHull::GetIntervalLocal(const Vector3& xAxis, scalar& min, scalar& max) const
{

    Vector3 s_p0 = GetLocalSupportPointWithMargin( xAxis );
    Vector3 s_p1 = GetLocalSupportPointWithMargin(-xAxis );
    min = s_p1.Dot(xAxis);
    max = s_p0.Dot(xAxis);
}




void ICollisionShapeHull::GetLocalBounds(Vector3& min, Vector3& max) const
{
    GetIntervalLocal( Vector3::X , min.x , max.x);
    GetIntervalLocal( Vector3::Y , min.y , max.y);
    GetIntervalLocal( Vector3::Z , min.z , max.z);
}



//void ICollisionShapeHull::ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const
//{
//    Vector3 min;
//    Vector3 max;
//    GetLocalBounds( min , max );

//    Vector3 halfSize;
//    halfSize.x = IAbs(min.x - max.x) * 0.5;
//    halfSize.y = IAbs(min.y - max.y) * 0.5;
//    halfSize.z = IAbs(min.z - max.z) * 0.5;


//    scalar  factor = (scalar(1.0) / scalar(3.0)) * mass;
//    Vector3 realExtent = halfSize + Vector3(mMargin, mMargin, mMargin);
//    scalar  xSquare = realExtent.x * realExtent.x;
//    scalar  ySquare = realExtent.y * realExtent.y;
//    scalar  zSquare = realExtent.z * realExtent.z;
//    tensor.SetAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
//                        0.0, factor * (xSquare + zSquare), 0.0,
//                        0.0, 0.0, factor * (xSquare + ySquare));

//}

SIMD_INLINE Matrix3 ICollisionShapeHull::ComputeLocalInertiaTensor2(scalar mass, const Matrix3& transform) const
{
    Vector3 min;
    Vector3 max;
    GetLocalBounds( min , max );


    Vector3 halfSize;
    halfSize.x = IAbs(min.x - max.x) * 0.5;
    halfSize.y = IAbs(min.y - max.y) * 0.5;
    halfSize.z = IAbs(min.z - max.z) * 0.5;


    Matrix3 worldAxis = transform;
    halfSize.x *= worldAxis.GetRow(0).Length();
    halfSize.y *= worldAxis.GetRow(1).Length();
    halfSize.z *= worldAxis.GetRow(2).Length();

    scalar  factor = (scalar(1.0) / scalar(3.0)) * mass;
    Vector3 realExtent = halfSize + Vector3(mMargin, mMargin, mMargin);
    scalar  xSquare = realExtent.x * realExtent.x;
    scalar  ySquare = realExtent.y * realExtent.y;
    scalar  zSquare = realExtent.z * realExtent.z;
    Matrix3 tensor(factor * (ySquare + zSquare), 0.0, 0.0,
                      0.0, factor * (xSquare + zSquare), 0.0,
                      0.0, 0.0, factor * (xSquare + ySquare));
    return tensor;
}


#undef DEFAULT_DEFAULYHULL_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_HULL_EPS_PETURBERATION_ANGLES_COFFICIENT


}
