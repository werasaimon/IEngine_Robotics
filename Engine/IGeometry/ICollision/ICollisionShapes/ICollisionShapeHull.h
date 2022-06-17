#ifndef ICOLLISIONSHAPEHULL_H
#define ICOLLISIONSHAPEHULL_H

#include "ICollisionShapeConvex.h"
#include "../../../IAlgorithm/IQuickHull.hpp"



namespace IEngine
{


using namespace  IAlgorithm;

    namespace
    {
        static IQuickHull<scalar>  QuickHullAlgorithm;
    }


struct ICalcModelHull
{

    //------------ Attribute -----------//
     IConvexHull<scalar> mConvexHull;

 public:

    ICalcModelHull( const Vector3 *axVertices , unsigned int NbCount )
    {
        mConvexHull = QuickHullAlgorithm.getConvexHull( axVertices , NbCount , true, false);
    }


    ICalcModelHull( std::vector<Vector3> Vertices )
    {
        mConvexHull = QuickHullAlgorithm.getConvexHull( Vertices , true, false);
    }


    ~ICalcModelHull()
    {
        mConvexHull.getIndexBuffer().clear();
    }

};


struct ITriangleIndex
{
    u32 index_a;
    u32 index_b;
    u32 index_c;
};



struct IHullDescriptor
{
    u32             NbVertices;
    const Vector3  *Vertices=nullptr;

    u32   NbIndexes;
    lu32 *Indexes=nullptr;

IHullDescriptor(){}

    IHullDescriptor(ICalcModelHull* clacHull) :
        NbVertices(clacHull->mConvexHull.getVertexBuffer().size()) , Vertices(clacHull->mConvexHull.getVertexBuffer().data()) ,
        NbIndexes(clacHull->mConvexHull.getIndexBuffer().size()) , Indexes((unsigned long*)clacHull->mConvexHull.getIndexBuffer().data())
    {


    }
};



class ICollisionShapeHull: public ICollisionShapeConvex
{

private:

    //-------------------- Attributes --------------------//
    u32            mNbVertices;
    const Vector3 *mVertices=nullptr;

    u32   mNbIndexes;
    lu32 *mIndexes=nullptr;


protected :

    /// Private copy-constructor
    ICollisionShapeHull(const ICollisionShapeHull& shape);

    /// Private assignment operator
    ICollisionShapeHull& operator=(const ICollisionShapeHull& shape);




    /// Return a local support interval ( minimum , maximum )
    void GetIntervalLocal(const Vector3 &xAxis, scalar &min, scalar &max) const;


    /// Return a local support point in a given direction without the object margin
    virtual Vector3 GetLocalSupportPointWithMargin(const Vector3& direction ) const;

    /// Return true if a point is inside the collision shape
    virtual bool TestPointInside(const Vector3& localPoint, IProxyShape* proxyShape) const;

    /// Raycast method with feedback information
    virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const;

    /// Return the number of bytes used by the collision shape
    virtual size_t GetSizeInBytes() const;






public:


//    virtual ICollisionShape *Clone()
//    {
//        lu32      *Indexes = new lu32[mNbIndexes];
//        Vector3 *Vertices = new Vector3[mNbVertices];
//        ICollisionShapeHull *res = new ICollisionShapeHull(mNbVertices,Vertices,mNbIndexes,Indexes);
//        res->Copy(this);
//        return res;
//    }

//    void Copy( const ICollisionShape* _shape )
//    {
//        ICollisionShape::Copy(_shape);

//        auto shape = static_cast<const ICollisionShapeHull*>(_shape);

//        mNbIndexes = shape->mNbIndexes;
//        mNbVertices = shape->mNbVertices;

//        std::memcpy(mIndexes,shape->mIndexes,shape->mNbIndexes);
//        std::memcpy(&mVertices,shape->mVertices,shape->mNbVertices);
//    }

    // -------------------- Methods -------------------- //

   ICollisionShapeHull(const IHullDescriptor &_HullDescriptor , scalar margin = OBJECT_MARGIN );
   ICollisionShapeHull(u32 _NbVertices , const Vector3 *_Vertices ,
                       u32 _NbIndexes  , lu32 *_Indexes , scalar margin = OBJECT_MARGIN );

    virtual ~ICollisionShapeHull();


    /// Set the scaling vector of the collision shape
    virtual void SetLocalScaling(const Vector3& scaling);

    /// Return the local bounds of the shape in x, y and z directions
    virtual void GetLocalBounds(Vector3& min, Vector3& max) const;

    /// Return the local inertia tensor of the collision shape
    //virtual void ComputeLocalInertiaTensor(Matrix3& tensor, scalar mass) const;
    virtual Matrix3 ComputeLocalInertiaTensor2(scalar mass , const Matrix3& transform) const;

};



}


#endif // ICOLLISIONSHAPEHULL_H
