#ifndef ICONTACTGENERATOR_H
#define ICONTACTGENERATOR_H

#include "../INarrowPhase/ICollisionAlgorithm.h"
#include "../INarrowPhase/ICollisionShapeInfo.h"
#include "../../../ICommon/IMemory/IMem.h"


namespace IEngine
{

struct IContactVertex
{
    Vector3 normal;
    scalar  penetration;

    Vector3 point1;
    Vector3 point2;


    //------------------ Constructor --------------------//
    /// Constructor
    IContactVertex(void){}

    /// Constructor
    IContactVertex(const Vector3& normal, scalar penetration,
                   const Vector3& Point1,
                   const Vector3& Point2)
        : normal(normal),
          penetration(penetration),
          point1(Point1),
          point2(Point2)
    {

    }

};




class IContactGenerator
{

private:

    //-------------------- Attributes --------------------//

    Vector3  mCachedSeparatingAxis;

    const IProxyShape  *mShape1;
    const IProxyShape  *mShape2;

    ICollisionAlgorithm    *mCollisionAlgorithm;

    //--------------- contact points ---------------------//
    i32                     mNbContactPoints;
    IContactVertex*         mContactPoints;


    // -------------------- Methods -------------------- //
    /// Private copy-constructor
    IContactGenerator(const IContactGenerator& GenerationContactPoints);

    /// Private assignment operator
    IContactGenerator& operator=(const IContactGenerator& GenerationContactPoints);


public:


    IContactGenerator(){}

    IContactGenerator(const IProxyShape* shape1 ,
                      const IProxyShape* shape2 , ICollisionAlgorithm *CollisionAlgorithm = nullptr)
        : mCachedSeparatingAxis(0.0,1.0,0.0),
          mShape1(shape1) ,
          mShape2(shape2) ,
          mCollisionAlgorithm(CollisionAlgorithm),
          mNbContactPoints(0)

    {

    }

   ~IContactGenerator()
    {
        mNbContactPoints = 0;
        IFree(mContactPoints);
    }



    i32 GetCountContactPoints() const
    {
        return mNbContactPoints;
    }

    IContactVertex *GetContactPoints() const
    {
        return mContactPoints;
    }

    Vector3 GetCachedSeparatingAxis() const
    {
        return mCachedSeparatingAxis;
    }

    void SetCachedSeparatingAxis(const Vector3 &cachedSeparatingAxis)
    {
        mCachedSeparatingAxis = cachedSeparatingAxis;
    }


    bool ComputeCollisionAndFindContactPoints();

//private:

    //***************************************************************//

        void AddContact( const  IContactVertex& p_contact );
        void SwapFlip();

        /*****************************************************************
         * Method taken from the site :
         * http://www.gamedev.ru/code/articles/convex_collisions
         *****************************************************************/
        static Vector3 *CreateAxisPeturberationPoints(const IProxyShape* proxy_shape , const Vector3& xAxis ,  i32 &_NbPoints , std::vector<Vector3>& results);
        static void     CreateAxisPeturberationPoints(const IProxyShape* proxy_shape , const Vector3& xAxis , std::vector<Vector3>& results);

        //***************************************************************//

        bool FindPointsToContacts(const Vector3* SupportVertA, i32 iNumVertsA,
                                  const Vector3* SupportVertB, i32 iNumVertsB , const IOutputCollisionInfo& info_collision, const Vector3 &_Normal ,
                                  const ICollisionShapeInfo &shape1Info,
                                  const ICollisionShapeInfo &shape2Info);

        //***************************************************************//

        void FaceToFaceContacts(  const Vector3* Poly , i32 iPolySize,  const Vector3* Clipper, i32 iClipperSize , const Vector3& _FaceNormal , bool flipNormal);
        void EdgeToEdgeContacts(  const Vector3& A0, const Vector3& A1, const Vector3& B0, const Vector3& B1 , const Vector3& _FaceNormal );
        void PointToEdgeContacts( const Vector3&  A, const Vector3& B0, const Vector3& B1 , const Vector3& _FaceNormal );
        void PointToFaceContacts( const Vector3&  A, const Vector3& xAxis, scalar bd , const Vector3& _FaceNormal );
        void PointToPointContacts(const Vector3&  A, const Vector3& B , const Vector3& _FaceNormal );

        //***************************************************************//



};


}

#endif // ICONTACTGENERATOR_H
