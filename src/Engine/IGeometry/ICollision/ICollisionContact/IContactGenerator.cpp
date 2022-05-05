#include "IContactGenerator.h"
#include "../../../IAlgorithm/QuickHull/QuickHull.hpp"
#include "../../../IAlgorithm/IQuickClipping.h"

#include "../../../ICommon/IMemory/IMem.h"
#include "../../Segments/IPlane.h"

//#include <GL/gl.h>
//#include <GL/glu.h>



namespace IEngine
{

using namespace IMath;
using namespace IEngine;
using namespace IAlgorithm;


//void drawSphere(double r, int lats, int longs)
//{
//    int i, j;
//    for(i = 0; i <= lats; i++)
//    {
//        double lat0 = M_PI * (-0.5 + (double) (i - 1) / lats);
//        double z0  = sin(lat0);
//        double zr0 =  cos(lat0);

//        double lat1 = M_PI * (-0.5 + (double) i / lats);
//        double z1 = sin(lat1);
//        double zr1 = cos(lat1);

//        glBegin(GL_QUAD_STRIP);
//        for(j = 0; j <= longs; j++)
//        {
//            double lng = 2 * M_PI * (double) (j - 1) / longs;
//            double x = cos(lng);
//            double y = sin(lng);

//            glNormal3f(x * zr0, y * zr0, z0);
//            glVertex3f(r * x * zr0, r * y * zr0, r * z0);
//            glNormal3f(x * zr1, y * zr1, z1);
//            glVertex3f(r * x * zr1, r * y * zr1, r * z1);
//        }
//        glEnd();
//    }
//}

const scalar squarePersistentContactThreshold = PERSISTENT_CONTACT_DIST_THRESHOLD * PERSISTENT_CONTACT_DIST_THRESHOLD;

bool IContactGenerator::ComputeCollisionAndFindContactPoints()
{

    const ICollisionShapeInfo shape1Info( mShape1->GetCollisionShape() , mShape1->GetWorldTransform() , nullptr);
    const ICollisionShapeInfo shape2Info( mShape2->GetCollisionShape() , mShape2->GetWorldTransform() , nullptr);

    mNbContactPoints = 0;
    mContactPoints = nullptr;
    mCollisionAlgorithm->mOutputCollisionInfo.Normal = mCachedSeparatingAxis;

    if ( mCollisionAlgorithm->TestCollision(shape2Info ,shape1Info) )
    {

        IOutputCollisionInfo info_collision = mCollisionAlgorithm->GetOutputCollisionInfo();

        const Vector3& SeparatonNormal = info_collision.Normal.GetUnit();

        mCachedSeparatingAxis = SeparatonNormal;


        /**/
        Vector3 SeparatonNormalA =  SeparatonNormal;
        Vector3 SeparatonNormalB = -SeparatonNormal;

//        i32 iNumVertsA = 0;
//        i32 iNumVertsB = 0;
//        Vector3* SupportVertA = nullptr;
//        Vector3* SupportVertB = nullptr;


//        std::vector<Vector3> results_a;
//        std::vector<Vector3> results_b;
//        SupportVertA = CreateAxisPeturberationPoints(mShape1 , SeparatonNormalA , iNumVertsA ,results_a);
//        SupportVertB = CreateAxisPeturberationPoints(mShape2 , SeparatonNormalB , iNumVertsB ,results_b);



        /**/
        std::vector<Vector3> results_a;
        std::vector<Vector3> results_b;
        CreateAxisPeturberationPoints(mShape1 , SeparatonNormalA , results_a);
        CreateAxisPeturberationPoints(mShape2 , SeparatonNormalB , results_b);
        /**/

        // Find contact points
        bool isCollide = FindPointsToContacts(results_a.data() , results_a.size() ,
                                              results_b.data() , results_b.size() ,
                                              info_collision ,
                                              SeparatonNormal ,
                                              shape1Info ,
                                              shape2Info);


//        IFree(SupportVertA);
//        IFree(SupportVertB);


        //--------------------------------------------------------------//

//        std::cout << "iNumVertsA: " << iNumVertsA << std::endl;
//        std::cout << "iNumVertsB: " << iNumVertsB << std::endl;

//                        for(int i=0; i < results_a.size(); ++i)
//                        {
//                            Vector3 pos = results_a[i];
//                //            std::cout << pos << std::endl;

//                            glPushMatrix();
//                            glColor3f(1,1,0);

//                            glTranslatef(pos.x,pos.y,pos.z);
//                            drawSphere(0.1,10,10);


//                            //glutWireSphere(0.4,10,10);
//                            glPopMatrix();
//                        }

                //        for(uint i=0; i < results_b.size(); ++i)
                //        {
                //            Vector3 pos = results_b[i];

                //  //          std::cout << pos << std::endl;
                //            glPushMatrix();
                //            glColor3f(1,1,0);
                //            glTranslatef(pos.x,pos.y,pos.z);
                //            glutWireSphere(0.2,10,10);
                //            glPopMatrix();
                //        }

//        for (i32 i = 0; i < mNbContactPoints; ++i)
//        {
//            Vector3 pos1 = mContactPoints[i].point1;
//            glPushMatrix();
//            glColor3f(0,0,1);
//            glTranslatef(pos1.x,pos1.y,pos1.z);
//            drawSphere(0.1,10,10);
//            //glutWireSphere(0.1,10,10);
//            glPopMatrix();


//            Vector3 pos2 = mContactPoints[i].point2;
//            glPushMatrix();
//            glColor3f(0,0,1);
//            glTranslatef(pos2.x,pos2.y,pos2.z);
//            drawSphere(0.1,10,10);
//            //glutWireSphere(0.1,10,10);
//            glPopMatrix();
//        }

        //--------------------------------------------------------------//

        return isCollide;

    }

    return false;
}


bool IContactGenerator::FindPointsToContacts(const Vector3 *SupportVertA, i32 iNumVertsA,
                                                    const Vector3 *SupportVertB, i32 iNumVertsB,
                                                    const IOutputCollisionInfo &info_collision, const Vector3 &_Normal,
                                                    const ICollisionShapeInfo &shape1Info,
                                                    const ICollisionShapeInfo &shape2Info)
{

    if (iNumVertsA == 0 || iNumVertsB == 0)
    return false;

    mNbContactPoints = 0;
    mContactPoints = nullptr;

    const Vector3 SeparatonNormal = _Normal.GetUnit();

    /**/

    if (iNumVertsA == 1)
    {
        if (iNumVertsB == 1)
        {
            PointToPointContacts(info_collision.Point1, info_collision.Point2 , SeparatonNormal);
        }
        else if (iNumVertsB == 2)
        {
            PointToEdgeContacts(SupportVertA[0], SupportVertB[0],SupportVertB[1] , SeparatonNormal);
        }
        else
        {
            Vector3 Bn = Vector3::triNormal(SupportVertB[0],
                                              SupportVertB[1],
                                              SupportVertB[2]);

            scalar bd = Bn.Dot(SupportVertB[0]);

            PointToFaceContacts(SupportVertA[0], Bn, bd , SeparatonNormal);
        }
    }
    else if (iNumVertsA == 2)
    {
        if (iNumVertsB == 1)
        {
            PointToEdgeContacts(SupportVertB[0],
                                SupportVertA[0],
                                SupportVertA[1],
                                SeparatonNormal);

        }
        else if (iNumVertsB == 2)
        {
            EdgeToEdgeContacts(SupportVertA[0], SupportVertA[1],
                               SupportVertB[0], SupportVertB[1],
                               SeparatonNormal);

        }
        else
        {
           FaceToFaceContacts(SupportVertB, iNumVertsB, SupportVertA, iNumVertsA ,SeparatonNormal , true);
           SwapFlip();

        }
      }
    else if (iNumVertsA >= 3)
    {

        if (iNumVertsB == 1)
        {
            Vector3 An = Vector3::triNormal(SupportVertA[0],
                                              SupportVertA[1],
                                              SupportVertA[2]);
            scalar bd = An.Dot(SupportVertA[0]);
            PointToFaceContacts(SupportVertB[0], An, bd , SeparatonNormal);
        }
        else if (iNumVertsB == 2)
        {
            FaceToFaceContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB , SeparatonNormal , false);
        }
        else if (iNumVertsB >= 3)
        {

            IPlane PlaneA(SupportVertA[0],SupportVertA[1], SupportVertA[2]);
            IPlane PlaneB(SupportVertB[0],SupportVertB[1], SupportVertB[2]);

            Vector3 SupportMinmum2 = shape2Info.GetWorldSupportPointWithMargin(-PlaneA.GetNormal());
            Vector3 SupportMinmum1 = shape1Info.GetWorldSupportPointWithMargin(-PlaneB.GetNormal());

            scalar distance1 = IPlane::Distance(PlaneA,SupportMinmum2);
            scalar distance2 = IPlane::Distance(PlaneB,SupportMinmum1);

            const scalar kRelFaceTolerance = scalar(0.95); //95%
            const scalar kAbsTolerance = scalar(0.5) * LINEAR_SLOP;


            // Favor first hull face to avoid face flip-flops.
            if(  distance2 > kRelFaceTolerance * distance1 + kAbsTolerance )
            {
                // 2 = reference, 1 = incident.
                FaceToFaceContacts(SupportVertB, iNumVertsB, SupportVertA,iNumVertsA , SeparatonNormal , true );
                SwapFlip();
            }
            else
            {
                // 1 = reference, 2 = incident.
                FaceToFaceContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB , SeparatonNormal , false );
            }

         }
      }


  return (mNbContactPoints > 0);
}


void IContactGenerator::FaceToFaceContacts(const Vector3 *Poly, i32 iPolySize,
                                                  const Vector3 *Clipper, i32 iClipperSize, const Vector3 &_FaceNormal, bool flipNormal)
{

    IQuickClipping polyClipping( Poly, iPolySize, Clipper,iClipperSize);

    //const std::vector<Vector3> clipping_vertices = polyClipping.ComputeClippingVertices();
    const std::vector<Vector3> clipping_vertices = polyClipping.ComputeClippingVertices2();

    if (!clipping_vertices.empty())
    {
        Vector3 planeNormal = Vector3::triNormal(Poly[0], Poly[1], Poly[2]);
        scalar    clipper_d = Poly[0].Dot(planeNormal);

        if(!flipNormal)
        {

            for (u32 i = 0; i < clipping_vertices.size(); i++)
            {
                Vector3 P =  clipping_vertices[i];
                scalar   d = (P.Dot(planeNormal) - clipper_d);

                if ((d) < scalar(squarePersistentContactThreshold))
                {
                    //--------------------------------------------------------------------//
                    Vector3 A = P;
                    Vector3 B = P - planeNormal * d;
                    //scalar penetration = (A - B).dot( _FaceNormal );
                    scalar penetration = (A-B).Length();
                    IContactVertex info( _FaceNormal , penetration , A, B);
                    AddContact(info);
                    //--------------------------------------------------------------------//

                }
            }

        }
        else
        {
            for (i32 i=(clipping_vertices.size())-1; i>=i32(0); i--)
            {
                Vector3 P =  clipping_vertices[i];
                scalar   d = (P.Dot(planeNormal) - clipper_d);
                if ((d) < scalar(squarePersistentContactThreshold))
                {
                    //--------------------------------------------------------------------//
                    Vector3 A = P;
                    Vector3 B = P - planeNormal * d;
                    //scalar penetration = (A - B).dot( _FaceNormal );
                    scalar penetration = (A-B).Length();
                    IContactVertex info( _FaceNormal , penetration , A, B);
                    AddContact(info);
                    //--------------------------------------------------------------------//

                }
            }

        }

    }

}


void IContactGenerator::EdgeToEdgeContacts(const Vector3 &A0, const Vector3 &A1,
                                                  const Vector3 &B0, const Vector3 &B1, const Vector3 &_FaceNormal)
{

    Vector3 DA = A1 - A0;
    Vector3 DB = B1 - B0;
    Vector3 r  = A0 - B0;

    scalar a = DA.Dot(DA);
    scalar e = DB.Dot(DB);
    scalar f = DB.Dot(r);
    scalar c = DA.Dot(r);

    scalar b =  DA.Dot(DB);
    scalar denom = a * e - b * b;

    scalar TA = (b * f - c * e) / denom;
    scalar TB = (b * TA + f) / e;

    TA = IClamp(TA, scalar(0.0), scalar(1.0));
    TB = IClamp(TB, scalar(0.0), scalar(1.0));

    const Vector3 A = A0 + DA * TA;
    const Vector3 B = B0 + DB * TB;


    scalar penetration = (B - A).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
      Vector3 c_point = (A+B) * scalar(0.5);
      IContactVertex info(  _FaceNormal  , penetration , c_point , c_point);
      AddContact(info);
    }
}



void IContactGenerator::PointToEdgeContacts(const Vector3 &A, const Vector3 &B0, const Vector3 &B1, const Vector3 &_FaceNormal)
{
    Vector3 B0A = A - B0;
    Vector3 BD  = B1 - B0;

    scalar t = B0A.Dot(BD) / BD.Dot(BD);
    t = IClamp(t, 0.0f, 1.0f);

    Vector3 B = B0 + t * BD;

    scalar penetration = (A - B).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
      IContactVertex info(  _FaceNormal  , penetration , A, B);
      AddContact(info);
    }
}


void IContactGenerator::PointToFaceContacts(const Vector3 &A, const Vector3 &xAxis, scalar bd, const Vector3 &_FaceNormal)
{
    scalar penetration = (A.Dot(xAxis)) - bd;
    Vector3 B = A - penetration * xAxis;

    if(penetration < squarePersistentContactThreshold)
    {
      IContactVertex info( _FaceNormal , penetration, A, B);
      AddContact(info);
    }
}


void IContactGenerator::PointToPointContacts(const Vector3 &A, const Vector3 &B, const Vector3 &_FaceNormal)
{
    scalar penetration = (A - B).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
       IContactVertex info( _FaceNormal  , penetration , A , B);
       AddContact(info);
    }
}



Vector3 *IContactGenerator::CreateAxisPeturberationPoints(const IProxyShape *proxy_shape, const Vector3 &xAxis, i32 &_NbPoints ,  std::vector<Vector3>& results)
{

    const ICollisionShape* CollisionShape = proxy_shape->GetCollisionShape();



    //Matrix3 inverseBasis = proxy_shape->GetWorldTransform().GetBasis().GetTranspose();
    //inverseBasis.AproximateEpsilon(0.0004);

    Vector3 axis =  /*inverseBasis * */xAxis;
    Vector3 n0, n1;
    Vector3::BiUnitGrammSchmidt( axis , n0 , n1 );
    //Vector3::BiUnitOrthogonalVector(axis , n0 , n1);


    i32    MAX_ITERATION =  CollisionShape->mNbMaxPeturberationIteration + 1;
    scalar MIN_EPSILON = CollisionShape->mEpsilonPeturberation;//

    const scalar OFF_SET_COLLISION_CONTACT = scalar(0.02);

    Vector3 startPoint;
    Vector3 OldPoint;

    Vector3* result = nullptr;
    i32 &num_element = _NbPoints = 0;
    for( i32 i = 0; i <= MAX_ITERATION; i++ )
    {
        scalar ang = (scalar(2.0) * M_PI / scalar(MAX_ITERATION)) * scalar(i);

       // ang=360 * M_PI / 180.f;
        Vector3 auxAxis = (axis + n0 * ICos(ang) * MIN_EPSILON
                                + n1 * ISin(ang) * MIN_EPSILON);

       // auxAxis = axis;

//        glPushMatrix();
//        Vector3 pos = proxy_shape->GetWorldTransform().GetPosition();
//        glBegin(GL_LINES);
//        glVertex3fv(pos);
//        glVertex3fv(pos + auxAxis);
//        glEnd();
//        glPopMatrix();


        /***************************************************************/

        Matrix3 basis = proxy_shape->GetWorldTransform().GetBasis();
        Vector3 spVertex  = proxy_shape->GetWorldTransform().GetPosition() +
        basis * CollisionShape->GetLocalSupportPointWithMargin( basis.GetInverse() * auxAxis );


        /*-----------------------------------------------------*/
        if ((spVertex - OldPoint).LengthSquare() > OFF_SET_COLLISION_CONTACT || i == 0)
        {
            if ((spVertex - startPoint).LengthSquare() < OFF_SET_COLLISION_CONTACT && num_element > 0 ) break;


            /**
            if( i > 2)
            {
                Vector3 sp  = proxy_shape->GetWorldTransform() * CollisionShape->GetLocalSupportPointWithMargin( axis );
                if(IAbs(axis.Dot(proxy_shape->GetWorldTransform().GetInverse() * (spVertex - sp))) < 0.001) continue;
            }
            **/

            //                    if( num_element >= 3 )
            //                    {
            //                        Vector3 NFace = Vector3::triNormal(result[0],result[1],result[2]);
            //                        if(NFace.dot(result[0] - spVertex) > OFF_SET_COLLISION_CONTACT) continue;
            //                    }

            /***************************************/
            num_element++;
            Vector3* array = (Vector3*) IRealloc(result , (num_element+1) * sizeof(Vector3));
            if (array != nullptr)
            {
                result = array;
                result[num_element - 1] = spVertex;

                results.push_back(spVertex);

            }
            else
            {
                IFree(result);
            }

            /***************************************/

            OldPoint = spVertex;
            if (num_element == 1) startPoint = spVertex;
        }
    }

    //_NbPoints = results.size();
    return result;

}

void IContactGenerator::CreateAxisPeturberationPoints(const IProxyShape *proxy_shape, const Vector3 &xAxis, std::vector<Vector3> &results)
{
    const ICollisionShape* CollisionShape = proxy_shape->GetCollisionShape();

    //Matrix3 inverseBasis = proxy_shape->GetWorldTransform().GetBasis().GetTranspose();
    //inverseBasis.AproximateEpsilon(0.0004);

    Vector3 axis =  /*inverseBasis **/ xAxis;
    Vector3 n0, n1;
    Vector3::BiUnitGrammSchmidt( axis , n0 , n1 );
    //Vector3::BiUnitOrthogonalVector(axis , n0 , n1);


    i32    MAX_ITERATION =  CollisionShape->mNbMaxPeturberationIteration + 1;
    scalar MIN_EPSILON = CollisionShape->mEpsilonPeturberation;//

    const scalar OFF_SET_COLLISION_CONTACT = scalar(0.02);

    Vector3 startPoint;
    Vector3 OldPoint;

    for( i32 i = 0; i <= MAX_ITERATION; i++ )
    {
        scalar ang = (scalar(2.f) * M_PI / scalar(MAX_ITERATION)) * scalar(i);
        Vector3 auxAxis = (axis + n0 * ICos(ang) * MIN_EPSILON
                                + n1 * ISin(ang) * MIN_EPSILON);

        /***************************************************************/

        /**/
        Matrix3 basis = proxy_shape->GetWorldTransform().GetBasis();
        Vector3 spVertex  = proxy_shape->GetWorldTransform() *
        CollisionShape->GetLocalSupportPointWithMargin( basis.GetTranspose() * auxAxis );
        /**/


        //Vector3 spVertex  = proxy_shape->GetWorldTransform() * CollisionShape->GetLocalSupportPointWithMargin( auxAxis );


        /*-----------------------------------------------------*/
        if ((spVertex - OldPoint).LengthSquare() > OFF_SET_COLLISION_CONTACT || i == 0)
        {
            if ((spVertex - startPoint).LengthSquare() < OFF_SET_COLLISION_CONTACT && results.size() > 0 ) break;

            /***************************************/

            results.push_back(spVertex);

            /***************************************/

            OldPoint = spVertex;
            if (results.size() == 1) startPoint = spVertex;
        }
    }


    //-----------------------------------------//

    /**
    for( i32 i = 0; i <= MAX_ITERATION; i++ )
    {
        scalar ang = (scalar(2.f) * M_PI / scalar(MAX_ITERATION)) * scalar(i);
        Vector3 auxAxis = (axis + n0 * ICos(ang) * MIN_EPSILON * 2.f
                                + n1 * ISin(ang) * MIN_EPSILON * 2.f);


        Matrix3 basis = proxy_shape->GetWorldTransform().GetBasis();
        Vector3 spVertex  = proxy_shape->GetWorldTransform() *
        CollisionShape->GetLocalSupportPointWithMargin( basis.GetTranspose() * auxAxis );

        bool isEnablePush=true;
        for( auto it = results.begin(); it != results.end(); ++it)
        {
            if((spVertex-*it).Length() < 0.1)
            {
                isEnablePush = false;
                break;
            }
        }

        if(isEnablePush) results.push_back(spVertex);

    }
    **/
}



void IContactGenerator::AddContact(const IContactVertex &p_contact)
{
    mNbContactPoints++;
    IContactVertex* array = (IContactVertex*) IRealloc(mContactPoints , mNbContactPoints * sizeof(IContactVertex));
    if (array != nullptr)
    {
        mContactPoints = array;
        mContactPoints[mNbContactPoints - 1] = p_contact;
    }
    else
    {
        IFree(mContactPoints);
    }
}

void IContactGenerator::SwapFlip()
{
    for (i32 i = 0; i < mNbContactPoints; ++i)
    {
        ISwap( mContactPoints[i].point1 ,
               mContactPoints[i].point2 );
    }
}



}
