#ifndef IQUICKCLIPPING_H
#define IQUICKCLIPPING_H

#include <vector>
#include "../imaths.hpp"
#include "../IGeometry/Segments/IPlane.h"
#include "../IGeometry/Segments/ILineSegment3D.h"


namespace IEngine
{

namespace IAlgorithm
{
  using namespace std;

class IQuickClipping
{
    enum { UNKNOWN = 10, P_IS_INSIDE, Q_IS_INSIDE };
    enum { LEFT = 30, RIGHT, BEHIND };
    enum { COLLINEAR = 40, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS };


//    static bool intersectionLineToLine(Vector3  start1, Vector3  end1, Vector3  start2, Vector3  end2, Vector3  &out_intersection)
//    {
//        Vector3  dir1 = end1 - start1;
//        Vector3  dir2 = end2 - start2;

//        //считаем уравнения прямых проходящих через отрезки
//        scalar a1 = -dir1.y;
//        scalar b1 = +dir1.x;
//        scalar d1 = -(a1*start1.x + b1*start1.y);

//        scalar a2 = -dir2.y;
//        scalar b2 = +dir2.x;
//        scalar d2 = -(a2*start2.x + b2*start2.y);

//        //подставляем концы отрезков, для выяснения в каких полуплоскотях они
//        scalar seg1_line2_start = a2*start1.x + b2*start1.y + d2;
//        scalar seg1_line2_end = a2*end1.x + b2*end1.y + d2;

//        scalar seg2_line1_start = a1*start2.x + b1*start2.y + d1;
//        scalar seg2_line1_end = a1*end2.x + b1*end2.y + d1;

//        //если концы одного отрезка имеют один знак, значит он в одной полуплоскости и пересечения нет.
//        if (seg1_line2_start * seg1_line2_end >= 0 || seg2_line1_start * seg2_line1_end >= 0)
//            return false;

//        scalar u = seg1_line2_start / (seg1_line2_start - seg1_line2_end);
//        out_intersection =  start1 + u*dir1;


//        return true;
//    }


    struct IClipEdge
    {

        IClipEdge( const Vector3& _a = Vector3(0,0,0) ,
                   const Vector3& _b = Vector3(0,0,0))
            : mA(_a) , mB(_b)
        {

        }

        // isInsideLineSegment(): determine if a point is inside a segment
        //    Input:  a point P, and a collinear segment S
        //    Return: 1 = P is inside S
        //            0 = P is  not inside S
        bool IsInsideLineSegment( const Vector3 &point )
        {

//            if( (point - mA).LengthSquare() <= scalar(0.001 * 0.001) ) return true;
//            if( (point - mB).LengthSquare() <= scalar(0.001 * 0.001) ) return true;

            if ((point - mA).Dot(point - mB) > 0)  return false;
            return true;
        }

//        static bool intersection(const IClipEdge& a, const IClipEdge& b)
//        {
//            Vector3 da = a.mB - a.mA;
//            Vector3 db = b.mB - b.mA;
//            Vector3 dc = b.mA - a.mB;

//            if (dc.Dot(da.Cross(dc)) != 0.f) // lines are not coplanar
//                return false;

//            float s = (dc.Cross(db)).Dot( (da.Cross(db)) / (da.Cross(db).LengthSquare()));

//            if (s >= 0.f && s <= 1.f)
//            {
//               return true;
//            }
//        }


        Vector3 GetCenter() const
        {
            return (mA + mB) * 0.5f;
        }


        Vector3 mA;
        Vector3 mB;
    };


    class IClipPlane : public IPlane
    {

      public:

        //-------------------- Constructor --------------------//

        IClipPlane( Vector3 _n , Vector3 _origin)
         : IPlane( _n.GetUnit() , _origin )
        {

        }

        IClipPlane( const Vector3& a ,
                    const Vector3& b ,
                    const Vector3& c)
          :IPlane(a,b,c)
        {

        }


        //-------------------- Method -------------------------//

        int SideClassifyPointToEdge( const Vector3& _inVtx ) const
        {

            scalar length = IPlane::InvTest(_inVtx);

            if (length >= 0.0f) return RIGHT;
            if (length <= 0.0f) return LEFT;

            return BEHIND;
        }



        bool IsAtLookToPoly(const Vector3& DirectionLook ,  int aclass ) const
        {
            Vector3 va = DirectionLook;
            Vector3 vb = mNormal;

            scalar  v = vb.Dot(va);

            if (v >= 0.0f)
            {
                return (aclass != RIGHT);
            }
            else
            {
                return (aclass != LEFT);
            }
        }


        //===================================================================

        // vIntersectionLineToPlane(): find the 3D intersection of a segment and a plane
        //    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
        //    Output: *I0 = the intersect point (when it exists)
        //    Return: 0 = disjoint (no intersection)
        Vector3 VIntersectionLineToPlane( const IClipEdge& edge ) const
        {
           //return IPlane::VIntersectionLineToPlane(ILineSegment3D(_edge.mA,_edge.mB) , parallel_test );

            Vector3 N =  mNormal;
            Vector3 P =  edge.mA;
            Vector3 W = (edge.mB - edge.mA);

            scalar  d =  InvTest(P);
            scalar  e =  N.Dot(W);

            if( IMath::IAbs(e) < MACHINE_EPSILON  ) return edge.mB + W.Normalized() * 0.02f;

            scalar param = d/e;
            return P + W * param;
        }


    };



    class IClipPolygon
    {
        const Vector3 *mVertices;
        const int       mNbVertices;

    public:

        IClipPolygon(const Vector3* _vertices , const int _count )
            : mVertices(_vertices) ,
              mNbVertices(_count)
        {

        }

        int GetCount() const
        {
            return mNbVertices;
        }

        const Vector3* GetVertices() const
        {
            return mVertices;
        }

        //        IClipEdge EdgeAtt( int &_index ) const
        //        {
        //            int index_a = (_index + 1);
        //            int index_b = _index;

        //            if(_index == (mNbVertices-1))
        //            {
        //                index_a = 0;
        //                index_b = (mNbVertices-1);
        //            }


        //            return IClipEdge(mVertices[index_a],
        //                             mVertices[index_b]);
        //        }



        IClipEdge EdgeAt( int &_i ) const
        {
                int index_a;
                int index_b;

                int index = (_i >= 0) ? _i : _i + mNbVertices;
                int indexation = (index >= 0 && index < mNbVertices) ? index : 0;

                if (index >= mNbVertices || index < 0)
                {
                    index_a = 0;
                    _i = 0;
                }

                index_b = (indexation >= 0) ? indexation : mNbVertices - 1;
                index_a = (index < mNbVertices && index >= 0) ? indexation - 1 : mNbVertices - 1;

                if (index_a < 0)
                {
                    _i = 0;
                    index_a = mNbVertices - 1;
                }

               // std::cout << "index " << _i << "     index_a: " << index_a << "   index_b: " << index_b << std::endl;

                return IClipEdge(mVertices[index_a],
                                 mVertices[index_b]);
        }

    };


private:


    IClipPolygon mPolygon;
    IClipPolygon mClipPolygon;

    /// Private copy-constructor
    IQuickClipping(const IQuickClipping& clipping);

    /// Private assignment operator
    IQuickClipping& operator=(const IQuickClipping& clipping);

public:

    IQuickClipping(const Vector3*  PolygonVertices , int  CountPolygonVertices ,
                   const Vector3*  ClipVertices    , int  CountClipVertices)
        : mPolygon(PolygonVertices,CountPolygonVertices) ,
          mClipPolygon(ClipVertices , CountClipVertices)
    {
    }

    ~IQuickClipping()
    {
    }


    //******************  Compute clipping **************************//

    std::vector<Vector3> ComputeClippingVertices() const;
    std::vector<Vector3> ComputeClippingVertices2() const;


private:


    //****************   Help Function ****************************//
    static int   NextMoveIndexToEdges( bool bPlaneAIMSLookToLineA ,
                                       bool aPlaneAIMSLookToLineB ,
                                       bool isLeftClassification  ,
                                       bool exceptions ,
                                       bool isLookFaceToFace ,
                                       int& moveIndexA ,
                                       int& moveIndexB );

    static void   MoveIndex(int& index , bool isLookFaceToFace );


    static bool   InsidePolygonSAT(const Vector3& vIntersection, const Vector3* PolyVertices , int NbCount, const IClipPlane &plane);


};

}

}

#endif // IQUICKCLIPPING_H
