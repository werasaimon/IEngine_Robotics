#ifndef ICLLIPING3D_H
#define ICLLIPING3D_H


#include <vector>
#include "../imaths.hpp"
#include "../IGeometry/Segments/IPlane.h"
#include "../IGeometry/Segments/ILineSegment3D.h"


namespace IEngine
{

namespace IAlgorithm
{


enum
{
    CLOCKWISE = 50
};

enum
{
    UNKNOWN = 100, P_IS_INSIDE, Q_IS_INSIDE
};
enum
{
    LEFT = 300, RIGHT, BEYOND, BEHIND, BETWEEN, ORIGIN, DESTINATION
};
enum
{
    COLLINEAR = 400, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS
};




class CPlane
{
public:

    CPlane(void)
    {

    }


    ~CPlane(void)
    {

    }

     CPlane(Vector3  Normal, Vector3  Origin)
     {
         origin = Origin;
             normal = Normal;
             normal.Normalize();
             //wsVector3::ComplimentaryPredipikularZY(normal, RightVec, UpVec);

              Vector3::BiUnitOrthogonalVector( normal, RightVec, UpVec );

     }

     Vector3  projectionIsPlane2D(const Vector3  &point) const
     {
         Vector3  RES;
         RES.x = RightVec.Dot(point - origin);
         RES.y = UpVec.Dot(point - origin);
         return RES;
     }

     Vector3  ProjectPoint3D(const Vector3  &point) const
     {
           Vector3  RES;
           RES = origin + UpVec * point.y + RightVec * point.x;
           return RES;
     }


     Vector3  VecTo3d(const Vector2 &point) const
     {
         Vector3  RES;
         RES = origin + UpVec * point.y + RightVec * point.x;
         return RES;
     }


     Vector3 VecTo3d(const Vector3  &point) const
     {
         Vector3  RES;
         RES = origin + UpVec * point.y + RightVec * point.x;
         return RES;
     }


    Vector3  getOrgin() const
    {
        return origin;
    }

    Vector3  getNormal() const
    {
        return normal;
    }

    Vector3  operator&(const Vector3 & _v) const
    {
        return ProjectPoint3D(_v);
    }

private:

    Vector3  UpVec;
    Vector3  RightVec;
    Vector3  normal;

    Vector3  origin;
};



class Cliping_3d
{

//private:
public:




    static bool InsidePolygon( Vector3  vIntersection , const Vector3  *Poly, int verticeCount);




    static bool PointOnLine(const Vector3  &_edgeA , const Vector3  &_edgeB , const Vector3  &point);




    static void MoveTheIndex(int& index, bool _status);




    static void OutputIndex(int &_out0, int &_out1, int _max, int &_i);




    static int classifyPoly3D(const Vector3 &  point,
                              const Vector3 &  polyNormal,
                              const Vector3 &  polyPoint);





    static Vector3  ProjectPointIsPolygon(const Vector3 & point,
                                           const Vector3 & normal_poly,
                                           const Vector3 & point_poly);





    static bool aimsAtLookPoly3D(const Vector3 & normaEdge,  int aclass,
                                 const Vector3 & normalPoly, int crossType);






    static int intersectionLinePoly(const Vector3  vLine[],
                                    const Vector3  &vNormal,
                                    const Vector3  &_polyPoint,
                                    scalar &t,
                                    scalar& _denom);





    static int intersectionLinePolygon(Vector3  vLine[], const Vector3  &vNormal, const Vector3  &pointPoly, Vector3  &_point);



    static bool intersectionLineToLine(Vector3  start1, Vector3  end1, Vector3  start2, Vector3  end2, Vector3  &out_intersection);



    static void GetViarableEdgeLineEnd(int &end_index, const Vector3 * element, int size_element,
                                       Vector3  &_org, Vector3  &_dest);


    static void ComputeEdgeIsPoly( const Vector3  _line[] ,
                                   const Vector3 * axVertices, int num_vertices ,
                                         Vector3  *Poly, int &poly_num );



    static int CorossTestIntersectionLinePoly(Vector3  line_p[], Vector3  line_q[],
                                              const Vector3 & normal, const Vector3 & point, Vector3 & _p, CPlane plane);

public:

    static bool PolygonClliping3D(const Vector3 * axVerticesA, int num_verticesA,
                                  const Vector3 * axVerticesB, int num_verticesB,
                                        Vector3  *Poly, int& poly_num );

};

}

}

#endif // ICLLIPING3D_H
