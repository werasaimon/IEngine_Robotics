#include "iclliping3d.h"


namespace IEngine
{

namespace IAlgorithm
{

bool Cliping_3d::InsidePolygon(Vector3 vIntersection, const Vector3 *Poly, int verticeCount)
{
    const scalar MATCH_FACTOR = 0.98f;
    scalar Angle = 0.0;
    Vector3  vA, vB;
    for (int i = 0; i < verticeCount; i++)
    {
        vA = Poly[i] - vIntersection;
        vB = Poly[(i + 1) % verticeCount] - vIntersection;

        Angle += vA.GetAngleBetween(vB);
    }

    if ((Angle >= (MATCH_FACTOR * (2.0 * M_PI))) ) return true;
    return false;
}

bool Cliping_3d::PointOnLine(const Vector3 &_edgeA, const Vector3 &_edgeB, const Vector3 &point)
{

    if( (point - _edgeA).LengthSquare() <= scalar(0.001 * 0.001) ) return true;
    if( (point - _edgeB).LengthSquare() <= scalar(0.001 * 0.001) ) return true;


    if ((point - _edgeA).Dot(point - _edgeB) > 0)  return false;
    return true;
}

void Cliping_3d::MoveTheIndex(int &index, bool _status)
{
    if (_status)
        index++;
    else
        index--;
}

void Cliping_3d::OutputIndex(int &_out0, int &_out1, int _max, int &_i)
{

    int index = (_i >= 0) ? _i : _i + _max;
    int indexation = (index >= 0 && index < _max) ? index : 0;

    if (index >= _max || index < 0)
    {
        _out0 = 0;
        _i = 0;
    }

    _out0 = (indexation >= 0) ? indexation : _max - 1;
    _out1 = (index < _max && index >= 0) ? indexation - 1 : _max - 1;

    if (_out1 < 0)
    {
        _i = 0;
        _out1 = _max - 1;
    }

}

int Cliping_3d::classifyPoly3D(const Vector3 &point, const Vector3 &polyNormal, const Vector3 &polyPoint)
{
    scalar length = (point - polyPoint).Dot( polyNormal );

    if (length >   0.f) return RIGHT;
    if (length <   0.f)  return LEFT;

    return BEHIND;
}

Vector3 Cliping_3d::ProjectPointIsPolygon(const Vector3 &point, const Vector3 &normal_poly, const Vector3 &point_poly)
{
    scalar tempf = (point_poly - point).Dot( normal_poly );
    Vector3  temp = point + normal_poly * tempf;

    return temp;
}


bool Cliping_3d::aimsAtLookPoly3D(const Vector3 &normaEdge, int aclass, const Vector3 &normalPoly, int crossType)
{

    Vector3  va = normaEdge;
    Vector3  vb = normalPoly;

    scalar v = vb.Dot( va );

    if (crossType != COLLINEAR)
    {
        if (v >= 0)
        {
            return (aclass != RIGHT);
        }
        else
        {
            return (aclass != LEFT);
        }
    }

    return false;
}

int Cliping_3d::intersectionLinePoly(const Vector3 vLine[], const Vector3 &vNormal, const Vector3 &_polyPoint, scalar &t, scalar &_denom)
{

    /**/
    Vector3  a = vLine[0];
    Vector3  b = vLine[1];

    Vector3  cc = _polyPoint;
    Vector3  n = vNormal;//.Direction();

    //Vector3  vLineDir = (a - b);

    scalar denom = Dot(n, b - a);
    _denom = denom;

    if (fabs(denom) == 0 )
    {
        int aclass = classifyPoly3D(a, n, _polyPoint);

        if ((aclass == LEFT) || (aclass == RIGHT))
        {
            return PARALLEL;
        }
        else
        {
            return COLLINEAR;
        }
    }

    scalar num = Dot(n, a - cc);

    t = -num / denom;
    return SKEW;
    /**/

}

int Cliping_3d::intersectionLinePolygon(Vector3 vLine[], const Vector3 &vNormal, const Vector3 &pointPoly, Vector3 &_point)
{
    Vector3  vLineDir = vLine[1] - vLine[0];
    scalar s = 0;
    //scalar epsilon = 10E-9;

    scalar denom = 0;
    int classe = intersectionLinePoly(vLine, vNormal, pointPoly, s, denom);


    //---------------------------//

    if (std::abs(denom) == 0)
    {
        _point = ( vLine[0] );
    }
    else
    {
        _point = vLine[0] + (vLineDir * s);
    }

    if ((classe == COLLINEAR) || (classe == PARALLEL))
    {
        return classe;
    }

    return SKEW_CROSS;;
}

bool Cliping_3d::intersectionLineToLine(Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2, Vector3 &out_intersection)
{
    Vector3  dir1 = end1 - start1;
    Vector3  dir2 = end2 - start2;

    //считаем уравнения прямых проходящих через отрезки
    scalar a1 = -dir1.y;
    scalar b1 = +dir1.x;
    scalar d1 = -(a1*start1.x + b1*start1.y);

    scalar a2 = -dir2.y;
    scalar b2 = +dir2.x;
    scalar d2 = -(a2*start2.x + b2*start2.y);

    //подставляем концы отрезков, для выяснения в каких полуплоскотях они
    scalar seg1_line2_start = a2*start1.x + b2*start1.y + d2;
    scalar seg1_line2_end = a2*end1.x + b2*end1.y + d2;

    scalar seg2_line1_start = a1*start2.x + b1*start2.y + d1;
    scalar seg2_line1_end = a1*end2.x + b1*end2.y + d1;

    //если концы одного отрезка имеют один знак, значит он в одной полуплоскости и пересечения нет.
    if (seg1_line2_start * seg1_line2_end >= 0 || seg2_line1_start * seg2_line1_end >= 0)
        return false;

    scalar u = seg1_line2_start / (seg1_line2_start - seg1_line2_end);
    out_intersection =  start1 + u*dir1;


    return true;
}

void Cliping_3d::GetViarableEdgeLineEnd(int &end_index, const Vector3 *element, int size_element, Vector3 &_org, Vector3 &_dest)
{
    int index0 = 0, index1 = 0;
    OutputIndex(index0, index1, size_element, end_index);

    Vector3  org = element[index0];
    Vector3  dest = element[index1];

    _org = org;
    _dest = dest;

}

void Cliping_3d::ComputeEdgeIsPoly(const Vector3 _line[], const Vector3 *axVertices, int num_vertices, Vector3 *Poly, int &poly_num)
{



    Vector3  Normal0 = Vector3::triNormal(axVertices[0] ,
            axVertices[1] ,
            axVertices[2]);


    CPlane planeP(Normal0 , axVertices[0]);

    bool crossIsStatus = false;
    for (int i = 0; i < num_vertices; ++i)
    {

        Vector3  vA = axVertices[i];
        Vector3  vB = axVertices[(i + 1) % num_vertices];

        Vector3  EdgeLineP[] = { vA , vB };
        Vector3  EdgeLineQ[] = { _line[0] , _line[1] };



        ISwap(EdgeLineP[0], EdgeLineP[1]);
        ISwap(EdgeLineQ[0], EdgeLineQ[1]);

        //            if ( (EdgeLineQ[0] - axVertices[0]) * normalAxis < 0)
        //            {
        //                Swap(EdgeLineQ[0], EdgeLineQ[1]);
        //            }

        //Vector3  _edgeP = (EdgeLineP[1] - EdgeLineP[0]);
        //Vector3  _edgeQ = (EdgeLineQ[1] - EdgeLineQ[0]);


        Vector3  EdgeNormalP = planeP.ProjectPoint3D(EdgeLineP[1] - EdgeLineP[0]).Cross( -Normal0 );
        Vector3  EdgeNormalQ = planeP.ProjectPoint3D(EdgeLineQ[1] - EdgeLineQ[0]).Cross( -Normal0 );
        Vector3  pointP = EdgeLineP[1];
        Vector3  pointQ = EdgeLineQ[1];

        int pclass = classifyPoly3D(EdgeLineP[1], EdgeNormalQ, pointQ);
        int qclass = classifyPoly3D(EdgeLineQ[1], EdgeNormalP, pointP);

        Vector3  iPnt;
        int crossType = CorossTestIntersectionLinePoly( EdgeLineP, EdgeLineQ , EdgeNormalP , pointP, iPnt , planeP);

        //bool pAIMSq = aimsAtLookPoly3D((EdgeLineP[1] - EdgeLineP[0]) , pclass, EdgeNormalQ, crossType);
        bool qAIMSp = aimsAtLookPoly3D((EdgeLineQ[1] - EdgeLineQ[0]) , qclass, EdgeNormalP, crossType);


        if ( crossType == SKEW_CROSS )
        {
            crossIsStatus = true;
            Poly[poly_num++] = iPnt;
            //Poly[poly_num++] = Pnt;

            if(!qAIMSp)
            {
                if (qclass == RIGHT)
                {
                    if(InsidePolygon( EdgeLineQ[0] , axVertices , num_vertices))
                        Poly[poly_num++] = EdgeLineQ[0];
                }
                else
                {
                    if(InsidePolygon( EdgeLineQ[1] , axVertices , num_vertices))
                        Poly[poly_num++] = EdgeLineQ[1];
                }
            }

        }
        else if ((crossType == COLLINEAR) &&
                 (qclass    != BEHIND))
        {
            crossIsStatus = true;
            Poly[poly_num++] = EdgeLineQ[0];
            Poly[poly_num++] = EdgeLineQ[1];

            break;
        }

    }


    /**

                   if( !crossIsStatus && poly_num == 1 )
                   {
                       bool isActive0 = InsidePolygon( _line[0] , axVertices , num_vertices);
                       bool isActive1 = InsidePolygon( _line[1] , axVertices , num_vertices);

                       if(isActive0) Poly[poly_num++] = _line[0];
                       else
                       if(isActive1) Poly[poly_num++] = _line[1];

                   }

        /**/

    // это единственный выход с данной ситуации
    if( !crossIsStatus || poly_num == 0 )
    {

        Poly[poly_num++] = _line[0];
        Poly[poly_num++] = _line[1];

        //printf("ggg :   %i  :-" , poly_num);
    }
    /**/


}

int Cliping_3d::CorossTestIntersectionLinePoly(Vector3 line_p[], Vector3 line_q[], const Vector3 &normal, const Vector3 &point, Vector3 &_p, CPlane plane)
{

    scalar denom = normal.Dot( line_q[1] - line_q[0]);
    if (denom == 0 )
    {
        int aclass = classifyPoly3D(line_q[0], normal, line_p[0]);

        if ((aclass == LEFT) || (aclass == RIGHT))
        {

            return PARALLEL;
        }
        else
        {
            return COLLINEAR;
        }
    }


    Vector3  crossPoint;
    intersectionLinePolygon( line_q , normal , point , crossPoint);
    _p = (crossPoint);

    // 			Vector3  projectLineQnaP[] = { plane.ProjectPoint3D( line_q[0] ) , plane.ProjectPoint3D( line_q[1] ) };
    // 			Vector3  projectLinePnaQ[] = { plane.ProjectPoint3D( line_p[0] ) , plane.ProjectPoint3D( line_p[1] ) };

    bool status_lineQ = PointOnLine(line_q[1], line_q[0], crossPoint);
    bool status_lineP = PointOnLine(line_p[1], line_p[0], crossPoint);


    if ( status_lineQ && status_lineP  )
    {

        return SKEW_CROSS;
    }
    else
    {
        return SKEW_NO_CROSS;
    }

}

bool Cliping_3d::PolygonClliping3D(const Vector3 *axVerticesA, int num_verticesA, const Vector3 *axVerticesB, int num_verticesB, Vector3 *Poly, int &poly_num)
{
    //Vector3  Normal0 = _normal;
    Vector3  Normal0 = Vector3::triNormal(axVerticesA[0],
            axVerticesA[1],
            axVerticesA[2]);

    int end_index_p = 0;
    int end_index_q = 0;

    poly_num = 0;

    if(num_verticesA >= 3 && num_verticesB == 2)
    {
        Vector3  line[] = { axVerticesB[0] ,
                            axVerticesB[1] };


        //if( _normal * Normal0 < 0 ) Swap( line[0] , line[1] );

        ComputeEdgeIsPoly(line , axVerticesA , num_verticesA , Poly , poly_num );


    }
    else  if(num_verticesA >= 3 && num_verticesB >= 3)
    {

        end_index_p = 0;
        end_index_q = 0;

        Vector3  end_point,
                start_point;

        int phase = 1;
        int maxItns = 2 * (num_verticesA + num_verticesB);
        int inflag = UNKNOWN;

        //Vector3  Normal0 = Vector3 ::Normal(axVerticesA[0], axVerticesA[1],axVerticesA[2]);

        Vector3  Normal1 = Vector3::triNormal(axVerticesB[0],
                                              axVerticesB[1],
                                              axVerticesB[2]);


        CPlane planeP(Normal0, axVerticesA[0]);
        CPlane planeQ(Normal1, axVerticesB[0]);

        Vector3  EdgeLineP[] = { axVerticesA[0], axVerticesA[1] };
        Vector3  EdgeLineQ[] = { axVerticesB[0], axVerticesB[1] };

        bool statusInvertNumerator = (Normal1.Dot(Normal0) >= 0);

        int num_testCrections = 0;

        for (int i = 1; ((i <= maxItns) || (phase == 2)); i++)
        {

            if (phase == 2)
                num_testCrections++;
            if (num_testCrections > (num_verticesA + num_verticesB))
            {
                break;
            }

            GetViarableEdgeLineEnd(end_index_p, axVerticesA, num_verticesA, EdgeLineP[0], EdgeLineP[1]);
            GetViarableEdgeLineEnd(end_index_q, axVerticesB, num_verticesB, EdgeLineQ[0], EdgeLineQ[1]);

            ISwap(EdgeLineP[0], EdgeLineP[1]);
            ISwap(EdgeLineQ[0], EdgeLineQ[1]);

            if (!statusInvertNumerator)
            {
                ISwap(EdgeLineQ[0], EdgeLineQ[1]);
            }

            //Vector3  _edgeP = (EdgeLineP[1] - EdgeLineP[0]);
            //Vector3  _edgeQ = (EdgeLineQ[1] - EdgeLineQ[0]);


            Vector3  EdgeNormalP = planeP.ProjectPoint3D(EdgeLineP[1] - EdgeLineP[0]).Cross( -Normal0 );
            Vector3  EdgeNormalQ = planeP.ProjectPoint3D(EdgeLineQ[1] - EdgeLineQ[0]).Cross( -Normal0 );

            Vector3  pointP = EdgeLineP[1];
            Vector3  pointQ = EdgeLineQ[1];

            int pclass = classifyPoly3D(EdgeLineP[1], EdgeNormalQ, pointQ);
            int qclass = classifyPoly3D(EdgeLineQ[1], EdgeNormalP, pointP);

            Vector3  iPnt;
            int crossType = CorossTestIntersectionLinePoly(EdgeLineP, EdgeLineQ,
                                                           EdgeNormalP, pointP, iPnt, planeP);


            /************************************/
            if ( crossType == SKEW_CROSS )
            {
                if (phase == 1 && poly_num == 0)
                {
                    phase = 2;
                    Poly[poly_num++] = (iPnt);
                    start_point = iPnt;
                }
                else if (iPnt != start_point || true)
                {
                    if (iPnt != start_point)
                    {
                        Poly[poly_num++] = (iPnt);
                    }
                    else if (poly_num > 1)
                    {
                        break;
                    }
                }

                if (pclass == RIGHT)
                {
                    inflag = P_IS_INSIDE;
                }
                else if (qclass == RIGHT)
                {
                    inflag = Q_IS_INSIDE;
                }
                else
                {
                    inflag = UNKNOWN;
                }

            }
            else if ((crossType == COLLINEAR) &&
                     (pclass != BEHIND)       &&
                     (qclass != BEHIND))
            {

                inflag = UNKNOWN;

                Poly[poly_num++] = EdgeLineP[0];
                Poly[poly_num++] = EdgeLineP[1];

                continue;
            }
            /********************************/

            bool pAIMSq = aimsAtLookPoly3D((EdgeLineP[1] - EdgeLineP[0]) , pclass, EdgeNormalQ, crossType);
            bool qAIMSp = aimsAtLookPoly3D((EdgeLineQ[1] - EdgeLineQ[0]) , qclass, EdgeNormalP, crossType);


            //                if(pAIMSq) Swap( EdgeLineP[1] , EdgeLineP[0]);
            //                if(qAIMSp) Swap( EdgeLineQ[1] , EdgeLineQ[0]);

            /**************************/
            if (pAIMSq && qAIMSp)
            {
                if ((inflag == Q_IS_INSIDE) ||
                        ((inflag == UNKNOWN) &&
                         (pclass == LEFT)))
                {
                    //advance(P, *R, FALSE);
                    MoveTheIndex(end_index_p, true);
                }
                else
                {
                    //advance (Q, *R, FALSE);
                    MoveTheIndex(end_index_q, statusInvertNumerator);
                }
            }
            else if (pAIMSq)
            {
                //advance (P, *R, inflag == P_IS_INSIDE);
                MoveTheIndex(end_index_p, true);

                if (inflag == P_IS_INSIDE && num_verticesB >= 3 )
                {


                    Vector3  point;
                    Vector3  _line[] = { EdgeLineP[1], EdgeLineP[1] + Normal0 };
                    intersectionLinePolygon(_line, Normal1, EdgeLineQ[1], point);

                    Poly[poly_num++] = (point);


                }
            }
            else if (qAIMSp)
            {
                // advance (Q, *R, inflag == Q_IS_INSIDE);
                MoveTheIndex(end_index_q, statusInvertNumerator);

                if (inflag == Q_IS_INSIDE)
                {
                    Poly[poly_num++] = EdgeLineQ[1];
                }
            }
            else
            {
                if ((inflag == Q_IS_INSIDE) ||
                        ((inflag == UNKNOWN) &&
                         (pclass == LEFT)))
                {

                    //advance ( P , *R , FALSE );
                    MoveTheIndex(end_index_p, true);
                }
                else
                {

                    //advance (Q, *R, FALSE);
                    MoveTheIndex(end_index_q, statusInvertNumerator);
                }
            }
            /************************/

        }

    }



    //**********************************************************************************//

    if (poly_num == 0)
    {

        end_index_p = (end_index_p == num_verticesA || end_index_p < num_verticesA-1) ? 0: end_index_p;
        end_index_q = (end_index_q == num_verticesB || end_index_q < num_verticesB-1) ? 0: end_index_q;

        //if (num_verticesA >= 3 && num_verticesB >= 3)
        {
            Vector3  Ppoint = axVerticesA[end_index_p];
            Vector3  Qpoint = axVerticesB[end_index_q];





            Vector3  Normal1 = Vector3::triNormal(axVerticesB[0],
                    axVerticesB[1],
                    axVerticesB[2]);



            if (InsidePolygon(Ppoint, axVerticesB, num_verticesB ))
            {
                for (int i = 0; i < num_verticesA; i++)
                {
                    Vector3  point;
                    Vector3  _Line[] = { axVerticesA[i], axVerticesA[i] + Normal0 };

                    if (intersectionLinePolygon(_Line, Normal1, Qpoint, point)||true)
                    {
                        Poly[poly_num++] = (point);
                    }

                }
            }
            else //if (InsidePolygon(Qpoint, axVerticesA, num_verticesA ))
            {
                for (int i = 0; i < num_verticesB; i++)
                {
                    Poly[poly_num++] = axVerticesB[i];
                }
            }

        }

    }

    return (poly_num > 0);

}

}

}
