#ifndef ICLLIPING_H
#define ICLLIPING_H


#include <vector>
#include "../imaths.hpp"
#include "../IGeometry/Segments/IPlane.h"
#include "../IGeometry/Segments/ILineSegment3D.h"


namespace IEngine
{

namespace IAlgorithm
{

    using namespace IEngine;

    class IClliping
    {
       public:

         IClliping();


         enum
         {
             eMaxContacts = 32
         };


         static bool PlaneClip(const Vector3 * A, int Anum,
                                     Vector3 * B, int& Bnum,
                               const Vector3 & xPlaneNormal, scalar  PlaneD)
         {
             bool bBack[eMaxContacts];
             bool bBackVerts = false;
             bool bFrontVerts = false;

             for (int i = 0; i < Anum; i++)
             {
                 scalar  side = (A[i].Dot(xPlaneNormal)) - PlaneD;

                 bBack[i] = (side < 0.01f) ? true : false;
                 bBackVerts |= bBack[i];
                 bFrontVerts |= !bBack[i];
             }

             if (!bBackVerts)
             {
                 Bnum = 0;
                 return false;
             }

             if (!bFrontVerts)
             {
                 Bnum = Anum;
                 for (int i = 0; i < Anum; i++)
                     B[i] = A[i];
                 return true;
             }

             Bnum = 0;
             int i = Anum - 1;

             for (int ip1 = 0; ip1 < Anum; i = ip1, ip1++)
             {
                 if (bBack[i])
                 {
                     if (Bnum >= eMaxContacts)
                         return true;

                     B[Bnum++] = A[i];
                 }


                 if (bBack[ip1] ^ bBack[i])
                 {
                     if (Bnum >= eMaxContacts)
                         return true;


                          IPlane plane(xPlaneNormal,PlaneD);
                          B[Bnum++] = plane.VIntersectionLineToPlane( ILineSegment3D( A[ip1] , A[i] ) );

                         /**
                         Vector3  D = (A[ip1] - A[i]);
                         scalar  t = (PlaneD - (A[i].Dot(xPlaneNormal))) / (D.Dot(xPlaneNormal));
                         B[Bnum++] = A[i] + D * t;
                         /**/
                 }
             }

             return true;
         }



//         static void clipFace(const Vector3 * A, int  Anum,
//                                    Vector3 * B, int& Bnum,
//                              const Vector3 & planeNormalWS, scalar  planeEqWS)
//         {

//             int ve;
//             scalar  ds, de;
//             int numVerts = Anum;
//             if (numVerts < 2)
//                 return;

//             Vector3  firstVertex=A[Anum-1];
//             Vector3  endVertex = A[0];

//             ds = planeNormalWS.Dot(firstVertex)+planeEqWS;

//             for (ve = 0; ve < numVerts; ve++)
//             {
//                 endVertex=A[ve];

//                 de = planeNormalWS.Dot(endVertex)+planeEqWS;

//                 if (ds<0)
//                 {
//                     if (de<0)
//                     {
//                         // Start < 0, end < 0, so output endVertex
//                         B[Bnum++] = (endVertex);
//                     }
//                     else
//                     {
//                         // Start < 0, end >= 0, so output intersection
//                         B[Bnum++] = (firstVertex.Lerp(scalar(ds * 1.f/(ds - de)),endVertex));

//                     }
//                 }
//                 else
//                 {
//                     if (de<0)
//                     {
//                         // Start >= 0, end < 0 so output intersection and end
//                         B[Bnum++] = (firstVertex.Lerp(scalar(ds * 1.f/(ds - de)),endVertex));
//                         B[Bnum++] = (endVertex);
//                     }
//                 }
//                 firstVertex = endVertex;
//                 ds = de;
//             }
//         }

         static bool PolygonCliping(const Vector3 * axClipperVertices, int iClipperNumVertices,
                                    const Vector3 * axPolygonVertices, int iPolygonNumVertices,
                                          Vector3 * axClippedPolygon, int& iClippedPolygonNumVertices
                                           )
         {


             iClippedPolygonNumVertices = 0;

             if (iClipperNumVertices <= 2)
                 return false;

             Vector3  ClipperNormal = -Vector3::triNormal(axClipperVertices[0],
                                                          axClipperVertices[1],
                                                          axClipperVertices[2]);

             Vector3  Temp[32];
             int TempNum = iPolygonNumVertices;
             memcpy(Temp, axPolygonVertices, sizeof(*Temp) * TempNum);

             int i = iClipperNumVertices - 1;
             for (int ip1 = 0; ip1 < iClipperNumVertices; i = ip1, ip1++)
             {
                 Vector3  A0(axClipperVertices[i]);
                 Vector3  A1(axClipperVertices[ip1]);
                 Vector3  D(A1 - A0);
                 Vector3  N(ClipperNormal.Cross(D));
                 scalar   d = A0.Dot(N);


                 PlaneClip(Temp, TempNum, axClippedPolygon, iClippedPolygonNumVertices, N, d);

                 TempNum = iClippedPolygonNumVertices;
                 memcpy(Temp, axClippedPolygon, sizeof(*Temp) * TempNum);
             }

             return true;
         }

    };
}

}


#endif // ICLLIPING_H
