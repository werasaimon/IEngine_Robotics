#ifndef ISCANCOLSINGPATH_H
#define ISCANCOLSINGPATH_H


#include "../imaths.hpp"
#include "Engine/IGeometry/Segments/IPlane.h"
#include <vector>
#include <algorithm>


#define MIN_EPS 0.001
#define MINMUM_EPS 0.0001

using namespace  IEngine;

class CallbeckSupprtModel
{
    public:

        CallbeckSupprtModel()
        {

        }

        /// Destructor
        virtual ~CallbeckSupprtModel()
        {

        }

        /// This method will be called for each ProxyShape that is hit by the
        /// ray. You cannot make any assumptions about the order of the
        /// calls. You should use the return value to control the continuation
        /// of the ray. The returned value is the next maxFraction value to use.
        /// If you return a fraction of 0.0, it means that the raycast should
        /// terminate. If you return a fraction of 1.0, it indicates that the
        /// ray is not clipped and the ray cast should continue as if no hit
        /// occurred. If you return the fraction in the parameter (hitFraction
        /// value in the IRaycastInfo object), the current ray will be clipped
        /// to this fraction in the next queries. If you return -1.0, it will
        /// ignore this ProxyShape and continue the ray cast.
        /**
         * @param IRaycastInfo Information about the raycast hit
         * @return Value that controls the continuation of the ray after a hit
         */
        virtual Vector3 SupportPoint(const Vector3& AxisDirection) const =0;

        virtual const Matrix4 GetTransform() const =0;
};

class IScanColsingPath
{
    public:
        IScanColsingPath();



        static void ScanClosingPath(const CallbeckSupprtModel& originalPoints,
                                              std::vector<Vector3>& hull,
                                              const IPlane& plane,
                                              const Vector3& _a,
                                              const Vector3& _b)
        {

            Vector3 normalAxis = plane.GetNormal();

            //------------------------------------------------------//

            //Vector3 OrthogonalVector = normalAxis.GetOneUnitOrthogonalVector();
            Vector3 start_p0 = plane.ClosestPoint(_b);
            Vector3 start_p1 = plane.ClosestPoint(_a);


            Vector3 line = plane.ClosestPoint(start_p0 - start_p1);
            Vector3 Axis = IMath::Cross(line,normalAxis);
            //------------------------------------------------------//


            const Vector3 start = start_p0;
            Vector3 out;
            Vector3 prev;


            enum test { UNKNOW , A , B };
            test type_stop = test::UNKNOW;


            hull.push_back( _b );

            int i =0;
            while((i++) < 100)
            {

                ClosingEdgePoint(originalPoints,hull,normalAxis,start_p0,start_p1,plane,out);

               // Vector3 a = plane.ClosestPoint(_b);
                Vector3 b = plane.ClosestPoint(out);
                //Vector3 axis = IMath::Cross(a - b, normalAxis);

                if((b - plane.ClosestPoint(_a)).LengthSquare() > MIN_EPS &&
                   (b - plane.ClosestPoint(_b)).LengthSquare() > MIN_EPS &&
                   (b - prev).LengthSquare() > MIN_EPS)
                {
                    hull.push_back(b - Axis.Normalized() * 15);
                }

                prev = b;

                if(type_stop == test::UNKNOW)
                {
                   if((plane.ClosestPoint(_b) - b).LengthSquare() < MIN_EPS ) type_stop = test::B;
                   if((plane.ClosestPoint(_a) - b).LengthSquare() < MIN_EPS ) type_stop = test::A;
                }
                else
                {
                    if(type_stop == test::A)
                    {
                        if((plane.ClosestPoint(_b) - b).LengthSquare() < MIN_EPS ) break;
                    }
                    else if(type_stop == test::B)
                    {
                        if((plane.ClosestPoint(_a) - b).LengthSquare() < MIN_EPS ) break;
                    }
                }

                start_p0 = out;
                start_p1 = plane.ClosestPoint(_a);

            }


            hull.push_back( _a );

        }



//        static void ScanConvexHull2D(const CallbeckSupprtModel& originalPoints, std::vector<Vector3>& hull, const IPlane& plane )
//        {

//            Vector3 normalAxis = plane.GetNormal();

//            //------------------------------------------------------//

//            Vector3 OrthogonalVector = normalAxis.GetOneUnitOrthogonalVector();
//            Vector3 start_p0 = originalPoints.SupportPoint(-OrthogonalVector);
//            Vector3 start_p1 = originalPoints.SupportPoint( OrthogonalVector);

//            Vector3 line = plane.ClosestPoint(start_p0 - start_p1);
//            Vector3 Axis = IMath::Cross(line,normalAxis);
//            if( (line).Dot(Axis) > 0 )
//            {
//               // IMath::ISwap(start_p0,start_p1);
//            }

//            //------------------------------------------------------//

//            const Vector3 start = start_p0;
//            Vector3 out;
//            Vector3 prev = start_p1;

//            while(start != out)
//            {
//                ClosingEdgePoint(originalPoints,hull,normalAxis,start_p0,start_p1,plane,out);

//                Vector3 a = plane.ClosestPoint(start_p0);
//                Vector3 b = plane.ClosestPoint(out);
//                Vector3 axis = IMath::Cross(a - b, normalAxis);

//                if((a-b).Length() > 0.01f /*&& (b-start).Length() > 0.01f*/)
//                {
//                    hull.push_back( /*originalPoints.GetTransform().GetTranslation() +*/ b );
//                }
//                else if(!hull.empty())
//                {
//                    start_p0 = prev;
//                }

//                start_p0 = out;
//                start_p1 = originalPoints.SupportPoint(axis);
//                prev = out;
//            }

//             //------------------------------------------------------//

//        }



        static void ClosingEdgePoint(const CallbeckSupprtModel& originalPoints,
                                     std::vector<Vector3>& hull,
                                     const Vector3& normalAxis,
                                     const Vector3& p1,
                                     const Vector3& p2,
                                     const IPlane& plane,
                                     Vector3& out,int i = 0)
        {

            Vector3 line = plane.ClosestPoint(p2 - p1);
            Vector3 Axis = IMath::Cross(line,normalAxis);
            Vector3 p = originalPoints.SupportPoint(Axis);
            scalar distance = ((p-p1).Dot(Axis)) ;

            if(p != p1) out = (p);

            if( (distance) > MINMUM_EPS)
            {
               ClosingEdgePoint(originalPoints,hull,normalAxis,p1,p,plane,out,i+1);
            }
            else if(p == p1)
            {
               out = (p2);
            }
        }
};

#endif // ISCANCOLSINGPATH_H
