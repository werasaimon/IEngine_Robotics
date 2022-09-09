#ifndef IGRAHAMSCAN2DCONVEXHULL_H
#define IGRAHAMSCAN2DCONVEXHULL_H

#include "../imaths.hpp"
#include "Engine/IGeometry/Segments/IPlane.h"
#include <vector>
#include <algorithm>

using namespace  IEngine;

struct GrahamVector3 : public Vector3
{
	GrahamVector3(const Vector3& org, int orgIndex)
		: Vector3(org),
		  m_orgIndex(orgIndex)
	{
	}
	scalar m_angle;
	int m_orgIndex;
};


namespace date_n {
	static GrahamVector3 Anchor(Vector3::ZERO,0);
	static Vector3 axis0, axis1;


	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are collinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	static int orientation(GrahamVector3 p, GrahamVector3 q, GrahamVector3 r)
	{
		auto px = axis0.Dot(p);
		auto py = axis1.Dot(p);

		auto qx = axis0.Dot(q);
		auto qy = axis1.Dot(q);

		auto rx = axis0.Dot(r);
		auto ry = axis1.Dot(r);

		int val = (qy - py) * (rx - qx) -
				  (qx - px) * (ry - qy);

		if (val == 0) return 0;  // collinear
		return (val > 0)? 1: 2; // clock or counterclock wise
	}


	static int comparee(const void *vp1, const void *vp2)
	{
	   GrahamVector3 *p1 = (GrahamVector3 *)vp1;
	   GrahamVector3 *p2 = (GrahamVector3 *)vp2;

	   // Find orientation
	   int o = orientation(Anchor, *p1, *p2);
	   if (o == 0)
		 return ((Anchor - *p2).LengthSquare() >= (Anchor - *p1).LengthSquare())? -1 : 1;

	   return (o == 2)? -1: 1;
	}
}




struct IAngleCompareFunc
{
		Vector3 m_anchor;
		IAngleCompareFunc(const Vector3& anchor)
		: m_anchor(anchor)
		{
		}

		bool operator()(const GrahamVector3& a, const GrahamVector3& b) const
		{
//			if (a.m_angle != b.m_angle)
			{
				return a.m_angle < b.m_angle;
			}
//			else
//			{
//				scalar al = (a - m_anchor).LengthSquare();
//				scalar bl = (b - m_anchor).LengthSquare();
//				if (al != bl)
//				{
//					return (al > bl);
//				}
//				else
//				{
//					return (a.m_orgIndex < b.m_orgIndex);
//				}
//			}
		}
};

inline void GrahamScanConvexHull2D(std::vector<GrahamVector3>& originalPoints, std::vector<GrahamVector3>& hull, const Vector3& normalAxis)
{

	Vector3::BiUnitOrthogonalVector(normalAxis, date_n::axis0, date_n::axis1);

	if (originalPoints.size() <= 1)
	{
		for (unsigned int i = 0; i < originalPoints.size(); i++)
		{
			hull.push_back(originalPoints[0]);
		}
		return;
	}
	//step1 : find anchor point with smallest projection on axis0 and move it to first location
	for (unsigned int i = 0; i < originalPoints.size(); i++)
	{
		//		const btVector3& left = originalPoints[i];
		//		const btVector3& right = originalPoints[0];
		scalar projL = originalPoints[i].Dot(date_n::axis0);
		scalar projR = originalPoints[0].Dot(date_n::axis0);
		if (projL < projR)
		{
			IMath::ISwap(originalPoints[0], originalPoints[i]);
		}
	}

	//also precompute angles
	originalPoints[0].m_angle = -1e30f;
	for (unsigned int i = 1; i < originalPoints.size(); i++)
	{
		Vector3 ar = originalPoints[i] - originalPoints[0];
		scalar ar1 = date_n::axis1.Dot(ar);
		scalar ar0 = date_n::axis0.Dot(ar);
		if (ar1 * ar1 + ar0 * ar0 < FLT_EPSILON)
		{
			originalPoints[i].m_angle = -1e30f;
		}
		else
		{
			originalPoints[i].m_angle =  IMath::IAtan2(ar1, ar0);
		}
	}

//	//step 2: sort all points, based on 'angle' with this anchor
	IAngleCompareFunc comp(originalPoints[0]);
	//originalPoints.quickSortInternal(comp, 1, originalPoints.size() - 1);
	std::sort(originalPoints.begin()+1, originalPoints.end()-1, comp);


//	date_n::Anchor = originalPoints[0];
//	qsort(&originalPoints[1], originalPoints.size()-1, sizeof(GrahamVector3), date_n::comparee);




	unsigned int i;
	for (i = 0; i < 3; i++)
		hull.push_back(originalPoints[i]);

	//step 3: keep all 'convex' points and discard concave points (using back tracking)
	for (; i < originalPoints.size(); i++)
	{
		bool isConvex = false;
		while (!isConvex && hull.size() > 1)
		{
			Vector3& a = hull[hull.size() - 2];
			Vector3& b = hull[hull.size() - 1];
			isConvex = IMath::Cross(a - b, a - originalPoints[i]).Dot(normalAxis) > 0;
			if (!isConvex)
			{
				hull.pop_back();
			}
			else
			{
				hull.push_back(originalPoints[i]);
			}
		}

		if (hull.size() == 1)
		{
			hull.push_back(originalPoints[i]);
		}
	}
}


static int num_iteration;


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



class IGrahamScan2dConvexHull
{


//      static Vector3 SupportPoint(const Vector3& direction, const std::vector<Vector3> points)
//      {
//          unsigned int index = 0;
//          scalar max = (points[0].Dot(direction));

//          for (unsigned int i = 1; i < points.size(); i++)
//          {
//              scalar d = (points[i].Dot(direction));
//              if (d > max)
//              {
//                  max = d;
//                  index = i;
//              }
//          }

//          return points[index];
//      }


    public:
        IGrahamScan2dConvexHull();


        static void ScanConvexHull2D2(const CallbeckSupprtModel& originalPoints,
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

                if((b - plane.ClosestPoint(_a)).LengthSquare() > 0.01 &&
                   (b - plane.ClosestPoint(_b)).LengthSquare() > 0.01 &&
                   (b - prev).LengthSquare() > 0.01)
                {
                    hull.push_back(b - Axis.Normalized() * 15);
                }

                prev = b;

                if(type_stop == test::UNKNOW)
                {
                   if((plane.ClosestPoint(_b) - b).LengthSquare() < 0.001 ) type_stop = test::B;
                   if((plane.ClosestPoint(_a) - b).LengthSquare() < 0.001 ) type_stop = test::A;
                }
                else
                {
                    if(type_stop == test::A)
                    {
                        if((plane.ClosestPoint(_b) - b).LengthSquare() < 0.001 ) break;
                    }
                    else if(type_stop == test::B)
                    {
                        if((plane.ClosestPoint(_a) - b).LengthSquare() < 0.001 ) break;
                    }
                }

                start_p0 = out;
                start_p1 = plane.ClosestPoint(_a);

            }


            hull.push_back( _a );

        }



        static void ScanConvexHull2D(const CallbeckSupprtModel& originalPoints, std::vector<Vector3>& hull, const IPlane& plane )
        {

            Vector3 normalAxis = plane.GetNormal();

            //------------------------------------------------------//

            Vector3 OrthogonalVector = normalAxis.GetOneUnitOrthogonalVector();
            Vector3 start_p0 = originalPoints.SupportPoint(-OrthogonalVector);
            Vector3 start_p1 = originalPoints.SupportPoint( OrthogonalVector);

            Vector3 line = plane.ClosestPoint(start_p0 - start_p1);
            Vector3 Axis = IMath::Cross(line,normalAxis);
            if( (line).Dot(Axis) > 0 )
            {
               // IMath::ISwap(start_p0,start_p1);
            }

            //------------------------------------------------------//

            const Vector3 start = start_p0;
            Vector3 out;
            Vector3 prev = start_p1;

            while(start != out)
            {
                ClosingEdgePoint(originalPoints,hull,normalAxis,start_p0,start_p1,plane,out);

                Vector3 a = plane.ClosestPoint(start_p0);
                Vector3 b = plane.ClosestPoint(out);
                Vector3 axis = IMath::Cross(a - b, normalAxis);

                if((a-b).Length() > 0.01f /*&& (b-start).Length() > 0.01f*/)
                {
                    hull.push_back( /*originalPoints.GetTransform().GetTranslation() +*/ b );
                }
                else if(!hull.empty())
                {
                    start_p0 = prev;
                }

                start_p0 = out;
                start_p1 = originalPoints.SupportPoint(axis);
                prev = out;
            }

             //------------------------------------------------------//

        }



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
            num_iteration++;

            if(p != p1) out = (p);

            if( (distance) > 0.0001f)
            {
               ClosingEdgePoint(originalPoints,hull,normalAxis,p1,p,plane,out,i+1);
            }
            else if(p == p1)
            {
               out = (p2);
            }
        }



};

#endif // IGRAHAMSCAN2DCONVEXHULL_H
