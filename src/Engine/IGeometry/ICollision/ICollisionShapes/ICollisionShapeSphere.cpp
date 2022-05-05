#include "ICollisionShapeSphere.h"
#include "../IProxyShape.h"

namespace IEngine
{

// Default initilization
#define DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS	    1
#define DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT 0.01;// 0.00001

// Constructor
/**
 * @param radius Radius of the sphere (in meters)
 */
ICollisionShapeSphere::ICollisionShapeSphere( scalar radius )
: ICollisionShapeConvex(SPHERE, radius)
{
    assert(radius > scalar(0.0));
    mNbMaxPeturberationIteration = DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS; // maximum iteration  for Axis Peturberation
    mEpsilonPeturberation        = DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT;// epsilon for Peturberation
}

// Destructor
ICollisionShapeSphere::~ICollisionShapeSphere()
{

}



// Raycast method with feedback information
bool ICollisionShapeSphere::Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const
{

        scalar radius = mMargin;


//        std::cout << "radius " << radius << std::endl;


//            Vector3 oc = ray.Origin - Vector3(0,0,0);
//            float a = Dot(ray.Direction, ray.Direction);
//            float b = 2.0 * Dot(oc, ray.Direction);
//            float c = Dot(oc,oc) - radius*radius;
//            float discriminant = b*b - 4*a*c;

//            if(discriminant>0)
//            {
//                raycastInfo.body = proxyShape->GetBody();
//                raycastInfo.proxyShape = proxyShape;
////                raycastInfo.hitFraction = t;
////                raycastInfo.worldPoint = ray.Origin + t * rayDirection;
////                raycastInfo.worldNormal = raycastInfo.worldPoint;
//            }


        /**/
        Vector3 m = ray.Origin;
        float a = Dot(ray.Direction, ray.Direction);
        float b = 2.0 * Dot(m, ray.Direction);    if (b > scalar(0.0)) return false;
        float c = Dot(m,m) - radius*radius;       if (c < scalar(0.0)) return false;
        float discriminant = b*b - 4.0*a*c;

        // If the discriminant is negative or the ray length is very small, there is no intersection
        if(discriminant < scalar(0.0) || a < MACHINE_EPSILON) return false;

        // intersection collision
        if(discriminant > scalar(0.0))
        {
            float t = (-b - sqrt(discriminant)) / (scalar(2.0)*a);

            raycastInfo.body = proxyShape->GetBody();
            raycastInfo.proxyShape  = proxyShape;
            raycastInfo.hitFraction = t;
            raycastInfo.worldPoint  = ray.Origin + t * ray.Direction;
            raycastInfo.worldNormal = raycastInfo.worldPoint;

            return true;
        }

        return false;
        /**/


                    //        const Vector3 m = ray.Origin;
                    //        scalar c = m.Dot(m) - radius * radius;

                    //        //        // If the origin of the ray is inside the sphere, we return no intersection
                    //        //        if (c < scalar(0.0)) return false;

                    //        const Vector3 rayDirection = ray.Direction;
                    //        scalar b = m.Dot(rayDirection);

                    //        //        // If the origin of the ray is outside the sphere and the ray
                    //        //        // is pointing away from the sphere, there is no intersection
                    //        //        if (b > scalar(0.0)) return false;

                    //        scalar a = rayDirection.LengthSquare();

                    //        // Compute the discriminant of the quadratic equation
                    //        //scalar discriminant = b * b - a * c;

                    //        float discriminant = b*b - 4*a*c;


                    //        if(discriminant > 0)
                    //        {
                    //            float t = (-b - sqrt(discriminant)) / (2.0*a);

                    //            // Compute the intersection information
                    //            //t /= a;

                    //            raycastInfo.body = proxyShape->GetBody();
                    //            raycastInfo.proxyShape = proxyShape;
                    //            raycastInfo.hitFraction = t;
                    //            raycastInfo.worldPoint = ray.Origin + t * rayDirection;
                    //            raycastInfo.worldNormal = raycastInfo.worldPoint;

                    //            return true;
                    //        }

                    //        return false;


        // If the discriminant is negative or the ray length is very small, there is no intersection
       // if (discriminant < scalar(0.0) || a < MACHINE_EPSILON) return false;

        // Compute the solution "t" closest to the origin
       // scalar t = -b - ISqrt(discriminant);

        //        assert(t >= scalar(0.0));

//        // If the hit point is withing the segment ray fraction
//        if (t < /*ray.maxFraction * raySquareLength*/0)
//        {

//            // Compute the intersection information
//            t /= a;

//            raycastInfo.body = proxyShape->GetBody();
//            raycastInfo.proxyShape = proxyShape;
//            raycastInfo.hitFraction = t;
//            raycastInfo.worldPoint = ray.Origin + t * rayDirection;
//            raycastInfo.worldNormal = raycastInfo.worldPoint;

//            return true;
//        }

//        return false;

}

#undef DEFAULT_SPHERE_MAX_PETURBERATION_ITERATIONS
#undef DEFAULT_SPHERE_EPS_PETURBERATION_ANGLES_COFFICIENT

}
