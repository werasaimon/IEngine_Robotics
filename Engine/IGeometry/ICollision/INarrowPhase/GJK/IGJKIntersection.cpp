#include "IGJKIntersection.h"
#include "Simplex.h"



namespace IEngine
{





IGJKIntersection::IGJKIntersection()
{

}

IGJKIntersection::~IGJKIntersection()
{

}



// Use the GJK Algorithm to find if a point is inside a convex collision shape
bool IGJKIntersection::testPointInside(const Vector3 &localPoint, IProxyShape *proxyShape)
{
    Vector3 suppA;             // Support point of object A
    Vector3 w;                 // Support point of Minkowski difference A-B
    scalar  prevDistSquare;

    // ghghhg
    const ICollisionShapeConvex* shape = static_cast<const ICollisionShapeConvex*>(proxyShape->GetCollisionShape());

    // world transform
    const Transform& transWorld = proxyShape->GetWorldTransform();

    // Support point of object B (object B is a single point)
    const Vector3 suppB(localPoint);

    // Create a simplex set
    Simplex simplex;

    // Initial supporting direction
    Vector3 v(1, 1, 1);

    // Initialize the upper bound for the square distance
    scalar distSquare = 0.0001;

    do
    {

        // Compute the support points for original objects (without margins) A and B
        suppA = transWorld * shape->GetLocalSupportPointWithoutMargin( /*transWorld.GetBasis().GetInverse() **/ -v );

        // Compute the support point for the Minkowski difference A-B
        w = suppA - suppB;

        // Add the new support point to the simplex
        simplex.addPoint(w, suppA, suppB);

        // If the simplex is affinely dependent
        if (simplex.isAffinelyDependent())
        {

            return false;
        }

        // Compute the point of the simplex closest to the origin
        // If the computation of the closest point fail
        if (!simplex.computeClosestPoint(v))
        {

            return false;
        }

        // Store and update the squared distance of the closest point
        prevDistSquare = distSquare;
        distSquare = v.LengthSquare();

        // If the distance to the closest point doesn't improve a lot
        if (prevDistSquare - distSquare <= MACHINE_EPSILON * prevDistSquare)
        {

            return false;
        }

    }
    while(!simplex.isFull() && distSquare > MACHINE_EPSILON *
           simplex.getMaxLengthSquareOfAPoint());

    // The point is inside the collision shape
    return true;

}



// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
/// This method implements the GJK ray casting algorithm described by Gino Van Den Bergen in
/// "Ray Casting against General Convex Objects with Application to Continuous Collision Detection".
bool IGJKIntersection::raycast(const IRay& ray, IRaycastInfo& raycastInfo ,IProxyShape* proxyShape )
{

    const ICollisionShapeConvex* shape = static_cast<const ICollisionShapeConvex*>(proxyShape->GetCollisionShape());

    Vector3 suppA;      // Current lower bound point on the ray (starting at ray's origin)
    Vector3 suppB;      // Support point on the collision shape
    const scalar machineEpsilonSquare = MACHINE_EPSILON * MACHINE_EPSILON;
    const scalar epsilon = scalar(0.0001);

    // World transform
   // const Transform& transWorld = proxyShape->GetWorldTransform();

    // Convert the ray origin and direction into the local-space of the collision shape
    Vector3 rayDirection = ray.Direction;

    // If the points of the segment are two close, return no hit
    if (rayDirection.LengthSquare() < machineEpsilonSquare) return false;

    Vector3 w;


    // Create a simplex set
    Simplex simplex;

    Vector3 n(scalar(0.0), scalar(0.0), scalar(0.0));
    scalar lambda = scalar(0.0);
    suppA = ray.Origin;    // Current lower bound point on the ray (starting at ray's origin)
    suppB = /* transWorld **/ shape->GetLocalSupportPointWithoutMargin( /*transWorld.GetBasis().GetInverse() **/ rayDirection );
    Vector3 v = suppA - suppB;
    scalar vDotW, vDotR;
    scalar distSquare = v.LengthSquare();
    int nbIterations = 0;

    // GJK Algorithm loop
    while (distSquare > epsilon && nbIterations < MAX_ITERATIONS_GJK_RAYCAST)
    {
        // Compute the support points
        suppB = /*transWorld **/ shape->GetLocalSupportPointWithoutMargin( /*transWorld.GetBasis().GetInverse() **/ v );
        w = suppA - suppB;

        vDotW = v.Dot(w);



        if (vDotW > scalar(0))
        {
            vDotR = v.Dot(rayDirection);

            if (vDotR >= -machineEpsilonSquare)
            {
                return false;
            }
            else
            {
                // We have found a better lower bound for the hit point along the ray
                lambda = lambda - vDotW / vDotR;
                suppA = ray.Origin + lambda * rayDirection;
                w = suppA - suppB;
                n = v;
            }
        }

        // Add the new support point to the simplex
        if (!simplex.isPointInSimplex(w))
        {
            simplex.addPoint(w, suppA, suppB);
        }

        // Compute the closest point
        if (simplex.computeClosestPoint(v))
        {
            distSquare = v.LengthSquare();
        }
        else
        {
            distSquare = scalar(0.0);
        }


        //  std::cout << "WWW  " << lambda <<std::endl;

        // If the current lower bound distance is larger than the maximum raycasting distance
        //if (lambda > 0 /*ray.maxFraction*/) return false;

        nbIterations++;
    }

    // If the origin was inside the shape, we return no hit
    if (lambda < MACHINE_EPSILON) return false;

    // Compute the closet points of both objects (without the margins)
    Vector3 pointA;
    Vector3 pointB;
    simplex.computeClosestPointsOfAandB(pointA, pointB);

    // A raycast hit has been found, we fill in the raycast info
    raycastInfo.hitFraction = lambda;
    raycastInfo.worldPoint = pointB;
    raycastInfo.body = proxyShape->GetBody();
    raycastInfo.proxyShape = proxyShape;

    if (n.LengthSquare() >= machineEpsilonSquare )
    { // The normal vector is valid
        raycastInfo.worldNormal = n;
    }
    else
    {  // Degenerated normal vector, we return a zero normal vector
        raycastInfo.worldNormal = Vector3(scalar(0),
                                          scalar(0),
                                          scalar(0));
    }

    return true;
}







}
