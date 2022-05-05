#include "IRaycastInfo.h"
#include "IProxyShape.h"

namespace IEngine
{

// Ray cast test against a proxy shape
scalar IRaycastTest::raycastAgainstShape(IProxyShape* shape, const IRay& ray)
{

    // Ray casting test against the collision shape
    IRaycastInfo raycastInfo;
    bool isHit = shape->Raycast(ray, raycastInfo);

    std::cout << "Ray  - TEST  : " << isHit << std::endl;

    // If the ray hit the collision shape
    if (isHit)
    {
        // Report the hit to the user and return the
        // user hit fraction value
        return userCallback->notifyRaycastHit(raycastInfo);
    }

    return ray.maxFraction;
}


}
