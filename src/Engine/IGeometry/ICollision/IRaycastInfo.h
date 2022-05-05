#ifndef IRAYCASTINFO_H
#define IRAYCASTINFO_H

#include "../../ICommon/ISettings.h"
#include "../../imaths.hpp"
#include "../../IGeometry/Segments/IRay.h"

namespace IEngine
{

// Declarations
class ICollisionBody;
class IProxyShape;
class ICollisionShape;

// Structure RaycastInfo
/**
 * This structure contains the information about a raycast hit.
 */
struct IRaycastInfo
{

    private:

        // -------------------- Methods -------------------- //

        /// Private copy constructor
        IRaycastInfo(const IRaycastInfo& IRaycastInfo);

        /// Private assignment operator
        IRaycastInfo& operator=(const IRaycastInfo& IRaycastInfo);

    public:

        // -------------------- Attributes -------------------- //

        /// Hit point in world-space coordinates
        Vector3 worldPoint;

        /// Surface normal at hit point in world-space coordinates
        Vector3 worldNormal;

        /// Fraction distance of the hit point between point1 and point2 of the ray
        /// The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
        scalar hitFraction;

        /// Mesh subpart index that has been hit (only used for triangles mesh and -1 otherwise)
        i32 meshSubpart;

        /// Hit triangle index (only used for triangles mesh and -1 otherwise)
        i32 triangleIndex;

        /// Pointer to the hit collision body
        ICollisionBody* body;

        /// Pointer to the hit proxy collision shape
        IProxyShape* proxyShape;

        // -------------------- Methods -------------------- //

        /// Constructor
        IRaycastInfo()
        : meshSubpart(-1), triangleIndex(-1) , proxyShape(nullptr)
        {

        }

        /// Destructor
        ~IRaycastInfo()
        {

        }
};


// Class RaycastCallback
/**
 * This class can be used to register a callback for ray casting queries.
 * You should implement your own class inherited from this one and implement
 * the notifyRaycastHit() method. This method will be called for each ProxyShape
 * that is hit by the ray.
 */
class IRaycastCallback
{

    public:

        // -------------------- Methods -------------------- //


        IRaycastCallback()
        {

        }

        /// Destructor
        virtual ~IRaycastCallback()
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
        virtual scalar notifyRaycastHit(const IRaycastInfo& IRaycastInfo)=0;

};

/// Structure RaycastTest
struct IRaycastTest
{

    public:

        /// User callback class
        IRaycastCallback* userCallback;

        /// Constructor
        IRaycastTest(IRaycastCallback* callback)
        {
            userCallback = callback;
        }

        /// Ray cast test against a proxy shape
        scalar raycastAgainstShape(IProxyShape* shape, const IRay& ray);
};

}

#endif // IRAYCASTINFO_H
