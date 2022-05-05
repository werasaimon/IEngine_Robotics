#ifndef GJKALGORITHM_H
#define GJKALGORITHM_H

// Libraries
#include "../ICollisionShapeInfo.h"
#include "../../IRaycastInfo.h"
#include "../../ICollisionShapes/ICollisionShapeConvex.h"
#include "../../IProxyShape.h"
#include "../../../Segments/IRay.h"



namespace IEngine
{

// Constants
const scalar REL_ERROR = scalar(1.0e-4);
const scalar REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
const int MAX_ITERATIONS_GJK_RAYCAST = 32;




class IGJKIntersection
{

    private :

        //-------------------- Methods --------------------//
        /// Private copy-constructor
        IGJKIntersection(const IGJKIntersection& algorithm);

        /// Private assignment operator
        IGJKIntersection& operator=(const IGJKIntersection& algorithm);


    public :


        /// Constructor
        IGJKIntersection();

        /// Destructor
        ~IGJKIntersection();


        /// Use the GJK Algorithm to find if a point is inside a convex collision shape
        bool testPointInside(const Vector3 &localPoint, IProxyShape *proxyShape);


        /// Ray casting algorithm agains a convex collision shape using the GJK Algorithm
        bool raycast(const IRay& ray, IRaycastInfo& raycastInfo , IProxyShape* proxyShape );


};



} /* namespace real_physics */




#endif // GJKALGORITHM_H
