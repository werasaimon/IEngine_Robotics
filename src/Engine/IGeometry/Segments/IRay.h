/********************************************************************************
 *
 * IRay.h
 *
 * IMath : 3d_math library,
 * Copyright (c)  *
 * Created on: 3 July. 2018 г.
 * Author: werasaimon                                     *
 *********************************************************************************
 *                                                                               *
 * This software is provided 'as-is', without any express or implied warranty.   *
 * In no event will the authors be held liable for any damages arising from the  *
 * use of this software.                                                         *
 *                                                                               *
 * Permission is granted to anyone to use this software for any purpose,         *
 * including commercial applications, and to alter it and redistribute it        *
 * freely, subject to the following restrictions:                                *
 *                                                                               *
 * 1. The origin of this software must not be misrepresented; you must not claim *
 *    that you wrote the original software. If you use this software in a        *
 *    product, an acknowledgment in the product documentation would be           *
 *    appreciated but is not required.                                           *
 *                                                                               *
 * 2. Altered source versions must be plainly marked as such, and must not be    *
 *    misrepresented as being the original software.                             *
 *                                                                               *
 * 3. This notice may not be removed or altered from any source distribution.    *
 *                                                                               *
 ********************************************************************************/

#ifndef IRAY_H
#define IRAY_H

#include "../../imaths.hpp"

namespace IEngine
{


class IRaycast
{
   public:

    scalar distance;
};

//-------------------------------//

class IRay
{
public:

    // -------------------- Attributes -------------------- //

    /// First point of the ray (origin)
    Vector3 Origin;

    /// Second point of the ray
    Vector3 Direction;

    /// Maximum fraction value
    scalar maxFraction;


    // -------------------- Methods -------------------- //


    /// Constructor default
    SIMD_INLINE IRay(){}

    /// Constructor with arguments
    SIMD_INLINE IRay(const Vector3& _origin, const Vector3& _direction, scalar maxFrac = (1.0))
        : Origin(_origin),
          Direction(_direction),
          maxFraction(maxFrac)
    {
    }

    /// Copy-constructor
    SIMD_INLINE IRay(const IRay& ray)
        : Origin(ray.Origin),
          Direction(ray.Direction),
          maxFraction(ray.maxFraction)
    {

    }


    /// Overloaded assignment operator
    SIMD_INLINE IRay& operator=(const IRay& ray)
;

#ifdef ENABLE_STL_SUPPORT

    //-------------[ output operator ]------------------------
    /**
    * Output to stream operator
    * @param lhs Left hand side argument of operator (commonly ostream instance).
    * @param rhs Right hand side argument of operator.
    * @return Left hand side argument - the ostream object passed to operator.
    */
    friend std::ostream& operator<<(std::ostream& lhs, const IRay& rhs)
    {
        lhs << "Origin :" << rhs.Origin << " , " <<  "Direction :" << rhs.Direction << " , " << "maxFriction:" << rhs.maxFraction;
        return lhs;
    }

    /**
    * Gets string representation.
    */
    std::string ToString() const
    {
        std::ostringstream oss;
        oss << *this;
        return oss.str();
    }

#endif

};


} /* namespace */



#endif // IRAY_H
