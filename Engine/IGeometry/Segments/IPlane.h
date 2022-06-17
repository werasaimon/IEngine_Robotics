 /********************************************************************************
 *
 * IPlane.h
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


#ifndef IPLANE_H
#define IPLANE_H

#include "../../imaths.hpp"
#include "ILine3D.h"
#include "ILineSegment3D.h"

namespace IEngine
{


//-------------------------------------------------------------------------------
//-- Classes --------------------------------------------------------------------
//-------------------------------------------------------------------------------
class IPlane
{

protected:


   // IVector3D<T>  mOrigin;
    Vector3  mNormal;
    scalar   mOffset;


public:
    // constructor/destructor

    enum  INTERSECTION_TYPE { INTERSECT = 0 , INSIDE = 1  , OUTSIDE = 2 };


    // Set this plane from a normal and a signed distance from its origin.
    IPlane( const Vector4& _v4 )
      : mNormal(_v4.GetXYZ()) ,
        mOffset(_v4.GetW())
    {

    }

    // Set this plane from a normal and a signed distance from its origin.
    IPlane( const Vector3& normal = Vector3(1,0,0) , scalar offset = scalar(0) )
      : mNormal(normal) ,
        mOffset(offset)
    {

    }

    // Set this plane from a normal and a point on the plane.
    IPlane(const Vector3& _normal, const Vector3& _point)
    {
        mNormal = _normal;
        mOffset = _normal.Dot(_point);
       // mOrigin = _point;
    }

    IPlane( const Vector3& p0,
            const Vector3& p1,
            const Vector3& p2 )
    {
       Set( p0, p1, p2 );
    }


    IPlane( scalar a, scalar b, scalar c, scalar d )
    {
       Set( a, b, c, d );
    }

    // copy operations
    IPlane(const IPlane& other)
     : mNormal( other.mNormal ),
       mOffset( other.mOffset )
    {

    }


    SIMD_INLINE ~IPlane() {}


    // ---------------------------------------------------------------------------
    // Assigment operator
    //-----------------------------------------------------------------------------
    IPlane operator=(const IPlane &other);


    // accessors
    SIMD_INLINE const Vector3& GetNormal() const { return mNormal; }
    SIMD_INLINE scalar GetOffset() const { return mOffset; }

    // ---------------------------------------------------------------------------
    // Returns the two endpoints
    //-----------------------------------------------------------------------------
    void Get( Vector3& normal, scalar& offset ) const
    {
        normal = mNormal;
        offset = mOffset;
    }

    // ---------------------------------------------------------------------------
    // Are two IPlane's equal?
    //----------------------------------------------------------------------------
    bool operator==( const IPlane&  plane ) const
    {
        return (plane.mNormal == mNormal &&
                plane.mOffset == mOffset);
    }

    // ---------------------------------------------------------------------------
    // Are two IPlane's not equal?
    //----------------------------------------------------------------------------
    bool operator!=( const IPlane&  plane ) const;

    // manipulators
    SIMD_INLINE void Set( const Vector3& n, scalar d )
    {
        Set( n.x, n.y, n.z, d );
    }

    // ---------------------------------------------------------------------------
    // Sets the parameters
    //-----------------------------------------------------------------------------
    void Set( scalar a, scalar b, scalar c, scalar d );


    // ---------------------------------------------------------------------------
    // Sets the parameters
    //-----------------------------------------------------------------------------
    void Set( const Vector3& p0, const Vector3& p1, const Vector3& p2 );


    // ---------------------------------------------------------------------------
    // Gets the parameters plane
    //-----------------------------------------------------------------------------
    const Vector4 &GetVec4();


    // ---------------------------------------------------------------------------
    // Transforms plane into new space
    //-----------------------------------------------------------------------------
    IPlane Transform( scalar scale, const Quaternion& rotate, const Vector3& translate ) const;

    // ---------------------------------------------------------------------------
    // Transforms plane into new space
    //-----------------------------------------------------------------------------
    IPlane Transform( scalar scale, const Matrix3& rotmatrix, const Vector3& translate ) const;

    // distance
    static scalar Distance( const IPlane& plane, const Vector3& point );

    // ---------------------------------------------------------------------------
    // Returns the closest point on plane to point
    //-----------------------------------------------------------------------------
    Vector3 ClosestPoint( const Vector3& point ) const;

    // result of plane test
    scalar Test( const Vector3& point ) const;

    // result of plane test
    scalar InvTest( const Vector3& point ) const;


    // vIntersectionLineToPlane(): find the 3D intersection of a segment and a plane
    //    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
    //    Output: *I0 = the intersect point (when it exists)
    Vector3 VIntersectionLineToPlane( const ILineSegment3D& _edge ) const;


    // vIntersectionLineToRay(): find the 3D intersection of a segment and a plane
    //    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
    //    Output: *I0 = the intersect point (when it exists)
    Vector3 VIntersectionRayToPlane( const Vector3& _ray_origin , const Vector3& _ray_dir ) const;


    //=================================================================//


   // unsigned int AABBIntersect( const Vector3& aabb_min , const Vector3& aabb_max) const;

    //=================================================================//


    unsigned int AABBIntersect( const Vector3& aabb_min , const Vector3& aabb_max) const;



    //=================================================================//


#ifdef ENABLE_STL_SUPPORT

    //----------[ output operator ]----------------------------
    /**
    * Provides output to standard output stream.
    */
    friend std::ostream& operator <<(std::ostream& oss, const IPlane& rhs)
    {
        oss << "(" << "normal: " << rhs.mNormal << " offset: " << rhs.mOffset << ")";
        return oss;
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


}


#endif // IPLANE_H
