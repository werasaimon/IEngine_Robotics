 /********************************************************************************
 *
 * ILineSegment3D.h
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


#ifndef ILINESEGMENT3D_H
#define ILINESEGMENT3D_H

#include "ILine3D.h"

namespace IEngine
{

class ILineSegment3D;

scalar DistanceSquared( const ILineSegment3D& seg0, const ILineSegment3D& seg1, scalar& s_c, scalar& t_c );
scalar DistanceSquared( const ILineSegment3D& segment, const ILine3D& line,  scalar& s_c, scalar& t_c );

//-------------------------------------------------------------------------------
//-- Classes --------------------------------------------------------------------
//-------------------------------------------------------------------------------
class ILineSegment3D
{

protected:

    Vector3 mOrigin;
    Vector3 mDirection;

public:
    // constructor/destructor
    ILineSegment3D()
     : mOrigin( 0.0f, 0.0f, 0.0f ),
       mDirection( 1.0f, 0.0f, 0.0f )
    {

    }

    ILineSegment3D( const Vector3& endpoint0, const Vector3& endpoint1 )
     : mOrigin( endpoint0 ),
       mDirection( endpoint1-endpoint0 )
    {

    }

    SIMD_INLINE ~ILineSegment3D() {}

    // copy operations
    ILineSegment3D(const ILineSegment3D& other)
     : mOrigin( other.mOrigin ),
       mDirection( other.mDirection )
    {

    }

    // ---------------------------------------------------------------------------
    // Assigment operator
    //-----------------------------------------------------------------------------
    ILineSegment3D& operator=(const ILineSegment3D& other);


    // accessors
//    scalar& operator()(unsigned int i, unsigned int j);
//    scalar  operator()(unsigned int i, unsigned int j) const;

    SIMD_INLINE Vector3 GetOrigin() const { return mOrigin; }
    SIMD_INLINE Vector3 GetDirection() const { return mDirection; }
    SIMD_INLINE Vector3 GetEndpoint0() const { return mOrigin; }
    SIMD_INLINE Vector3 GetEndpoint1() const { return mOrigin + mDirection; }
    SIMD_INLINE Vector3 GetCenter() const { return mOrigin + scalar(0.5f)*mDirection; }

    // ---------------------------------------------------------------------------
    // Returns the two endpoints
    //-----------------------------------------------------------------------------
    void Get( Vector3& end0, Vector3& end1 ) const;
    void Set( const Vector3& end0, const Vector3& end1 );

    // ---------------------------------------------------------------------------
    // Returns the distance between two endpoints
    //-----------------------------------------------------------------------------
    scalar Length() const;

    // ---------------------------------------------------------------------------
    // Returns the squared distance between two endpoints
    //-----------------------------------------------------------------------------
    scalar LengthSquared() const;

    // ---------------------------------------------------------------------------
    // Are two IvLineSegment3's equal?
    //----------------------------------------------------------------------------
    bool operator==( const ILineSegment3D& segment ) const;

    // ---------------------------------------------------------------------------
    // Are two IvLineSegment3's not equal?
    //----------------------------------------------------------------------------
    bool operator!=( const ILineSegment3D& segment ) const;



    // ---------------------------------------------------------------------------
    // Transforms segment into new space
    //-----------------------------------------------------------------------------
    ILineSegment3D Transform( scalar scale, const Quaternion& quat, const Vector3& translate ) const;

    // ---------------------------------------------------------------------------
    // Transforms segment into new space
    //-----------------------------------------------------------------------------
    ILineSegment3D Transform( scalar scale, const  Matrix3& rotate, const Vector3& translate ) const;



    // ---------------------------------------------------------------------------
    // Returns the distance squared between two line segments.
    // Based on article and code by Dan Sunday at www.geometryalgorithms.com
    //-----------------------------------------------------------------------------
    static scalar DistanceSquared( const ILineSegment3D& segment0, const ILineSegment3D& segment1,  scalar& s_c, scalar& t_c );

    // ---------------------------------------------------------------------------
    // Returns the distance squared between line segment and line.
    // Based on article and code by Dan Sunday at www.geometryalgorithms.com
    //-----------------------------------------------------------------------------
    static scalar DistanceSquared( const ILineSegment3D& segment,  const ILine3D& line,  scalar& s_c, scalar& t_c );
    static scalar DistanceSquared( const ILine3D& line,  const ILineSegment3D& segment, scalar& s_c, scalar& t_c );

    // ---------------------------------------------------------------------------
    // Returns the distance squared between line segment and point.
    //-----------------------------------------------------------------------------
    static scalar DistanceSquared( const ILineSegment3D& segment,  const Vector3& point,  scalar& t_c );



    // ---------------------------------------------------------------------------
    // Returns the closest points between two line segments.
    //-----------------------------------------------------------------------------
    static void ClosestPoints( const ILineSegment3D& segment0,
                               const ILineSegment3D& segment1 ,
                               Vector3& point0,
                               Vector3& point1 );


    // ---------------------------------------------------------------------------
    // Returns the closest points between line segment and line.
    //-----------------------------------------------------------------------------
    static void ClosestPoints( const ILineSegment3D& segment,
                               const ILine3D& line ,
                               Vector3& point0,
                               Vector3& point1 );

    // ---------------------------------------------------------------------------
    // Returns the closest point on line segment to point
    //-----------------------------------------------------------------------------
    Vector3 ClosestPoint( const Vector3& point ) const;




    // returns true if (p) is on lineSegment [GetEndpoint0(), GetEndpoint1()]
    bool IsPointOnLine(const Vector3 p);


#ifdef ENABLE_STL_SUPPORT

    //----------[ output operator ]----------------------------
    /**
    * Provides output to standard output stream.
    */
    friend std::ostream& operator <<(std::ostream& oss, const ILineSegment3D& rhs)
    {
        oss << "(" << "origin: " << rhs.mOrigin << " direction: " << rhs.mDirection << ")";
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

#endif // ILINESEGMENT3D_H
