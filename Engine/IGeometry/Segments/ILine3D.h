 /********************************************************************************
 *
 * ILine3D.h
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

#ifndef ILINE3D_H
#define ILINE3D_H


#include "../../imaths.hpp"

namespace IEngine
{

class ILine3D
{

protected:

    Vector3 mOrigin;
    Vector3 mDirection;


public:
      // constructor/destructor
      ILine3D()
      : mOrigin( 0.0f, 0.0f, 0.0f ),
        mDirection( 1.0f, 0.0f, 0.0f )
      {

      }

      ILine3D( const Vector3& origin, const Vector3& direction )
      : mOrigin( origin ),
        mDirection( direction )
      {
          mDirection.Normalize();
      }

      // copy operations
      ILine3D(const ILine3D& other)
       : mOrigin( other.mOrigin ),
         mDirection( other.mDirection )
      {

      }


      ~ILine3D() {}


      // ---------------------------------------------------------------------------
      // Assigment operator
      //-----------------------------------------------------------------------------
      ILine3D& operator=(const ILine3D& other);

      // ---------------------------------------------------------------------------
      // Returns the two endpoints
      //-----------------------------------------------------------------------------
      void Get( Vector3& origin, Vector3& direction ) const;

      // manipulators
      void Set( const Vector3& origin, const Vector3& direction );


      // accessors
      SIMD_INLINE Vector3 GetOrigin() const { return mOrigin; }
      SIMD_INLINE Vector3 GetDirection() const { return mDirection; }



      // comparison
      bool operator==( const ILine3D& line ) const;
      bool operator!=( const ILine3D& line ) const;


      // ---------------------------------------------------------------------------
      // Transforms ray into new space
      //-----------------------------------------------------------------------------
      ILine3D Transform(scalar scale, const Quaternion& quad, const Vector3& translate) const;

      // ---------------------------------------------------------------------------
      // Transforms ray into new space
      //-----------------------------------------------------------------------------
      ILine3D Transform(scalar scale, const Matrix3&  rotate, const Vector3& translate) const;

      // ---------------------------------------------------------------------------
      // Returns the distance squared between lines.
      //-----------------------------------------------------------------------------
      static scalar DistanceSquared( const ILine3D& line0, const ILine3D& line1,  scalar& s_c, scalar& t_c );

      static scalar Distance( const ILine3D& line0, const ILine3D& line1,  scalar& s_c, scalar& t_c );

      // ---------------------------------------------------------------------------
      // Returns the distance squared between line and point.
      //-----------------------------------------------------------------------------
      static scalar DistanceSquared( const ILine3D& line, const Vector3& point, scalar &t_c );
      static scalar Distance( const ILine3D& line, const Vector3& point,  scalar &t_c );


      //----------------------------------------------------------------------------
      // @ ClosestPoints()
      // ---------------------------------------------------------------------------
      // Returns the closest points between two lines
      //-----------------------------------------------------------------------------
      static void ClosestPoints( const ILine3D& line0,  const ILine3D& line1 , Vector3& point0, Vector3& point1 );


      // ---------------------------------------------------------------------------
      // Returns the closest point on line to point.
      //-----------------------------------------------------------------------------
      Vector3 ClosestPoint( const Vector3& point ) const;


#ifdef ENABLE_STL_SUPPORT

      //----------[ output operator ]----------------------------
      /**
      * Provides output to standard output stream.
      */
      friend std::ostream& operator <<(std::ostream& oss, const ILine3D& rhs)
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
#endif // ILINE3D_H
