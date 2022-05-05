#ifndef RECT_H
#define RECT_H

#include "../imaths.hpp"

namespace IEngine
{

using namespace IMath;

class Rect2D
{
   private:

      scalar mWidth;
      scalar mHeight;

      Vector2 mOrigin;

   public:

      //==================================//

      Rect2D( const scalar &_width ,
              const scalar &_height ,
              const Vector2 &_origin) :
        mWidth(_width) ,
        mHeight(_height) ,
        mOrigin(_origin)
      {}

      Rect2D( const Vector2 &_min ,
              const Vector2 &_max)
      {
          mWidth  = IAbs(_min.x - _max.x);
          mHeight = IAbs(_min.y - _max.y);
          mOrigin = (_max - _min) * scalar(0.5) + _min;
      }

      //==================================//

      Vector2 GetMin() const;
      Vector2 GetMax() const;

      //==================================//

      scalar width() const;
      scalar height() const;
      Vector2 origin() const;
};


}

#endif // RECT_H
