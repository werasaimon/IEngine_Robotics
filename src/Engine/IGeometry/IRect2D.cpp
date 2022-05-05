#include "IRect2D.h"


namespace IEngine
{

scalar Rect2D::height() const
{
    return mHeight;
}


Vector2 Rect2D::origin() const
{
    return mOrigin;
}


Vector2 Rect2D::GetMin() const
{
    return Vector2( mOrigin.x - mWidth  * scalar(0.5) ,
                    mOrigin.y - mHeight * scalar(0.5));
}


Vector2 Rect2D::GetMax() const
{
    return Vector2( mOrigin.x + mWidth  * scalar(0.5) ,
                    mOrigin.y + mHeight * scalar(0.5));
}


scalar Rect2D::width() const
{
    return mWidth;
}

}

