
#include "ILine3D.h"

namespace IEngine
{

using namespace IMath;

ILine3D &ILine3D::operator=(const ILine3D &other)
{
    //         // if same object
    //         if ( this == &other )
    //          return *this;

    mOrigin = other.mOrigin;
    mDirection = other.mDirection;
    mDirection.Normalize();

    return *this;
}

void ILine3D::Get(Vector3 &origin, Vector3 &direction) const
{
    origin    = mOrigin;
    direction = mDirection;

}

void ILine3D::Set(const Vector3 &origin, const Vector3 &direction)
{
    mOrigin = origin;
    mDirection = direction;
    mDirection.Normalize();
}

bool ILine3D::operator==(const ILine3D &line) const
{
    return (line.mOrigin == mOrigin && line.mDirection == mDirection);
}

bool ILine3D::operator!=(const ILine3D &line) const
{
    return !(line.mOrigin == mOrigin && line.mDirection == mDirection);
}

ILine3D ILine3D::Transform(scalar scale, const Quaternion &quad, const Vector3 &translate) const
{
    ILine3D  line;
    Matrix3  rotate = quad.GetRotMatrix();

    line.mDirection = rotate * mDirection;
    line.mDirection *= scale;

    line.mOrigin =  rotate * mOrigin;
    line.mOrigin *= scale;
    line.mOrigin += translate;

    return line;
}

ILine3D ILine3D::Transform(scalar scale, const Matrix3 &rotate, const Vector3 &translate) const
{
    ILine3D line;

    line.mDirection  = rotate * mDirection;
    line.mDirection *= scale;

    line.mOrigin  = rotate * mOrigin;
    line.mOrigin *= scale;
    line.mOrigin += translate;

    return line;

}

scalar ILine3D::DistanceSquared(const ILine3D &line0, const ILine3D &line1, scalar &s_c, scalar &t_c)
{
    Vector3 w0 = line0.mOrigin - line1.mOrigin;
    scalar a = line0.mDirection.Dot( line0.mDirection );
    scalar b = line0.mDirection.Dot( line1.mDirection );
    scalar c = line1.mDirection.Dot( line1.mDirection );
    scalar d = line0.mDirection.Dot( w0 );
    scalar e = line1.mDirection.Dot( w0 );
    scalar denom = a*c - b*b;
    if ( IsZero(denom) )
    {
        s_c = 0.0f;
        t_c = e/c;
        Vector3 wc = w0 - t_c*line1.mDirection;
        return wc.Dot(wc);
    }
    else
    {
        s_c = ((b*e - c*d)/denom);
        t_c = ((a*e - b*d)/denom);
        Vector3 wc = w0 + s_c*line0.mDirection
                - t_c*line1.mDirection;
        return wc.Dot(wc);
    }
}

scalar ILine3D::Distance(const ILine3D &line0, const ILine3D &line1, scalar &s_c, scalar &t_c)
{
    return ISqrt( DistanceSquared( line0, line1, s_c, t_c ) );
}

scalar ILine3D::DistanceSquared(const ILine3D &line, const Vector3 &point, scalar &t_c)
{
    Vector3 w = point - line.mOrigin;
    scalar vsq = line.mDirection.Dot(line.mDirection);
    scalar proj = w.Dot(line.mDirection);
    t_c = proj/vsq;

    return w.Dot(w) - t_c*proj;
}

scalar ILine3D::Distance(const ILine3D &line, const Vector3 &point, scalar &t_c)
{
    return ISqrt( DistanceSquared( line, point, t_c ) );
}

void ILine3D::ClosestPoints(const ILine3D &line0, const ILine3D &line1, Vector3 &point0, Vector3 &point1)
{
    // compute intermediate parameters
    Vector3 w0 = line0.mOrigin - line1.mOrigin;
    scalar a = line0.mDirection.Dot( line0.mDirection );
    scalar b = line0.mDirection.Dot( line1.mDirection );
    scalar c = line1.mDirection.Dot( line1.mDirection );
    scalar d = line0.mDirection.Dot( w0 );
    scalar e = line1.mDirection.Dot( w0 );

    scalar denom = a*c - b*b;

    if ( IsZero(denom) )
    {
        point0 = line0.mOrigin;
        point1 = line1.mOrigin + (e/c)*line1.mDirection;
    }
    else
    {
        point0 = line0.mOrigin + ((b*e - c*d)/denom)*line0.mDirection;
        point1 = line1.mOrigin + ((a*e - b*d)/denom)*line1.mDirection;
    }
}

Vector3 ILine3D::ClosestPoint(const Vector3 &point) const
{
    Vector3 w = point - mOrigin;
    scalar vsq  = mDirection.Dot(mDirection);
    scalar proj = w.Dot(mDirection);

    return mOrigin + (proj/vsq)*mDirection;
}




}
