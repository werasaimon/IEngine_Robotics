#include "ILineSegment3D.h"


namespace IEngine
{

using namespace IMath;

ILineSegment3D &ILineSegment3D::operator=(const ILineSegment3D &other)
{
    // if same object
    if ( this == &other )
        return *this;

    mOrigin = other.mOrigin;
    mDirection = other.mDirection;

    return *this;
}

void ILineSegment3D::Get(Vector3 &end0, Vector3 &end1) const
{
    end0 = mOrigin;
    end1 = mOrigin + mDirection;
}

void ILineSegment3D::Set(const Vector3 &end0, const Vector3 &end1)
{
    mOrigin = end0;
    mDirection = end1-end0;
}

scalar ILineSegment3D::Length() const
{
    return mDirection.Length();
}

scalar ILineSegment3D::LengthSquared() const
{
    return mDirection.LengthSquare();
}

bool ILineSegment3D::operator==(const ILineSegment3D &segment) const
{
    return ((segment.mOrigin == mOrigin && segment.mDirection == mDirection) ||
            (segment.mOrigin == mOrigin+mDirection && segment.mDirection == -mDirection));
}

bool ILineSegment3D::operator!=(const ILineSegment3D &segment) const
{
    return !((segment.mOrigin == mOrigin && segment.mDirection == mDirection) ||
             (segment.mOrigin == mOrigin+mDirection && segment.mDirection == -mDirection));
}

ILineSegment3D ILineSegment3D::Transform(scalar scale, const Quaternion &quat, const Vector3 &translate) const
{
    ILineSegment3D segment;
    Matrix3  rotate = quat.GetRotMatrix();

    segment.mDirection = rotate * mDirection;
    segment.mDirection *= scale;

    segment.mOrigin = rotate * mOrigin;
    segment.mOrigin *= scale;
    segment.mOrigin += translate;

    return segment;
}

ILineSegment3D ILineSegment3D::Transform(scalar scale, const Matrix3 &rotate, const Vector3 &translate) const
{
    ILineSegment3D segment;

    segment.mDirection = rotate * mDirection;
    segment.mDirection *= scale;

    segment.mOrigin = rotate * mOrigin;
    segment.mOrigin *= scale;
    segment.mOrigin += translate;

    return segment;
}

scalar ILineSegment3D::DistanceSquared(const ILineSegment3D &segment0, const ILineSegment3D &segment1, scalar &s_c, scalar &t_c)
{
    // compute intermediate parameters
    Vector3 w0 = segment0.mOrigin - segment1.mOrigin;
    scalar a = segment0.mDirection.Dot( segment0.mDirection );
    scalar b = segment0.mDirection.Dot( segment1.mDirection );
    scalar c = segment1.mDirection.Dot( segment1.mDirection );
    scalar d = segment0.mDirection.Dot( w0 );
    scalar e = segment1.mDirection.Dot( w0 );

    scalar denom = a*c - b*b;
    // parameters to compute s_c, t_c
    scalar sn, sd, tn, td;

    // if denom is zero, try finding closest point on segment1 to origin0
    if ( IsZero(denom) )
    {
        // clamp s_c to 0
        sd = td = c;
        sn = 0.0f;
        tn = e;
    }
    else
    {
        // clamp s_c within [0,1]
        sd = td = denom;
        sn = b*e - c*d;
        tn = a*e - b*d;

        // clamp s_c to 0
        if (sn < 0.0f)
        {
            sn = 0.0f;
            tn = e;
            td = c;
        }
        // clamp s_c to 1
        else if (sn > sd)
        {
            sn = sd;
            tn = e + b;
            td = c;
        }
    }

    // clamp t_c within [0,1]
    // clamp t_c to 0
    if (tn < 0.0f)
    {
        t_c = 0.0f;
        // clamp s_c to 0
        if ( -d < 0.0f )
        {
            s_c = 0.0f;
        }
        // clamp s_c to 1
        else if ( -d > a )
        {
            s_c = 1.0f;
        }
        else
        {
            s_c = -d/a;
        }
    }
    // clamp t_c to 1
    else if (tn > td)
    {
        t_c = 1.0f;
        // clamp s_c to 0
        if ( (-d+b) < 0.0f )
        {
            s_c = 0.0f;
        }
        // clamp s_c to 1
        else if ( (-d+b) > a )
        {
            s_c = 1.0f;
        }
        else
        {
            s_c = (-d+b)/a;
        }
    }
    else
    {
        t_c = tn/td;
        s_c = sn/sd;
    }

    // compute difference vector and distance squared
    Vector3 wc = w0 + s_c*segment0.mDirection - t_c*segment1.mDirection;
    return wc.Dot(wc);
}

scalar ILineSegment3D::DistanceSquared(const ILineSegment3D &segment, const ILine3D &line, scalar &s_c, scalar &t_c)
{
    // compute intermediate parameters
    Vector3 w0 = segment.mOrigin - line.GetOrigin();
    scalar a = segment.mDirection.Dot( segment.mDirection );
    scalar b = segment.mDirection.Dot( line.GetDirection() );
    scalar c = line.GetDirection().Dot( line.GetDirection() );
    scalar d = segment.mDirection.Dot( w0 );
    scalar e = line.GetDirection().Dot( w0 );

    scalar denom = a*c - b*b;

    // if denom is zero, try finding closest point on segment1 to origin0
    if ( IsZero(denom) )
    {
        s_c = 0.0f;
        t_c = e/c;
        // compute difference vector and distance squared
        Vector3 wc = w0 - t_c*line.GetDirection();
        return wc.Dot(wc);
    }
    else
    {
        // parameters to compute s_c, t_c
        scalar sn;

        // clamp s_c within [0,1]
        sn = b*e - c*d;

        // clamp s_c to 0
        if (sn < 0.0f)
        {
            s_c = 0.0f;
            t_c = e/c;
        }
        // clamp s_c to 1
        else if (sn > denom)
        {
            s_c = 1.0f;
            t_c = (e+b)/c;
        }
        else
        {
            s_c = sn/denom;
            t_c = (a*e - b*d)/denom;
        }

        // compute difference vector and distance squared
        Vector3 wc = w0 + s_c*segment.mDirection - t_c*line.GetDirection();
        return wc.Dot(wc);
    }
}

scalar ILineSegment3D::DistanceSquared(const ILine3D &line, const ILineSegment3D &segment, scalar &s_c, scalar &t_c)
{
    return DistanceSquared( segment, line, t_c, s_c );
}

scalar ILineSegment3D::DistanceSquared(const ILineSegment3D &segment, const Vector3 &point, scalar &t_c)
{
    Vector3 w = point - segment.mOrigin;
    scalar proj = w.Dot(segment.mDirection);
    // endpoint 0 is closest point
    if ( proj <= 0 )
    {
        t_c = 0.0f;
        return w.Dot(w);
    }
    else
    {
        scalar vsq = segment.mDirection.Dot(segment.mDirection);
        // endpoint 1 is closest point
        if ( proj >= vsq )
        {
            t_c = 1.0f;
            return w.Dot(w) - 2.0f*proj + vsq;
        }
        // otherwise somewhere else in segment
        else
        {
            t_c = proj/vsq;
            return w.Dot(w) - t_c*proj;
        }
    }
}

void ILineSegment3D::ClosestPoints(const ILineSegment3D &segment0, const ILineSegment3D &segment1, Vector3 &point0, Vector3 &point1)
{
    // compute intermediate parameters
    Vector3 w0 = segment0.mOrigin - segment1.mOrigin;
    scalar a = segment0.mDirection.Dot( segment0.mDirection );
    scalar b = segment0.mDirection.Dot( segment1.mDirection );
    scalar c = segment1.mDirection.Dot( segment1.mDirection );
    scalar d = segment0.mDirection.Dot( w0 );
    scalar e = segment1.mDirection.Dot( w0 );

    scalar denom = a*c - b*b;
    // parameters to compute s_c, t_c
    scalar s_c, t_c;
    scalar sn, sd, tn, td;

    // if denom is zero, try finding closest point on segment1 to origin0
    if ( IsZero(denom) )
    {
        // clamp s_c to 0
        sd = td = c;
        sn = 0.0f;
        tn = e;
    }
    else
    {
        // clamp s_c within [0,1]
        sd = td = denom;
        sn = b*e - c*d;
        tn = a*e - b*d;

        // clamp s_c to 0
        if (sn < 0.0f)
        {
            sn = 0.0f;
            tn = e;
            td = c;
        }
        // clamp s_c to 1
        else if (sn > sd)
        {
            sn = sd;
            tn = e + b;
            td = c;
        }
    }

    // clamp t_c within [0,1]
    // clamp t_c to 0
    if (tn < 0.0f)
    {
        t_c = 0.0f;
        // clamp s_c to 0
        if ( -d < 0.0f )
        {
            s_c = 0.0f;
        }
        // clamp s_c to 1
        else if ( -d > a )
        {
            s_c = 1.0f;
        }
        else
        {
            s_c = -d/a;
        }
    }
    // clamp t_c to 1
    else if (tn > td)
    {
        t_c = 1.0f;
        // clamp s_c to 0
        if ( (-d+b) < 0.0f )
        {
            s_c = 0.0f;
        }
        // clamp s_c to 1
        else if ( (-d+b) > a )
        {
            s_c = 1.0f;
        }
        else
        {
            s_c = (-d+b)/a;
        }
    }
    else
    {
        t_c = tn/td;
        s_c = sn/sd;
    }

    // compute closest points
    point0 = segment0.mOrigin + s_c*segment0.mDirection;
    point1 = segment1.mOrigin + t_c*segment1.mDirection;

}

void ILineSegment3D::ClosestPoints(const ILineSegment3D &segment, const ILine3D &line, Vector3 &point0, Vector3 &point1)
{

    // compute intermediate parameters
    Vector3 w0 = segment.mOrigin - line.GetOrigin();
    scalar a = segment.mDirection.Dot( segment.mDirection );
    scalar b = segment.mDirection.Dot( line.GetDirection() );
    scalar c = line.GetDirection().Dot( line.GetDirection() );
    scalar d = segment.mDirection.Dot( w0 );
    scalar e = line.GetDirection().Dot( w0 );

    scalar denom = a*c - b*b;

    // if denom is zero, try finding closest point on line to segment origin
    if ( IsZero(denom) )
    {
        // compute closest points
        point0 = segment.mOrigin;
        point1 = line.GetOrigin() + (e/c)*line.GetDirection();
    }
    else
    {
        // parameters to compute s_c, t_c
        scalar s_c, t_c;
        scalar sn;

        // clamp s_c within [0,1]
        sn = b*e - c*d;

        // clamp s_c to 0
        if (sn < 0.0f)
        {
            s_c = 0.0f;
            t_c = e/c;
        }
        // clamp s_c to 1
        else if (sn > denom)
        {
            s_c = 1.0f;
            t_c = (e+b)/c;
        }
        else
        {
            s_c = sn/denom;
            t_c = (a*e - b*d)/denom;
        }

        // compute closest points
        point0 = segment.mOrigin + s_c*segment.mDirection;
        point1 = line.GetOrigin() + t_c*line.GetDirection();
    }

}

Vector3 ILineSegment3D::ClosestPoint(const Vector3 &point) const
{
    Vector3 w = point - mOrigin;
    scalar proj = w.Dot(mDirection);
    // endpoint 0 is closest point
    if ( proj <= 0.0f )
        return mOrigin;
    else
    {
        scalar vsq = mDirection.Dot(mDirection);
        // endpoint 1 is closest point
        if ( proj >= vsq )
            return mOrigin + mDirection;
        // else somewhere else in segment
        else
            return mOrigin + (proj/vsq)*mDirection;
    }
}

bool ILineSegment3D::IsPointOnLine(const Vector3 p)
{
    return (p - GetEndpoint0()).Dot(p - GetEndpoint1()) <= 0;
}




}
