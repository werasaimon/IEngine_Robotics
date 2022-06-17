#include "IQCamera.h"


namespace IEngine
{



IQCamera::IQCamera()
    : mAngle(45.0),
      mAspect(1),
      mNear(1e-2),
      mFar(100),
      mAtDistance(1),
      mAngleOfViewSize(1.0),
      mOrthographic(false),
      mRotationConj(1,0,0,0),
      mTranslation(0,0,1)
{

}


IQCamera::~IQCamera()
{

}

//=====================================//


Quaternion IQCamera::Rotation() const
{
    return mRotationConj.GetConjugate();
}


Matrix4 IQCamera::ProjectionViewMatrix() const
{
    return ProjectionMatrix() * ViewMatrix();
}


void IQCamera::SetAngle(float angle)
{
    mAngle = angle;
}

void IQCamera::SetAspect(float aspect)
{
    mAspect = aspect;
}

void IQCamera::SetNear(float near)
{
    mNear = near;
}

void IQCamera::SetFar(float far)
{
    mFar = far;
}

void IQCamera::SetAtDistance(float atDist)
{
    mAtDistance = atDist;
}

scalar IQCamera::AngleOfViewSize() const
{
    return mAngleOfViewSize;
}

void IQCamera::SetOrthographic(bool orthographic)
{
    mOrthographic = orthographic;
}

//=====================================//


Matrix4 IQCamera::ProjectionMatrix() const
{

    Matrix4 P;
    using namespace std;
    const scalar far = mAtDistance + mFar;
    const scalar near = mNear;

    // http://stackoverflow.com/a/3738696/148668
    if(mOrthographic)
    {
        const scalar f = 0.5;
        const scalar left = -f*mAspect;
        const scalar right = f*mAspect;
        const scalar bottom = -f;
        const scalar top = f;
        const scalar tx = (right+left)/(right-left);
        const scalar ty = (top+bottom)/(top-bottom);
        const scalar tz = (far+near)/(far-near);
        const scalar z_fix = 0.5 /mAtDistance / tan(mAngle*0.5 * (M_PI/180.) );

        P = Matrix4
                (z_fix*2./(right-left), 0, 0, -tx,
                 0, z_fix*2./(top-bottom), 0, -ty,
                 0, 0, -z_fix*2./(far-near),  -tz,
                 0, 0, 0, 1);
        P = P.GetTranspose();

        P.SetToIdentity();
        P.GetData()[0] = z_fix*2./(right-left);
        P.GetData()[5] = z_fix*2./(top-bottom);
        P.GetData()[10] = -z_fix*2./(far-near);
        P.GetData()[12] = -tx;
        P.GetData()[13] = -ty;
        P.GetData()[14] = -tz;


        P = Matrix4::CreateOrtho(near , far , near , far , near , far , z_fix);
    }
    else
    {
        const scalar yScale = IMath::ITan(M_PI*0.5 - 0.5*mAngle*M_PI/180.);
        // http://stackoverflow.com/a/14975139/148668
        const scalar xScale = yScale/mAspect;
        P = Matrix4(xScale, 0, 0, 0,
                    0, yScale, 0, 0,
                    0, 0, -(far+near)/(far-near), -1,
                    0, 0, -2.*near*far/(far-near), 0);

      //  P = Matrix4::CreatePerspective( mAngle , mAspect , mNear , mFar );
    }

    return P;
}



void IQCamera::LookAt(const Vector3 &eye, const Vector3 &at, const Vector3 &up)
{

    mRotationConj = Quaternion::LookAtRH( eye , at , up ); //IQuaternion::LookAtLH(eye,at,up);
    mTranslation = /*mRotationConj.GetConjugate() **/ eye;

    mTransformMatrix.SetToIdentity();
    mTransformMatrix.Rotate(mRotationConj.GetConjugate());
    mTransformMatrix.Translate(-mTranslation);

}




Matrix4 IQCamera::ViewMatrix() const
{
    return mTransformMatrix;//(  Matrix4::CreateRotation(mRotationConj).GetInverse() * Matrix4::CreateTranslation(-mTranslation));
}



void IQCamera::DollyZoom(const scalar &fov)
{
    const scalar old_angle = mAngle;
    if(old_angle + fov < IGL_CAMERA_MIN_ANGLE)
    {
        mOrthographic = true;
    }
    else if(old_angle + fov > IGL_CAMERA_MIN_ANGLE)
    {
        mOrthographic = false;
    }

    if(!mOrthographic)
    {
        mAngle += fov;
        mAngle = IMath::IMin(scalar(89.),IMath::IMax(scalar(IGL_CAMERA_MIN_ANGLE),mAngle));
        // change in distance
        const scalar s = (2.*IMath::ITan(old_angle/2./180.*M_PI)) /
                         (2.*IMath::ITan(mAngle/2./180.*M_PI));

        mAngleOfViewSize = (4.*IMath::ITan(mAngle/2./180.*M_PI));
        const scalar old_at_dist = mAtDistance;
        mAtDistance = old_at_dist * s;
        Dolly(Vector3(0,0,1)*(mAtDistance - old_at_dist));
    }

}

void IQCamera::Orbit(const Quaternion &q , const Vector3& origin)
{

    LookAt(q*(Vector3::Z * mAtDistance) , origin , Vector3::Y );
}



Matrix4 IQCamera::GetAffine() const
{
    Matrix4 t = Matrix4::IDENTITY;
    t =     Matrix4::CreateRotation(mRotationConj).GetInverse();
    t = t * Matrix4::CreateTranslation(-mTranslation);
    return t;
}

Matrix4 IQCamera::GetInverse() const
{
    Matrix4 t = Matrix4::IDENTITY;
    t =     Matrix4::CreateTranslation(-mTranslation);
    t = t * Matrix4::CreateRotation(mRotationConj);
    return t;
}



Vector3 IQCamera::GetEye() const
{
    return -(Vector3(0,0,0) * GetInverse());
}

Vector3 IQCamera::GetAt() const
{
    return -(Vector3(0,0,-1)*mAtDistance) * GetInverse();
}

Vector3 IQCamera::GetUp() const
{
    return Matrix4::CreateRotation(mRotationConj.GetConjugate()) * Vector3(0,1,0);
}

Vector3 IQCamera::UnitPlane() const
{
    // Distance of center pixel to eye
    const scalar d = 1.0;
    const scalar a = mAspect;
    const scalar theta = mAngle*M_PI/180.;
    const scalar w =2.*IMath::ISqrt(-d*d/(a*a*IMath::IPow(tan(0.5*theta),2.)-1.))*a*IMath::ITan(0.5*theta);
    const scalar h = w/a;
    return Vector3(w*0.5,h*0.5,-d);
}

void IQCamera::Dolly(const Vector3 &dv)
{
    mTranslation += dv;
}


//===================================//


void IQCamera::GetFrustumPlanes(Vector4 *FrustumPlanes, bool inverse)
{

    Matrix4 MView = (inverse)? ViewMatrix() : ViewMatrix().GetTranspose();
    Matrix4 viewProj =  ProjectionMatrix() * MView;

    scalar invert = (inverse) ? -1.0 : 1.0;

    // Left Frustum Plane
    // Add first column of the matrix to the fourth column
    FrustumPlanes[0].x = invert * (-viewProj[0][3] + viewProj[0][0]);
    FrustumPlanes[0].y = invert * (-viewProj[1][3] + viewProj[1][0]);
    FrustumPlanes[0].z = invert * (-viewProj[2][3] + viewProj[2][0]);
    FrustumPlanes[0].w = invert * (-viewProj[3][3] + viewProj[3][0]);

    // Right Frustum Plane
    // Subtract first column of matrix from the fourth column
    FrustumPlanes[1].x = invert * (-viewProj[0][3] - viewProj[0][0]);
    FrustumPlanes[1].y = invert * (-viewProj[1][3] - viewProj[1][0]);
    FrustumPlanes[1].z = invert * (-viewProj[2][3] - viewProj[2][0]);
    FrustumPlanes[1].w = invert * (-viewProj[3][3] - viewProj[3][0]);

    // Top Frustum Plane
    // Subtract second column of matrix from the fourth column
    FrustumPlanes[2].x = invert * (-viewProj[0][3] - viewProj[0][1]);
    FrustumPlanes[2].y = invert * (-viewProj[1][3] - viewProj[1][1]);
    FrustumPlanes[2].z = invert * (-viewProj[2][3] - viewProj[2][1]);
    FrustumPlanes[2].w = invert * (-viewProj[3][3] - viewProj[3][1]);

    // Bottom Frustum Plane
    // Add second column of the matrix to the fourth column
    FrustumPlanes[3].x = invert * (-viewProj[0][3] + viewProj[0][1]);
    FrustumPlanes[3].y = invert * (-viewProj[1][3] + viewProj[1][1]);
    FrustumPlanes[3].z = invert * (-viewProj[2][3] + viewProj[2][1]);
    FrustumPlanes[3].w = invert * (-viewProj[3][3] + viewProj[3][1]);

    // Near Frustum Plane
    // We could add the third column to the fourth column to get the near plane,
    // but we don't have to do this because the third column IS the near plane
    FrustumPlanes[4].x = (viewProj[0][3]);
    FrustumPlanes[4].y = (viewProj[1][3]);
    FrustumPlanes[4].z = (viewProj[2][3]);
    FrustumPlanes[4].w = (viewProj[3][3]) - mNear;

    //std::cout << "GELP " << viewProj[3][2] << std::endl;

    // Far Frustum Plane
    // Subtract third column of matrix from the fourth column
    FrustumPlanes[5].x = (viewProj[0][3] - viewProj[0][2]);
    FrustumPlanes[5].y = (viewProj[1][3] - viewProj[1][2]);
    FrustumPlanes[5].z = (viewProj[2][3] - viewProj[2][2]);
    FrustumPlanes[5].w = (viewProj[3][3] - viewProj[3][2]);



    // Normalize plane normals (A, B and C (xyz))
    // Also take note that planes face inward
    FrustumPlanes[0] /= FrustumPlanes[0].GetXYZ().Length();
    FrustumPlanes[1] /= FrustumPlanes[1].GetXYZ().Length();
    FrustumPlanes[2] /= FrustumPlanes[2].GetXYZ().Length();
    FrustumPlanes[3] /= FrustumPlanes[3].GetXYZ().Length();
    FrustumPlanes[4] /= FrustumPlanes[4].GetXYZ().Length();
    FrustumPlanes[5] /= FrustumPlanes[5].GetXYZ().Length();

}



std::vector<Vector4> IQCamera::FrustumPlanes()
{
    // x, y, z, and w represent A, B, C and D in the plane equation
    // where ABC are the xyz of the planes normal, and D is the plane constant
    std::vector<Vector4> tempFrustumPlane(6);


    Matrix4 MView = ViewMatrix();
    Matrix4 viewProj =  ProjectionMatrix() * MView.GetTranspose();

    // Left Frustum Plane
    // Add first column of the matrix to the fourth column
    tempFrustumPlane[0].x = -(-viewProj[0][3] + viewProj[0][0]);
    tempFrustumPlane[0].y = -(-viewProj[1][3] + viewProj[1][0]);
    tempFrustumPlane[0].z = -(-viewProj[2][3] + viewProj[2][0]);
    tempFrustumPlane[0].w = -(-viewProj[3][3] + viewProj[3][0]);

    // Right Frustum Plane
    // Subtract first column of matrix from the fourth column
    tempFrustumPlane[1].x = -(-viewProj[0][3] - viewProj[0][0]);
    tempFrustumPlane[1].y = -(-viewProj[1][3] - viewProj[1][0]);
    tempFrustumPlane[1].z = -(-viewProj[2][3] - viewProj[2][0]);
    tempFrustumPlane[1].w = -(-viewProj[3][3] - viewProj[3][0]);

    // Top Frustum Plane
    // Subtract second column of matrix from the fourth column
    tempFrustumPlane[2].x = -(-viewProj[0][3] - viewProj[0][1]);
    tempFrustumPlane[2].y = -(-viewProj[1][3] - viewProj[1][1]);
    tempFrustumPlane[2].z = -(-viewProj[2][3] - viewProj[2][1]);
    tempFrustumPlane[2].w = -(-viewProj[3][3] - viewProj[3][1]);

    // Bottom Frustum Plane
    // Add second column of the matrix to the fourth column
    tempFrustumPlane[3].x = -(-viewProj[0][3] + viewProj[0][1]);
    tempFrustumPlane[3].y = -(-viewProj[1][3] + viewProj[1][1]);
    tempFrustumPlane[3].z = -(-viewProj[2][3] + viewProj[2][1]);
    tempFrustumPlane[3].w = -(-viewProj[3][3] + viewProj[3][1]);


    tempFrustumPlane[0].Normalize();
    tempFrustumPlane[1].Normalize();
    tempFrustumPlane[2].Normalize();
    tempFrustumPlane[3].Normalize();


    Vector3 LookDir = (tempFrustumPlane[0].GetXYZ() +
            tempFrustumPlane[1].GetXYZ() +
            tempFrustumPlane[2].GetXYZ() +
            tempFrustumPlane[3].GetXYZ()).Normalized();

    // Near Frustum Plane
    // We could add the third column to the fourth column to get the near plane,
    // but we don't have to do this because the third column IS the near plane
    tempFrustumPlane[4].x = LookDir.x;
    tempFrustumPlane[4].y = LookDir.y;
    tempFrustumPlane[4].z = LookDir.z;
    tempFrustumPlane[4].w = mNear;// -ViewProj[3][2] / 2.f;


    // Far Frustum Plane
    // Subtract third column of matrix from the fourth column
    tempFrustumPlane[5].x = -LookDir.x;
    tempFrustumPlane[5].y = -LookDir.y;
    tempFrustumPlane[5].z = -LookDir.z;
    tempFrustumPlane[5].w = mFar;//ViewProj[2][2];



    // Normalize plane normals (A, B and C (xyz))
    // Also take note that planes face inward
    tempFrustumPlane[4].Normalize();
    tempFrustumPlane[5].Normalize();


    return tempFrustumPlane;
}

std::vector<Vector3> IQCamera::GetFurstumPoints(scalar zNear, scalar zFar)
{

    std::vector<Vector4> planes = FrustumPlanes();

    Vector3 position_eye = ViewMatrix().GetTranslation();

    IPlane plane[6];
    plane[0] = IPlane(planes[0].GetXYZ(),position_eye); // Left Frustum Plane
    plane[1] = IPlane(planes[1].GetXYZ(),position_eye); // Right Frustum Plane
    plane[2] = IPlane(planes[2].GetXYZ(),position_eye); // Bottom Frustum Plane
    plane[3] = IPlane(planes[3].GetXYZ(),position_eye); // Top Frustum Plane

    // Direction look axis
    Vector3 LookDirection =  (planes[0].GetXYZ() +
            planes[1].GetXYZ() +
            planes[2].GetXYZ() +
            planes[3].GetXYZ()).Normalized();


    Vector3 dir0 = planes[3].ClosestPoint(planes[0].ClosestPoint(LookDirection));
    Vector3 dir1 = planes[3].ClosestPoint(planes[1].ClosestPoint(LookDirection));
    Vector3 dir2 = planes[2].ClosestPoint(planes[0].ClosestPoint(LookDirection));
    Vector3 dir3 = planes[2].ClosestPoint(planes[1].ClosestPoint(LookDirection));


    std::vector<Vector3> tempFrustumPoints(8);

    scalar e0 = LookDirection.Dot(dir0);
    scalar e1 = LookDirection.Dot(dir1);
    scalar e2 = LookDirection.Dot(dir2);
    scalar e3 = LookDirection.Dot(dir3);



    tempFrustumPoints[0] = ( position_eye + dir0 * zNear/e0 );
    tempFrustumPoints[1] = ( position_eye + dir1 * zNear/e1 );
    tempFrustumPoints[2] = ( position_eye + dir2 * zNear/e2 );
    tempFrustumPoints[3] = ( position_eye + dir3 * zNear/e3 );


    tempFrustumPoints[4] = ( position_eye + dir0 * zFar/e0 );
    tempFrustumPoints[5] = ( position_eye + dir1 * zFar/e1 );
    tempFrustumPoints[6] = ( position_eye + dir2 * zFar/e2 );
    tempFrustumPoints[7] = ( position_eye + dir3 * zFar/e3 );

    return tempFrustumPoints;

}

std::vector<ILineSegment3D> IQCamera::GetFrustumLines(scalar zNear, scalar zFar)
{
    std::vector<Vector3> points = GetFurstumPoints(zNear,zFar);
    std::vector<ILineSegment3D> lines(12);

    lines[0] = ILineSegment3D( points[0] , points[1] );
    lines[1] = ILineSegment3D( points[1] , points[3] );
    lines[2] = ILineSegment3D( points[2] , points[3] );
    lines[3] = ILineSegment3D( points[2] , points[0] );


    lines[4] = ILineSegment3D( points[4] , points[5] );
    lines[5] = ILineSegment3D( points[5] , points[7] );
    lines[6] = ILineSegment3D( points[6] , points[7] );
    lines[7] = ILineSegment3D( points[6] , points[4] );


    lines[8] = ILineSegment3D( points[0] , points[4] );
    lines[9] = ILineSegment3D( points[1] , points[5] );
    lines[10] = ILineSegment3D( points[2] , points[6] );
    lines[11] = ILineSegment3D( points[3] , points[7] );

    return lines;
}

std::vector<Vector3> IQCamera::GetFurstumPoints()
{
    return GetFurstumPoints(mNear,mFar);
}

std::vector<ILineSegment3D> IQCamera::GetFrustumLines()
{
    return GetFrustumLines(mNear, mFar);
}


bool IQCamera::FrustumCullingAABB(const Vector3 &min, const Vector3 &max)
{
    Vector4 FrustumPlane[6];
    GetFrustumPlanes(FrustumPlane);
    return IntersectionCullingAABB(FrustumPlane,min,max);

}


bool IQCamera::IntersectionCullingAABB(Vector4 *frustumPlanes, const Vector3 &aabb_min, const Vector3 &aabb_max)
{
    bool cull = true;
    // Loop through each frustum plane
    for(int planeID = 0; planeID < 6; ++planeID)
    {
        Vector3 planeNormal   = frustumPlanes[planeID].GetXYZ();
        scalar  planeConstant = frustumPlanes[planeID].w;

        // Check each axis (x,y,z) to get the AABB vertex furthest away from the direction the plane is facing (plane normal)
        Vector3 axisVert;

        if(frustumPlanes[planeID].x < 0.0f)    // Which AABB vertex is furthest down (plane normals direction) the x axis
            axisVert.x = aabb_min.x; // min x plus tree positions x
        else
            axisVert.x = aabb_max.x; // max x plus tree positions x

        // y-axis
        if(frustumPlanes[planeID].y < 0.0f)    // Which AABB vertex is furthest down (plane normals direction) the y axis
            axisVert.y = aabb_min.y; // min y plus tree positions y
        else
            axisVert.y = aabb_max.y; // max y plus tree positions y

        // z-axis
        if(frustumPlanes[planeID].z < 0.0f)    // Which AABB vertex is furthest down (plane normals direction) the z axis
            axisVert.z = aabb_min.z; // min z plus tree positions z
        else
            axisVert.z = aabb_max.z; // max z plus tree positions z

        if(planeNormal.Dot(axisVert) + planeConstant < 0)
        {
            cull = false;
            break;
        }
    }

    return  cull;
}


}
