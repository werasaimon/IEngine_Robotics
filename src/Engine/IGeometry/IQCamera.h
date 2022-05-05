#ifndef IQCAMERA_H
#define IQCAMERA_H

#include "../imaths.hpp"
#include "Segments/ILineSegment3D.h"
#include "Segments/IPlane.h"


#define IGL_CAMERA_MIN_ANGLE 5.0

namespace IEngine
{

class IQCamera : public AffineTransform
{

public:
        // On windows you might need: -fno-delayed-template-parsing
        //static constexpr float IGL_CAMERA_MIN_ANGLE = 5.;
        //  m_angle  Field of view angle in degrees {45}
        //  m_aspect  Aspect ratio {1}
        //  m_near  near clipping plane {1e-2}
        //  m_far  far clipping plane {100}
        //  m_at_dist  distance of looking at point {1}
        //  m_orthographic  whether to use othrographic projection {false}
        //  m_rotation_conj  Conjugate of rotation part of rigid transformation of
        //    camera {identity}. Note: we purposefully store the conjugate because
        //    this is what TW_TYPE_QUAT4D is expecting.
        //  m_translation  Translation part of rigid transformation of camera
        //    {(0,0,1)}
        scalar mAngle;
        scalar mAspect;
        scalar mNear;
        scalar mFar;
        scalar mAtDistance;


 private:

        scalar mAngleOfViewSize;
        bool   mOrthographic;


        Quaternion mRotationConj;
        Vector3    mTranslation;


    public:

       IQCamera();
       virtual ~IQCamera();


       // Return projection matrix that takes relative camera coordinates and
       // transforms it to viewport coordinates
       //
       // Note:
       //
       //     if(m_angle > 0)
       //     {
       //       gluPerspective(m_angle,m_aspect,m_near,m_at_dist+m_far);
       //     }else
       //     {
       //       gluOrtho(-0.5*aspect,0.5*aspect,-0.5,0.5,m_at_dist+m_near,m_far);
       //     }
       //
       // Is equivalent to
       //
       //     glMultMatrixd(projection().data());
       //
       Matrix4 ProjectionMatrix() const;


       // Transform Matrix4x4 o Camera
       Matrix4 ViewMatrix() const;



       /// \brief Rotation
       /// \return Orientation camera
       Quaternion Rotation() const;


       ///
       /// \brief ProjectionViewMatrix
       /// \return Projection camera
       Matrix4 ProjectionViewMatrix() const;


       // Rotate and translate so that camera is situated at "eye" looking at "at"
       // with "up" pointing up.
       //
       // Inputs:
       //   eye  (x,y,z) coordinates of eye position
       //   at   (x,y,z) coordinates of at position
       //   up   (x,y,z) coordinates of up vector
       void LookAt(const Vector3& eye, const Vector3& at, const Vector3& up);




       // Return an Affine transformation (rigid actually) that
       // takes relative coordinates and tramsforms them into world 3d
       // coordinates: moves the camera into the scene.
       Matrix4 GetAffine() const;

       // Return an Affine transformation (rigid actually) that puts the takes a
       // world 3d coordinate and transforms it into the relative camera
       // coordinates: moves the scene in front of the camera.
       //
       // Note:
       //
       //     gluLookAt(
       //       eye()(0), eye()(1), eye()(2),
       //       at()(0), at()(1), at()(2),
       //       up()(0), up()(1), up()(2));
       //
       // Is equivalent to
       //
       //     glMultMatrixd(camera.inverse().matrix().data());
       //
       // See also: affine, eye, at, up
       Matrix4 GetInverse() const;

       // Returns world coordinates position of center or "eye" of camera.
       Vector3 GetEye() const;

       // Returns world coordinate position of a point "eye" is looking at.
       Vector3 GetAt() const;

       // Returns world coordinate unit vector of "up" vector
       Vector3 GetUp() const;

       // Return top right corner of unit plane in relative coordinates, that is
       // (w/2,h/2,1)
       Vector3 UnitPlane() const;

       // Move dv in the relative coordinate frame of the camera (move the FPS)
       //
       // Inputs:
       //   dv  (x,y,z) displacement vector
       //
       void Dolly(const Vector3 &dv);
       // "Scale zoom": Move `eye`, but leave `at`



       // Aka "Hitchcock", "Vertigo", "Spielberg" or "Trombone" zoom:
       // simultaneously dolly while changing angle so that `at` not only stays
       // put in relative coordinates but also projected coordinates. That is
       //
       // Inputs:
       //   da  change in angle in degrees
       void DollyZoom(const scalar &da);


       // Orbit around at so that rotation is now q
       //
       // Inputs:
       //   q  new rotation as quaternion
       void Orbit(const Quaternion &q , const Vector3& origin = Vector3::ZERO);





       //----------------------  Frustum Planes -------------------------//

       void GetFrustumPlanes(Vector4 *FrustumPlanes, bool inverse=true);
       std::vector<Vector4> FrustumPlanes();
       std::vector<Vector3> GetFurstumPoints( scalar zNear , scalar zFar );
       std::vector<ILineSegment3D> GetFrustumLines(scalar zNear, scalar zFar);
       std::vector<Vector3> GetFurstumPoints();
       std::vector<ILineSegment3D> GetFrustumLines();

       bool FrustumCullingAABB(const Vector3 &min, const Vector3 &max);




       void SetAngle(float angle);
       void SetAspect(float aspect);
       void SetNear(float near);
       void SetFar(float far);
       void SetAtDistance(float atDist);
       void SetOrthographic(bool orthographic);

       ///
       /// \brief AngleOfViewSize
       /// \return camera angle of view
       scalar AngleOfViewSize() const;



private:

       static bool IntersectionCullingAABB(Vector4 *frustumPlanes , const Vector3 &aabb_min, const Vector3 &aabb_max );

};

}

#endif // IQCAMERA_H
