#ifndef ORIENTATIONSENSOR_H
#define ORIENTATIONSENSOR_H


#include "Engine/engine.hpp"
#include "Engine/imaths.hpp"


using namespace IEngine;
using namespace IMath;

class IOrientationSensor2
{
    IRigidBody *m_PhysicsBody;
    scalar      m_TimeStep;

public:

    IOrientationSensor2(IRigidBody *_PhysicsBody=nullptr , scalar _time_step=1.0)
    : m_PhysicsBody(_PhysicsBody) ,
      m_TimeStep(_time_step)
    {}

    Quaternion Quaternione() const
    {
       return  m_PhysicsBody->GetTransform().GetRotation();
    }

    Vector3 EulerAngle3( const Quaternion& desired_correction_value_tau = Quaternion::IDENTITY) const
    {
//       return  (m_PhysicsBody->GetTransform().GetRotation() *
//                desired_correction_value_tau).GetEulerAngles2();

//        Quaternion q1 = m_PhysicsBody->GetTransform().GetRotation();
//        Quaternion q2 = desired_correction_value_tau;

//        return q1.Lerp(0.5f , q2).GetEulerAngles2();

            return  (m_PhysicsBody->GetTransform().GetRotation() *
                     desired_correction_value_tau).GetEulerAngles2();

            //return m_PhysicsBody->GetTransform().GetRotation().Lerp(1.0f,desired_correction_value_tau).GetEulerAngles2();
    }

    Vector3 EulerAngle( const Quaternion& desired_correction_value_tau = Quaternion::IDENTITY) const
    {
       return  (m_PhysicsBody->GetTransform().GetRotation() *
                desired_correction_value_tau).GetEulerAngleGimbalLock(Quaternion::RotSeq::yxz);
    }

    Vector3 GyroscopeAngle( const Quaternion& desired_correction_value_tau = Quaternion::IDENTITY) const
    {
       return (m_PhysicsBody->GetTransform().GetRotation() *
               desired_correction_value_tau) *
               m_PhysicsBody->GetAngularVelocity() * m_TimeStep;
    }

    scalar BarometricAltitude() const
    {
       return m_PhysicsBody->GetTransform().GetPosition().Dot(Vector3::Y);
    }

    void setTimeStep(const scalar &TimeStep)
    {
        m_TimeStep = TimeStep;
    }
};


class IOrientationSensor
{
    IRigidBody *m_PhysicsBody;
    scalar      m_TimeStep;

public:

    IOrientationSensor(IRigidBody *_PhysicsBody=nullptr , scalar _time_step=1.0)
    : m_PhysicsBody(_PhysicsBody) ,
      m_TimeStep(_time_step)
    {
        assert(m_PhysicsBody);
    }


    Quaternion Rotation() const
    {
       return  m_PhysicsBody->GetTransform().GetRotation();
    }

    Vector3 AccelerometerAngle( const Quaternion& desired_correction_value_tau = Quaternion::IDENTITY) const
    {
       return  (m_PhysicsBody->GetTransform().GetRotation() *
                desired_correction_value_tau).GetEulerAngleGimbalLock();
    }

    Vector3 GyroscopeAngle( const Quaternion& desired_correction_value_tau = Quaternion::IDENTITY) const
    {
       return (m_PhysicsBody->GetTransform().GetRotation() *
               desired_correction_value_tau) *
               m_PhysicsBody->GetAngularVelocity() * m_TimeStep;
    }

    scalar BarometricAltitude() const
    {
       return m_PhysicsBody->GetTransform().GetPosition().Dot(Vector3::Y);
    }

    void setTimeStep(const scalar &TimeStep)
    {
        m_TimeStep = TimeStep;
    }
};

class OrientationSensor
{
    Quaternion m_Quat;
    Vector3 m_EulerAngle;


public:

    OrientationSensor()
    {

    }


    void setQuat(const Quaternion& _Quat)
    {
        m_Quat = _Quat;
    }

    void Update()
    {
        //m_EulerAngle = m_Quat.GetEulerAngles2();//
        m_EulerAngle = m_Quat.GetEulerAngleGimbalLock(Quaternion::RotSeq::yxz);
          //m_EulerAngle = m_Quat.GetEulerAngles();

//        if (m_EulerAngle.y >= M_PI)
//        {
//           m_EulerAngle.y -= M_PI * 2.f;
//        }
//        else if (m_EulerAngle.y < -M_PI)
//        {
//           m_EulerAngle.y += M_PI * 2.f;
//        }
    }


//    void Update2()
//    {
//        //m_EulerAngle = m_Quat.GetEulerAngles2();//
//        m_EulerAngle = m_Quat.GetEulerAngleGimbalLock(Quaternion::RotSeq::zyz);
//    }


    const Vector3 &EulerAngle() const
    {
        return m_EulerAngle;
    }
};

#endif // ORIENTATIONSENSOR_H
