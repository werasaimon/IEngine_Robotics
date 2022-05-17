#ifndef VEHICLEROBOTCAR_H
#define VEHICLEROBOTCAR_H


#include "Engine/engine.hpp"
#include "Engine/imaths.hpp"
#include "EncoderSensor.h"
#include "OrientationSensor.h"

using namespace IEngine;
using namespace IMath;



class RobotSensors
{

    friend class VehicleRobotCar;

 public:

    RobotSensors(EncoderSensor *_a,
                 EncoderSensor *_b,
                 EncoderSensor *_c,
                 EncoderSensor *_d,
                 IRigidBody* _PhysicsBase);

   ~RobotSensors();


    void setPos_motorA(scalar newPos_motorA);
    void setPos_motorB(scalar newPos_motorB);
    void setPos_motorC(scalar newPos_motorC);
    void setPos_motorD(scalar newPos_motorD);


    scalar getPos_motorA() const;
    scalar getPos_motorB() const;
    scalar getPos_motorC() const;
    scalar getPos_motorD() const;

private:

    scalar pos_motorA;
    scalar pos_motorB;
    scalar pos_motorC;
    scalar pos_motorD;

    EncoderSensor *enc_a;
    EncoderSensor *enc_b;
    EncoderSensor *enc_c;
    EncoderSensor *enc_d;

    IOrientationSensor2 *mOrientationSensor;


};


struct DispatcherAttribute
{

    float coffPosDeverative0 = 4.f;
    float coffPosDeverative1 = 5.f;
    float coffPosDeverative2 = 5.f;

    float coffRotDev = 1.0f;
    float coffRotFix = 0.5f;
    float coffRotLen = 10.f;

    float coffEncoders = 0.05f;

    float extrem_MIN_MAX = 600;
    float minimal = 1.0;

    bool isCorrectlyDynamics = false;

};


struct DebugAnalysis
{
    float AngleYaw;
    float AngleYaw_Derivative;

    float dirivative_motor_a;
    float dirivative_motor_b;
    float dirivative_motor_c;
    float dirivative_motor_d;
};

class VehicleRobotCar
{

public:

    DispatcherAttribute mDispatcherAttribute;
    DebugAnalysis mDebugAnalisys;


//private:

    RobotSensors* mVehicleRobot;

    IRigidBody *physBody_Wheel_0;
    IRigidBody *physBody_Wheel_1;
    IRigidBody *physBody_Wheel_2;
    IRigidBody *physBody_Wheel_3;
    IRigidBody *physBody_Base;

    IHingeJoint *Wheel_HingeJoint0;
    IHingeJoint *Wheel_HingeJoint1;
    IHingeJoint *Wheel_HingeJoint2;
    IHingeJoint *Wheel_HingeJoint3;

    Quaternion mFixOrientation;


public:


  VehicleRobotCar(IRigidBody *base ,
                  IRigidBody* wheel0 ,
                  IRigidBody* wheel1 ,
                  IRigidBody* wheel2 ,
                  IRigidBody* wheel3 ,
                  IHingeJoint *_wheel_hinge_joint0,
                  IHingeJoint *_wheel_hinge_joint1,
                  IHingeJoint *_wheel_hinge_joint2,
                  IHingeJoint *_wheel_hinge_joint3):
        physBody_Wheel_0(wheel0),
        physBody_Wheel_1(wheel1),
        physBody_Wheel_2(wheel2),
        physBody_Wheel_3(wheel3),
        physBody_Base(base),
        Wheel_HingeJoint0(_wheel_hinge_joint0),
        Wheel_HingeJoint1(_wheel_hinge_joint1),
        Wheel_HingeJoint2(_wheel_hinge_joint2),
        Wheel_HingeJoint3(_wheel_hinge_joint3)
    {
        mVehicleRobot = new RobotSensors(new EncoderSensor(Wheel_HingeJoint0),
                                         new EncoderSensor(Wheel_HingeJoint1),
                                         new EncoderSensor(Wheel_HingeJoint2),
                                         new EncoderSensor(Wheel_HingeJoint3),
                                         base);

        mFixOrientation = Quaternion::IDENTITY;
    }

    ~VehicleRobotCar()
    {
        if(mVehicleRobot)
        {
            delete mVehicleRobot;
        }
    }


    void UpdateControlPointGuidance( const Vector3& target_position );
    void Update(scalar _dt, const Vector3 &target_position);
    void AddTransform(const Transform& t);
    void Stop();


};

#endif // VEHICLEROBOTCAR_H
