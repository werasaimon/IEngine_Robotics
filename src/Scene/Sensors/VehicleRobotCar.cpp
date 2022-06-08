#include "VehicleRobotCar.h"


RobotSensors::RobotSensors(EncoderSensor *_a,
                           EncoderSensor *_b,
                           EncoderSensor *_c,
                           EncoderSensor *_d,
                           IRigidBody *_PhysicsBase) :
    enc_a(_a),
    enc_b(_b),
    enc_c(_c),
    enc_d(_d)
{
    mOrientationSensor = new IOrientationSensor2(_PhysicsBase);
    pos_motorA = 0;
    pos_motorB = 0;
    pos_motorC = 0;
    pos_motorD = 0;
}

RobotSensors::~RobotSensors()
{
  if(mOrientationSensor)
  {
      delete mOrientationSensor;
  }

  if(enc_a) delete enc_a;
  if(enc_b) delete enc_b;
  if(enc_c) delete enc_c;
  if(enc_d) delete enc_d;
}


void RobotSensors::setPos_motorA(scalar newPos_motorA)
{
    pos_motorA = newPos_motorA;
}

void RobotSensors::setPos_motorB(scalar newPos_motorB)
{
    pos_motorB = newPos_motorB;
}

void RobotSensors::setPos_motorC(scalar newPos_motorC)
{
    pos_motorC = newPos_motorC;
}

void RobotSensors::setPos_motorD(scalar newPos_motorD)
{
    pos_motorD = newPos_motorD;
}


scalar RobotSensors::getPos_motorA() const
{
    return pos_motorA;
}

scalar RobotSensors::getPos_motorB() const
{
    return pos_motorB;
}

scalar RobotSensors::getPos_motorC() const
{
    return pos_motorC;
}

scalar RobotSensors::getPos_motorD() const
{
    return pos_motorD;
}


//-----------------------------------------------------------------------------//


void VehicleRobotCar::UpdateControlPointGuidance(const Vector3 &target_position)
{
    Vector3 p = physBody_Base->GetTransform().GetPosition();
    Vector3 direction = Vector3::Clamp(p - target_position , 25);

//    if(IMath::IAbs(IMath::Cross(p,target_position).LengthSquare()) < 0.0001 )
//    {
//        direction = Vector3::X;
//    }

    Vector3 eye_dir = physBody_Base->GetTransform().GetRotation() * Vector3::X;
    Vector3 vel = physBody_Base->GetLinearVelocity();

    scalar L = direction.Dot(eye_dir);
    scalar V = vel.Dot(eye_dir);


    const static float MIN_MAX = mDispatcherAttribute.extrem_MIN_MAX;
    L = IMath::IClamp(L,-MIN_MAX,MIN_MAX);

    float coff0 = mDispatcherAttribute.coffPosDeverative0;
    float coff1 = mDispatcherAttribute.coffPosDeverative1;
    float coff2 = mDispatcherAttribute.coffPosDeverative2;
    mVehicleRobot->setPos_motorA((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorB((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorC((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorD((L * coff1 + V * coff2) * coff0);

    if(direction.Length() > /*mDispatcherAttribute.minimal*/ 1 )
    {
        Vector3 dir = (target_position - p).Normalized();

        mFixOrientation = (Quaternion::SlerpToVector(Vector3::X ,dir).GetConjugate());
        if(IMath::IAbs(IMath::Cross(Vector3::X ,dir).LengthSquare()) < 0.0001 )
        {
          mFixOrientation = Quaternion::IDENTITY;
        }


        //        mRigidBodyVehicle->ApplyImpulseLinear( mOrientationSensor->Quaternione() * Vector3::X * -0.025f *
        //                                               Vector3::Clamp(direction,1.0));
    }
}

void VehicleRobotCar::UpdateControlPointGuidance2(const Vector3 &target_position)
{
    Vector3 p = physBody_Base->GetTransform().GetPosition();
    Vector3 direction = (p - target_position);

//    if(IMath::IAbs(IMath::Cross(p,target_position).LengthSquare()) < 0.0001 )
//    {
//        direction = Vector3::X;
//    }

    Vector3 eye_dir = physBody_Base->GetTransform().GetRotation() * Vector3::X;
    Vector3 vel = physBody_Base->GetLinearVelocity();

    scalar L = direction.Dot(eye_dir);
    scalar V = vel.Dot(eye_dir);


    const static float MIN_MAX = mDispatcherAttribute.extrem_MIN_MAX;
    L = IMath::IClamp(L,-MIN_MAX,MIN_MAX);

    float coff0 = mDispatcherAttribute.coffPosDeverative0;
    float coff1 = mDispatcherAttribute.coffPosDeverative1;
    float coff2 = mDispatcherAttribute.coffPosDeverative2;
    mVehicleRobot->setPos_motorA((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorB((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorC((L * coff1 + V * coff2) * coff0);
    mVehicleRobot->setPos_motorD((L * coff1 + V * coff2) * coff0);

}

void VehicleRobotCar::Update(scalar _dt , const Vector3 &target_position)
{
    if(mVehicleRobot)
    {

        //--------------------------------------------//

        auto encoder_a = mVehicleRobot->enc_a;
        auto encoder_b = mVehicleRobot->enc_b;
        auto encoder_c = mVehicleRobot->enc_c;
        auto encoder_d = mVehicleRobot->enc_d;

        encoder_a->Update(_dt);
        encoder_b->Update(_dt);
        encoder_c->Update(_dt);
        encoder_d->Update(_dt);

        float dist_pos_a = (encoder_a->getPos() - mVehicleRobot->getPos_motorA());
        float dirivative_a = dist_pos_a * _dt - encoder_a->getVel() * mDispatcherAttribute.coffEncoders;

        float dist_pos_b = (encoder_b->getPos() - mVehicleRobot->getPos_motorB());
        float dirivative_b = dist_pos_b * _dt - encoder_b->getVel() * mDispatcherAttribute.coffEncoders;

        float dist_pos_c = (encoder_c->getPos() - mVehicleRobot->getPos_motorC());
        float dirivative_c = dist_pos_c * _dt - encoder_c->getVel() * mDispatcherAttribute.coffEncoders;

        float dist_pos_d = (encoder_d->getPos() - mVehicleRobot->getPos_motorD());
        float dirivative_d = dist_pos_d * _dt - encoder_d->getVel() * mDispatcherAttribute.coffEncoders;

        //--------------------------------------------//

        auto  OrientationSensor = mVehicleRobot->mOrientationSensor;

        Vector3 euler = OrientationSensor->EulerAngle3(mFixOrientation);
        auto velocity_angle = physBody_Base->GetAngularVelocity();

        //------------------------------------------------------------------//
        mDebugAnalisys.AngleYaw = euler.y *  mDispatcherAttribute.coffRotDev;
        mDebugAnalisys.AngleYaw_Derivative = (OrientationSensor->Quaternione() * velocity_angle).y * mDispatcherAttribute.coffRotDev;
        //------------------------------------------------------------------//

        Vector3 p = physBody_Base->GetTransform().GetPosition();
        Vector3 direction = (p - target_position);

        if(direction.Length() > mDispatcherAttribute.minimal )
        {
            float yaw = euler.y *  mDispatcherAttribute.coffRotDev +
                    (OrientationSensor->Quaternione() * velocity_angle).y * mDispatcherAttribute.coffRotDev;

            float coff = mDispatcherAttribute.coffRotLen;
            dirivative_a += -yaw * coff;
            dirivative_b += -yaw * coff;
            dirivative_c += +yaw * coff;
            dirivative_d += +yaw * coff;
        }



        //--------------------------------------------//

        Wheel_HingeJoint0->SetMotorSpeed(-dirivative_a);
        Wheel_HingeJoint1->SetMotorSpeed(-dirivative_b);
        Wheel_HingeJoint2->SetMotorSpeed(-dirivative_c);
        Wheel_HingeJoint3->SetMotorSpeed(-dirivative_d);


        //------------------------------------------------------------------//
        mDebugAnalisys.dirivative_motor_a = dirivative_a;
        mDebugAnalisys.dirivative_motor_b = dirivative_b;
        mDebugAnalisys.dirivative_motor_c = dirivative_c;
        mDebugAnalisys.dirivative_motor_d = dirivative_d;
        //------------------------------------------------------------------//




        //------------------------------------------------------------------//

        if(mDispatcherAttribute.isCorrectlyDynamics == true)
        {
            const static float MIN_MAX = 180.f * 2.f;
            float ya = IMath::IClamp(euler.y , -MIN_MAX , MIN_MAX);
            float yaw2 = ya * 0.08 + (OrientationSensor->Quaternione() * velocity_angle).y * 0.03f;
            physBody_Base->ApplyImpulseAngular( Vector3::Y * yaw2 * -10.25f *
                                                physBody_Base->GetInertiaTensorWorld());
        }
        //------------------------------------------------------------------//
    }
}

void VehicleRobotCar::AddTransform(const Transform &t)
{

    Transform t0 = physBody_Base->GetTransform() * t;
    Transform t1 = physBody_Wheel_0->GetTransform() * t;
    Transform t2 = physBody_Wheel_1->GetTransform() * t;
    Transform t3 = physBody_Wheel_2->GetTransform() * t;
    Transform t4 = physBody_Wheel_3->GetTransform() * t;

    physBody_Base->SetTransform(t0);
    physBody_Wheel_0->SetTransform(t1);
    physBody_Wheel_1->SetTransform(t2);
    physBody_Wheel_2->SetTransform(t3);
    physBody_Wheel_3->SetTransform(t4);

    physBody_Base->SetCenterOfMassWorld(t0.GetPosition());
    physBody_Wheel_0->SetCenterOfMassWorld(t1.GetPosition());
    physBody_Wheel_1->SetCenterOfMassWorld(t2.GetPosition());
    physBody_Wheel_2->SetCenterOfMassWorld(t3.GetPosition());
    physBody_Wheel_3->SetCenterOfMassWorld(t4.GetPosition());
}

void VehicleRobotCar::Stop()
{
    Wheel_HingeJoint0->SetMotorSpeed(0);
    Wheel_HingeJoint1->SetMotorSpeed(0);
    Wheel_HingeJoint2->SetMotorSpeed(0);
    Wheel_HingeJoint3->SetMotorSpeed(0);
}
