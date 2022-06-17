#ifndef IENGINEGIMBALSTABILIZATION_H
#define IENGINEGIMBALSTABILIZATION_H



#include "Sensors/OrientationSensor.h"
#include "Engine/engine.hpp"

#include "IGizmoManipulator.h"
#include "EngineComponent/IEngineComponent.hpp"
#include "Shader/Shader.h"


using namespace IMath;
using namespace IEngine;



class IEngineGimbalStabilization
{

//private:

public:

    OrientationSensor mSensor;

    IComponentMesh *mGimbalRoot;
    IComponentMesh *mGimbalConnectA;
    IComponentMesh *mGimbalConnectB;
    IComponentMesh *mGimbalConnectC;


    Vector3 mOriginGimbalRoot;
    Vector3 mOriginGimbalConnectA;
    Vector3 mOriginGimbalConnectB;
    Vector3 mOriginGimbalConnectC;

    Vector3 mOrigin;
    Vector3 mAngleGimbalStabilization;

    bool m_isExtremal;
    bool m_isRange;
    bool m_isStabilizationGimbal;



public:

    IEngineGimbalStabilization();

    void Update(const Quaternion &orintation, const Vector3 &position, const Vector3 &target);



    /**
    void Update(const Quaternion& orintation)
    {

        Quaternion QQL = Quaternion::IDENTITY;
        Quaternion QOrientation = orintation;
        mSensor.setQuat(QQL * QOrientation);
        mSensor.Update();

        mGimbalConnectA->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectA));
        mGimbalConnectA->RotateAroundWorldPoint(Vector3::Y, -mSensor.EulerAngle().y, mOriginGimbalConnectA);

        Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y, mSensor.EulerAngle().y);
        QI.Normalize();

        mSensor.setQuat(QQL * (QOrientation * QI.GetInverse()));
        mSensor.Update();

        mGimbalConnectB->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectB));
        mGimbalConnectB->RotateAroundWorldPoint(Vector3::X, -mSensor.EulerAngle().x, mOriginGimbalConnectB);


        mGimbalConnectC->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectC));
        mGimbalConnectC->RotateAroundWorldPoint(Vector3::Z, -mSensor.EulerAngle().z, mOriginGimbalConnectC);

    }


    void Update(const Quaternion& orintation , const Vector3& target)
    {
        Quaternion QQL = Quaternion::LookAtLH(mOrigin,
                                              target,
                                              Vector3::Y).GetConjugate();
        Quaternion QOrientation = orintation;
        mSensor.setQuat(QQL * QOrientation);
        mSensor.Update();

        mAngleGimbalStabilization.y = -mSensor.EulerAngle().y;
        mGimbalConnectA->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectA));
        mGimbalConnectA->RotateAroundWorldPoint(Vector3::Y, mAngleGimbalStabilization.y, mOriginGimbalConnectA);

        Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y, mSensor.EulerAngle().y);
        QI.Normalize();

        mSensor.setQuat(QQL * (QOrientation * QI.GetInverse()));
        mSensor.Update();

        mAngleGimbalStabilization.x = -mSensor.EulerAngle().x;
        mGimbalConnectB->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectB));
        mGimbalConnectB->RotateAroundWorldPoint(Vector3::X, mAngleGimbalStabilization.x, mOriginGimbalConnectB);


//        mAngleGimbalStabilization.z = -mSensor.EulerAngle().z;
//        mGimbalConnectC->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectC));
//        mGimbalConnectC->RotateAroundWorldPoint(Vector3::Z, -mSensor.EulerAngle().z, mOriginGimbalConnectC);


    }
    /**/





};



#endif // IENGINEGIMBALSTABILIZATION_H
