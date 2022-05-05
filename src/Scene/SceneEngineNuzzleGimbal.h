#ifndef SCENEENGINENUZZLEGIMBAL_H
#define SCENEENGINENUZZLEGIMBAL_H

#include "SceneMain.h"

#include "Scene/Sensors/OrientationSensor.h"
#include "Engine/engine.hpp"

#include "Engine/ICommon/IMemory/IList.h"
#include "Engine/ICommon/IMemory/IMem.h"
#include "Engine/ICommon/IMemory/IStack.h"
#include "Engine/IComponent/IComponent.h"
#include "IGizmoManipulator.h"
#include "EngineComponent/IEngineComponent.hpp"
#include "Shader/Shader.h"


using namespace IMath;
using namespace IEngine;



class EngineGimbalStabilization
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

    EngineGimbalStabilization()
    {

        m_isExtremal = false;
        m_isRange = false;
        m_isStabilizationGimbal = true;
        /**/

        mOrigin = Vector3(0,5,0);

        mOriginGimbalRoot = Vector3(0,0,0);
        mOriginGimbalConnectA = Vector3(0,3.3,0);
        mOriginGimbalConnectB = Vector3(0,5,0);
        mOriginGimbalConnectC = Vector3(0,0,-1.5);

        MeshGenerator::CuboidDescriptor cuboid_dscp2(Vector3(5.0,5.0,5.0));
        IMesh *BoxMesh2 = new IMeshGenerate(cuboid_dscp2);
        IComponentMesh *Box2 = new IComponentMesh(BoxMesh2);
        mGimbalRoot = Box2;
        Box2->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalRoot));


        //MeshGenerator::CuboidDescriptor cuboid_dscp3(Vector3(10.0,1.0,10.0));

        MeshGenerator::CylinderDescriptor cilinder_dscp(Vector2(5,5),1);
        IMesh *CilinderMesh3 = new IMeshGenerate(cilinder_dscp);
        IComponentMesh *Box3 = new IComponentMesh(CilinderMesh3);
        mGimbalConnectA = Box3;
        Box3->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectA));
        Box2->AddChild(Box3);


//        MeshGenerator::CuboidDescriptor cuboid_sub0(Vector3(1.0,4.5,1.0));
//        IMesh *BoxMeshSub0 = new IMeshGenerate(cuboid_sub0);
//        IComponentMesh *BoxSub0 = new IComponentMesh(BoxMeshSub0);
//        BoxSub0->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(4.5,2.8,0)));
//        Box3->AddChild(BoxSub0);


//        MeshGenerator::CuboidDescriptor cuboid_sub1(Vector3(1.0,4.5,1.0));
//        IMesh *BoxMeshSub1 = new IMeshGenerate(cuboid_sub1);
//        IComponentMesh *BoxSub1 = new IComponentMesh(BoxMeshSub1);
//        BoxSub1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-4.5,2.8,0)));
//        Box3->AddChild(BoxSub1);


        //============================================================//

        MeshGenerator::CuboidDescriptor cuboid_dscp4(Vector3(8.0,5.0,1.0));
        IMesh *BoxMesh4 = new IMeshGenerate(cuboid_dscp4);
        IComponentMesh *Box4 = new IComponentMesh(BoxMesh4);
        mGimbalConnectB = Box4;
        Box4->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectB));
        Box3->AddChild(Box4);


//        MeshGenerator::CuboidDescriptor cuboid_dscp_nuzzle(Vector3(3.0,3.0,3.0));
//        IMesh *BoxMeshNuzzle = new IMeshGenerate(cuboid_dscp_nuzzle);
//        IComponentMesh *BoxNuzzle = new IComponentMesh(BoxMeshNuzzle);
//        BoxNuzzle->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
//        mComponents.push_back(BoxNuzzle);
//        Box4->AddChild(BoxNuzzle);


        MeshGenerator::CuboidDescriptor cuboid_dscp5(Vector3(2.f,2.f,2.f));
        IMesh *BoxMesh5 = new IMeshGenerate(cuboid_dscp5);
        IComponentMesh *Box5 = new IComponentMesh(BoxMesh5);
        mGimbalConnectC = Box5;
        Box5->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectC));
        Box4->AddChild(Box5);


        /**/
    }



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


    void Update(const Quaternion& orintation , const Vector3& target , const Vector3& position)
    {

        m_isExtremal = true;

        if(!m_isStabilizationGimbal) return;


        Vector3 p = mGimbalConnectB->GetTransformHierarchy().GetPosition();
        Quaternion QQL = Quaternion::LookAtRH(p, target, Vector3::Y).GetConjugate();
        Quaternion QOrientation = orintation;

        mSensor.setQuat(QQL * QOrientation);
        mSensor.Update();


        Vector3 _Sensor;
        _Sensor.x = mSensor.EulerAngle().x;

        Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y, mSensor.EulerAngle().x);
        QI.Normalize();

        mSensor.setQuat(QQL * (QOrientation * QI.GetInverse()));
        mSensor.Update();

        _Sensor.y = mSensor.EulerAngle().y;
        _Sensor.z = mSensor.EulerAngle().z;


        if( (IMath::IAbs(_Sensor.x) < IMath::IDegreesToRadians(60.f) &&
             IMath::IAbs(_Sensor.y) < IMath::IDegreesToRadians(35.f)) || !m_isRange)
        {
            m_isExtremal = false;
            mAngleGimbalStabilization.y = -_Sensor.x;
//            mAngleGimbalStabilization.y = IMath::IClamp(  mAngleGimbalStabilization.y,
//                                                          IMath::IDegreesToRadians(-60.f + 0.01f),
//                                                          IMath::IDegreesToRadians( 60.f - 0.01f));

            mGimbalConnectA->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectA));
            mGimbalConnectA->RotateAroundWorldPoint(Vector3::Y, mAngleGimbalStabilization.y, mOriginGimbalConnectA);


            mAngleGimbalStabilization.x = -_Sensor.y;
//            mAngleGimbalStabilization.x = IMath::IClamp( mAngleGimbalStabilization.x,
//                                                         IMath::IDegreesToRadians(-35.f + 0.01f),
//                                                         IMath::IDegreesToRadians( 35.f - 0.01f));

            mGimbalConnectB->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectB));
            mGimbalConnectB->RotateAroundWorldPoint(Vector3::X, mAngleGimbalStabilization.x, mOriginGimbalConnectB);


//            mAngleGimbalStabilization.z = -_Sensor.z;
//            mGimbalConnectC->SetTransformMatrix(Matrix4::CreateTranslation(mOriginGimbalConnectC));
//            mGimbalConnectC->RotateAroundWorldPoint(Vector3::Z, mAngleGimbalStabilization.z, mOriginGimbalConnectC);
        }

    }


};



class SceneEngineNuzzleGimbal : public SceneMain
{
    //-----------------------------//
    struct Mouse
    {
        float mouseOldX;
        float mouseOldY;

        float mouseX;
        float mouseY;

    }  data_mouse;

    float mWidth;
    float mHeight;

    //-----------------------------//
    float mCameraZDistance;
    float mCameraAngleYaw;
    float mCameraAnglePitch;

    //-----------------------------//
    IComponentCamera *mCamera;

    //-----------------------------//
    OrientationSensor Sensor;

    //-----------------------------//
    std::vector<IComponentAbstract*>            mComponents;
    std::map<IComponentAbstract*, IProxyShape*> mProxyColliderConnects;

    float mTimeStep;
    IDynamicsWorld *mDynamicsWorld;

    //-----------------------------//

    Quaternion QQL;

    IComponentMesh *Nuzzle;
    IComponentMesh *NuzzleChild1;
    IComponentMesh *NuzzleChild2;
    IComponentMesh *NuzzleChild3;

    IComponentMesh *NuzzleChoice;
    IComponentMesh *EyeCube;

    //---------------- Manipulator -------------------//
    std::auto_ptr<IGizmoManipulator> mGizmoManipulator;
    std::set<int>                    mSelectedIndexIds;


    //---------------------------------------------//

    Vector2 AngleGimbal;
    EngineGimbalStabilization *mGimbalStabilization;

public:
    SceneEngineNuzzleGimbal();

    void initCamera();

    void initialization();
    void render(float FrameTime);
    void update();
    void resize( float width , float height );

    void mouseMove( float x , float y  , int button);
    void mousePress( float x , float y , int button );
    void mouseReleasePress( float x , float y , int button );
    void mouseWheel( float delta );

    void keyboard(int key );
    void destroy();

    IGizmoManipulator *gizmoManipulator() const;
    const OrientationSensor &getSensor() const;
    const Vector2 &getAngleGimbal() const;
    EngineGimbalStabilization *gimbalStabilization() const;
};
#endif // SCENEENGINENUZZLEGIMBAL_H
