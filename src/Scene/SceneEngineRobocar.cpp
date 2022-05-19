#include "SceneEngineRobocar.h"
#include "IEngineFactory.h"

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"

#include <cmath>

//----------------------------------------------------------//

VehicleRobotCar *FactoryMethod::CretaeRobotCar()
{
    MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(25.0,1.0,12.0));
    IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
    BoxDown->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
    mScene->mComponents.push_back(BoxDown);

    Transform transform_4 = BoxDown->GetTransformMatrixHierarchy();
    transform_4.SetPosition(Vector3(0,0,0));
    IRigidBody* physBody_Target_4 = mScene->mDynamicsWorld->CreateRigidBody(transform_4);

    Transform trans = Transform::Identity();
    trans.SetPosition( trans.GetPosition() + trans.GetBasis() * Vector3::Y * 1.5f);
    IProxyShape* proxy_sphere_4 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(cuboid_dscp_down.size),1.f,trans);
    mScene->AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);


    float height = -1.5;

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_0(Vector3(4));
    IMesh *Mesh_Wheel_0 = new IMeshGenerate(sphere_dscp_wheel_0);
    IComponentMesh *Wheel_0 = new IComponentMesh(Mesh_Wheel_0);
    Wheel_0->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,10)));
    mScene->mComponents.push_back(Wheel_0);

    Transform transform_0 = Mesh_Wheel_0->GetTransformMatrixHierarchy();
    transform_0.SetPosition(Vector3(10,height,10) + physBody_Target_4->CenterOfMassWorld());
    IRigidBody* physBody_Wheel_0 = mScene->mDynamicsWorld->CreateRigidBody(transform_0);
    IProxyShape* proxy_sphere_0 = physBody_Wheel_0->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_0,proxy_sphere_0);
    physBody_Wheel_0->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_1(Vector3(4));
    IMesh *Mesh_Wheel_1 = new IMeshGenerate(sphere_dscp_wheel_1);
    IComponentMesh *Wheel_1 = new IComponentMesh(Mesh_Wheel_1);
    Wheel_1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,10)));
    mScene->mComponents.push_back(Wheel_1);

    Transform transform_1 = Mesh_Wheel_1->GetTransformMatrixHierarchy();
    transform_1.SetPosition(Vector3(-10,height,10) + physBody_Target_4->CenterOfMassWorld());
    IRigidBody* physBody_Wheel_1 = mScene->mDynamicsWorld->CreateRigidBody(transform_1);
    IProxyShape* proxy_sphere_1 = physBody_Wheel_1->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_1,proxy_sphere_1);
    physBody_Wheel_1->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_2(Vector3(4));
    IMesh *Mesh_Wheel_2 = new IMeshGenerate(sphere_dscp_wheel_2);
    IComponentMesh *Wheel_2 = new IComponentMesh(Mesh_Wheel_2);
    Wheel_2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,-10.0)));
    mScene->mComponents.push_back(Wheel_2);

    Transform transform_2 = Mesh_Wheel_2->GetTransformMatrixHierarchy();
    transform_2.SetPosition(Vector3(10,height,-10) + physBody_Target_4->CenterOfMassWorld());
    IRigidBody* physBody_Wheel_2 = mScene->mDynamicsWorld->CreateRigidBody(transform_2);
    IProxyShape* proxy_sphere_2 = physBody_Wheel_2->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_2,proxy_sphere_2);
    physBody_Wheel_2->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_3(Vector3(4));
    IMesh *Mesh_Wheel_3 = new IMeshGenerate(sphere_dscp_wheel_3);
    IComponentMesh *Wheel_3 = new IComponentMesh(Mesh_Wheel_3);
    Wheel_3->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,-10.0)));
    mScene->mComponents.push_back(Wheel_3);

    Transform transform_3 = Mesh_Wheel_3->GetTransformMatrixHierarchy();
    transform_3.SetPosition(Vector3(-10,height,-10)+ physBody_Target_4->CenterOfMassWorld());
    IRigidBody* physBody_Wheel_3 = mScene->mDynamicsWorld->CreateRigidBody(transform_3);
    IProxyShape* proxy_sphere_3 = physBody_Wheel_3->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_3,proxy_sphere_3);
    physBody_Wheel_3->SetType(BodyType::DYNAMIC);


    IPhysicsMaterial phys_material;
    phys_material.SetBounciness(0.0);
    phys_material.SetFrictionCoefficient(0.5f);
    phys_material.SetRollingResistance(0.01);


    //            IPhysicsMaterial phys_material2;
    //            phys_material2.SetBounciness(0.0);
    //            phys_material2.SetFrictionCoefficient(0.04f);
    //            phys_material2.SetRollingResistance(0.0);

    physBody_Wheel_0->SetMaterial(phys_material);
    physBody_Wheel_1->SetMaterial(phys_material);
    physBody_Wheel_2->SetMaterial(phys_material);
    physBody_Wheel_3->SetMaterial(phys_material);
    physBody_Target_4->SetMaterial(phys_material);


    mScene->mDynamicsWorld->addNoCollisionPair(physBody_Wheel_0,physBody_Target_4);
    mScene->mDynamicsWorld->addNoCollisionPair(physBody_Wheel_1,physBody_Target_4);
    mScene->mDynamicsWorld->addNoCollisionPair(physBody_Wheel_2,physBody_Target_4);
    mScene->mDynamicsWorld->addNoCollisionPair(physBody_Wheel_3,physBody_Target_4);

    //--------------------------------------//


    physBody_Target_4->SetCenterOfMassWorld( physBody_Target_4->CenterOfMassWorld() + Vector3::Y * height );


    IHingeJointInfo HingeJointInfoWheel0(physBody_Wheel_0,physBody_Target_4,transform_0.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel1(physBody_Wheel_1,physBody_Target_4,transform_1.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel2(physBody_Wheel_2,physBody_Target_4,transform_2.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel3(physBody_Wheel_3,physBody_Target_4,transform_3.GetPosition(),Vector3::Z);

    HingeJointInfoWheel0.isMotorEnabled = true;
    HingeJointInfoWheel1.isMotorEnabled = true;
    HingeJointInfoWheel2.isMotorEnabled = true;
    HingeJointInfoWheel3.isMotorEnabled = true;

    IHingeJoint* Wheel_HingeJoint0 = static_cast<IHingeJoint*>(mScene->mDynamicsWorld->CreateJoint(HingeJointInfoWheel0));
    IHingeJoint* Wheel_HingeJoint1 = static_cast<IHingeJoint*>(mScene->mDynamicsWorld->CreateJoint(HingeJointInfoWheel1));
    IHingeJoint* Wheel_HingeJoint2 = static_cast<IHingeJoint*>(mScene->mDynamicsWorld->CreateJoint(HingeJointInfoWheel2));
    IHingeJoint* Wheel_HingeJoint3 = static_cast<IHingeJoint*>(mScene->mDynamicsWorld->CreateJoint(HingeJointInfoWheel3));

    Wheel_HingeJoint0->EnableMotor(true);
    Wheel_HingeJoint1->EnableMotor(true);
    Wheel_HingeJoint2->EnableMotor(true);
    Wheel_HingeJoint3->EnableMotor(true);

    Wheel_HingeJoint0->SetMaxMotorTorque(500);
    Wheel_HingeJoint1->SetMaxMotorTorque(500);
    Wheel_HingeJoint2->SetMaxMotorTorque(500);
    Wheel_HingeJoint3->SetMaxMotorTorque(500);


    auto mRoboCar = new VehicleRobotCar(physBody_Target_4,
    physBody_Wheel_0,
    physBody_Wheel_1,
    physBody_Wheel_2,
    physBody_Wheel_3,
    Wheel_HingeJoint0,
    Wheel_HingeJoint1,
    Wheel_HingeJoint2,
    Wheel_HingeJoint3);

    return mRoboCar;
}

//----------------------------------------------------------//

SceneEngineRobocar::SceneEngineRobocar() :
    mFactoryMethod( new IEngineFactoryRobot(this) )
{

}

void SceneEngineRobocar::initCamera()
{
    mWidth  = 600;
    mHeight = 400;

    float aspect = mWidth / mHeight;
    float zNear  = 1.0;
    float zFar   = 250;
    float fov    = 30.0;

    IComponentCamera *camera = new IComponentCamera();

    camera->InitPerspective( fov , aspect , zNear , zFar );

    camera->SetEye(Vector3(0,0,-200));
    camera->SetCenter(Vector3(0,0,0));
    camera->SetUp(Vector3(0,1,0));
    camera->InitTransform();
}

void SceneEngineRobocar::initialization()
{
    glClearColor(0.f,0.f,0.0f,1.f);

    m_PointS = Vector3::X * 15.f;
    m_IsDynamic_LQR = false;

    //===========================//

    for( int i=0; i < 5; ++i )
    {
        float x = i * 30;
        float y = cos(i*2) * 20;
        float z =-13.5;
        mPoints.push_back( m_PointS + Vector3(x,z,y) - Vector3::Z * 20);
    }

    Vector3 end = mPoints[mPoints.size()-1];
    for( int i=0; i < 5; ++i )
    {
        float x = i * 30;
        float y = sin(i*2) * 20;
        float z =-13.5;
        mPoints.push_back( end + Vector3(y,0,x) );
    }

    m_EndPoint = mPoints[num=0];

            //===========================//

            NullAllKey();
            initCamera();

            //===========================//

            mCameraAngleYaw = 0;
            mCameraAnglePitch = 0;

            //===========================//

            mWidth  = 600;
            mHeight = 400;

            float aspect = mWidth / mHeight;
            float zNear  = 1.0;
            float zFar   = 100;
            float fov    = 30.0;

            mCamera = new IComponentCamera();
            mCamera->InitPerspective( fov , aspect , zNear , zFar );

            mCamera->SetEye(Vector3(0,0,mCameraZDistance=40));
            mCamera->SetCenter(Vector3(0,0,0));
            mCamera->SetUp(Vector3(0,1,0));

            //------------- Physics ----------------//

            mTimeStep = 1.0/60.f;
            mDynamicsWorld = new IDynamicsWorld(Vector3::Y * -20);


            mSceneDscriptor.m_IsSimulateDynamics = false;

            //--------------------------------------//

            mGizmoManipulator = std::auto_ptr<IGizmoManipulator>(new IGizmoManipulator());
            mGizmoManipulator->InitilizationDefault();
            mGizmoManipulator->Resize(mWidth,mHeight);
            mGizmoManipulator->DisplayScale(2.0);

            //--------------------------------------//

            mRoboCar = mFactoryMethod->CreateRobot(RobotDescriptor(Vector3(25.0,1.0,12.0),4));




            //--------------------------------------//


            //            //--------------------------------------//


            //            MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(25.0,1.0,12.0));
            //            IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
            //            BoxDown->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
            //            mComponents.push_back(BoxDown);

            //            Transform transform_4 = BoxDown->GetTransformMatrixHierarchy();
            //            transform_4.SetPosition(Vector3(0,0,0));
            //            IRigidBody* physBody_Target_4 = mDynamicsWorld->CreateRigidBody(transform_4);

            //            Transform trans = Transform::Identity();
            //            trans.SetPosition( trans.GetPosition() + trans.GetBasis() * Vector3::Y * 1.5f);
            //            IProxyShape* proxy_sphere_4 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(25.0,1.0,10.0)),1.f,trans);
            //            AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);


            //            float s = -1.5;

            //            //--------------------------------------//

            //            MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_0(Vector3(4));
            //            IMesh *Mesh_Wheel_0 = new IMeshGenerate(sphere_dscp_wheel_0);
            //            IComponentMesh *Wheel_0 = new IComponentMesh(Mesh_Wheel_0);
            //            Wheel_0->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,10)));
            //            mComponents.push_back(Wheel_0);

            //            Transform transform_0 = Mesh_Wheel_0->GetTransformMatrixHierarchy();
            //            transform_0.SetPosition(Vector3(10,s,10) + physBody_Target_4->CenterOfMassWorld());
            //            IRigidBody* physBody_Wheel_0 = mDynamicsWorld->CreateRigidBody(transform_0);
            //            IProxyShape* proxy_sphere_0 = physBody_Wheel_0->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
            //            AddPhysicsProxyInModel(Wheel_0,proxy_sphere_0);
            //            physBody_Wheel_0->SetType(BodyType::DYNAMIC);

            //            //--------------------------------------//

            //            MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_1(Vector3(4));
            //            IMesh *Mesh_Wheel_1 = new IMeshGenerate(sphere_dscp_wheel_1);
            //            IComponentMesh *Wheel_1 = new IComponentMesh(Mesh_Wheel_1);
            //            Wheel_1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,10)));
            //            mComponents.push_back(Wheel_1);

            //            Transform transform_1 = Mesh_Wheel_1->GetTransformMatrixHierarchy();
            //            transform_1.SetPosition(Vector3(-10,s,10) + physBody_Target_4->CenterOfMassWorld());
            //            IRigidBody* physBody_Wheel_1 = mDynamicsWorld->CreateRigidBody(transform_1);
            //            IProxyShape* proxy_sphere_1 = physBody_Wheel_1->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
            //            AddPhysicsProxyInModel(Wheel_1,proxy_sphere_1);
            //            physBody_Wheel_1->SetType(BodyType::DYNAMIC);

            //            //--------------------------------------//

            //            MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_2(Vector3(4));
            //            IMesh *Mesh_Wheel_2 = new IMeshGenerate(sphere_dscp_wheel_2);
            //            IComponentMesh *Wheel_2 = new IComponentMesh(Mesh_Wheel_2);
            //            Wheel_2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,-10.0)));
            //            mComponents.push_back(Wheel_2);

            //            Transform transform_2 = Mesh_Wheel_2->GetTransformMatrixHierarchy();
            //            transform_2.SetPosition(Vector3(10,s,-10) + physBody_Target_4->CenterOfMassWorld());
            //            IRigidBody* physBody_Wheel_2 = mDynamicsWorld->CreateRigidBody(transform_2);
            //            IProxyShape* proxy_sphere_2 = physBody_Wheel_2->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
            //            AddPhysicsProxyInModel(Wheel_2,proxy_sphere_2);
            //            physBody_Wheel_2->SetType(BodyType::DYNAMIC);

            //            //--------------------------------------//

            //            MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_3(Vector3(4));
            //            IMesh *Mesh_Wheel_3 = new IMeshGenerate(sphere_dscp_wheel_3);
            //            IComponentMesh *Wheel_3 = new IComponentMesh(Mesh_Wheel_3);
            //            Wheel_3->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,-10.0)));
            //            mComponents.push_back(Wheel_3);

            //            Transform transform_3 = Mesh_Wheel_3->GetTransformMatrixHierarchy();
            //            transform_3.SetPosition(Vector3(-10,s,-10)+ physBody_Target_4->CenterOfMassWorld());
            //            IRigidBody* physBody_Wheel_3 = mDynamicsWorld->CreateRigidBody(transform_3);
            //            IProxyShape* proxy_sphere_3 = physBody_Wheel_3->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
            //            AddPhysicsProxyInModel(Wheel_3,proxy_sphere_3);
            //            physBody_Wheel_3->SetType(BodyType::DYNAMIC);


            //            IPhysicsMaterial phys_material;
            //            phys_material.SetBounciness(0.0);
            //            phys_material.SetFrictionCoefficient(0.5f);
            //            phys_material.SetRollingResistance(0.01);


            //            //            IPhysicsMaterial phys_material2;
            //            //            phys_material2.SetBounciness(0.0);
            //            //            phys_material2.SetFrictionCoefficient(0.04f);
            //            //            phys_material2.SetRollingResistance(0.0);

            //            physBody_Wheel_0->SetMaterial(phys_material);
            //            physBody_Wheel_1->SetMaterial(phys_material);
            //            physBody_Wheel_2->SetMaterial(phys_material);
            //            physBody_Wheel_3->SetMaterial(phys_material);
            //            physBody_Target_4->SetMaterial(phys_material);


            //            mDynamicsWorld->addNoCollisionPair(physBody_Wheel_0,physBody_Target_4);
            //            mDynamicsWorld->addNoCollisionPair(physBody_Wheel_1,physBody_Target_4);
            //            mDynamicsWorld->addNoCollisionPair(physBody_Wheel_2,physBody_Target_4);
            //            mDynamicsWorld->addNoCollisionPair(physBody_Wheel_3,physBody_Target_4);

            //            //--------------------------------------//


            //            physBody_Target_4->SetCenterOfMassWorld( physBody_Target_4->CenterOfMassWorld() + Vector3::Y * s );

            ////            //IHingeJointInfo(physBody_Target_4,physBody_Wheel_0,transform_0.GetPosition(),Vector3::X);
            ////            //float s = 0;
            //            IHingeJointInfo HingeJointInfoWheel0(physBody_Wheel_0,physBody_Target_4,transform_0.GetPosition(),Vector3::Z);
            //            IHingeJointInfo HingeJointInfoWheel1(physBody_Wheel_1,physBody_Target_4,transform_1.GetPosition(),Vector3::Z);
            //            IHingeJointInfo HingeJointInfoWheel2(physBody_Wheel_2,physBody_Target_4,transform_2.GetPosition(),Vector3::Z);
            //            IHingeJointInfo HingeJointInfoWheel3(physBody_Wheel_3,physBody_Target_4,transform_3.GetPosition(),Vector3::Z);

            //            HingeJointInfoWheel0.isMotorEnabled = true;
            //            HingeJointInfoWheel1.isMotorEnabled = true;
            //            HingeJointInfoWheel2.isMotorEnabled = true;
            //            HingeJointInfoWheel3.isMotorEnabled = true;

            //            IHingeJoint* Wheel_HingeJoint0 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel0));
            //            IHingeJoint* Wheel_HingeJoint1 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel1));
            //            IHingeJoint* Wheel_HingeJoint2 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel2));
            //            IHingeJoint* Wheel_HingeJoint3 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel3));

            //            Wheel_HingeJoint0->EnableMotor(true);
            //            Wheel_HingeJoint1->EnableMotor(true);
            //            Wheel_HingeJoint2->EnableMotor(true);
            //            Wheel_HingeJoint3->EnableMotor(true);

            //            Wheel_HingeJoint0->SetMaxMotorTorque(500);
            //            Wheel_HingeJoint1->SetMaxMotorTorque(500);
            //            Wheel_HingeJoint2->SetMaxMotorTorque(500);
            //            Wheel_HingeJoint3->SetMaxMotorTorque(500);


            //            mRoboCar = new VehicleRobotCar(physBody_Target_4,
            //            physBody_Wheel_0,
            //            physBody_Wheel_1,
            //            physBody_Wheel_2,
            //            physBody_Wheel_3,
            //            Wheel_HingeJoint0,
            //            Wheel_HingeJoint1,
            //            Wheel_HingeJoint2,
            //            Wheel_HingeJoint3);

            ////            Transform t;
            ////            t.SetBasis(Matrix3::CreateRotationAxis(Vector3::Y,M_PI * 4.0));
            ////            t.SetPosition(Vector3::Y * 8 + m_EndPoint);
            ////            mRoboCar->AddTransform(t);

            //--------------------------------------//

            MeshGenerator::CuboidDescriptor cuboid_dscp(Vector3(500.0,5.0,500.0));
            IMesh *BoxMeshRoot = new IMeshGenerate(cuboid_dscp);
            IComponentMesh *BoxRoot = new IComponentMesh(BoxMeshRoot);
            BoxRoot->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,-15,0)));
            mComponents.push_back(BoxRoot);

            Transform transform = BoxMeshRoot->GetTransfom();
            transform.SetPosition(Vector3::Y * -20.f);
            IRigidBody *physBody = mDynamicsWorld->CreateRigidBody(transform);
            IProxyShape* proxy = physBody->AddCollisionShape(new ICollisionShapeBox(Vector3(500.0,5.0,500.0)),10.f);
            AddPhysicsProxyInModel(BoxRoot,proxy);
            physBody->SetType(BodyType::STATIC);

            //--------------------------------------//

            MeshGenerator::EllipsoidDescriptor cuboid_dscp_choice(Vector3(1.0,1.0,1.0));
            IMesh *BoxMeshChoice = new IMeshGenerate(cuboid_dscp_choice);
            IComponentMesh *BoxChoice = new IComponentMesh(BoxMeshChoice);
            NuzzleChoice = BoxChoice;
            BoxChoice->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,-10)));
            mComponents.push_back(BoxChoice);

            //-------------------------------------//


//            MeshGenerator::CuboidDescriptor cuboid_dscp_down_t(Vector3(5.0,5.0,5.0));
//            IComponentMesh *BoxDown_t = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down_t));
//            Transform transform_t = physBody_Target_4->GetTransform();
//            transform_t.SetPosition(Vector3(0,4,0));
//            BoxDown_t->SetTransform(transform_t);
//            mComponents.push_back(BoxDown_t);




            mGimbalStabilization = new IEngineGimbalStabilization();
            mComponents.push_back(mGimbalStabilization->mGimbalRoot);
            mComponents.push_back(mGimbalStabilization->mGimbalConnectA);
            mComponents.push_back(mGimbalStabilization->mGimbalConnectB);
            mComponents.push_back(mGimbalStabilization->mGimbalConnectC);


//            Transform transform_s = transform_t;
//           // transform_s.SetPosition(Vector3::Y * -20.f);
//            IRigidBody *physBody_s = mDynamicsWorld->CreateRigidBody(transform_s);
//            IProxyShape* proxy_s = physBody_s->AddCollisionShape(new ICollisionShapeBox(Vector3(5.0,5.0,5.0)),10.f);
//            AddPhysicsProxyInModel(mGimbalStabilization->mGimbalRoot,proxy_s);
//            physBody_s->SetType(BodyType::DYNAMIC);


//            trans = Transform::Identity();
//            trans.SetPosition( trans.GetPosition() + trans.GetBasis() * Vector3::Y * 4.5f);
//            IProxyShape* proxy_sphere_t = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(5.0,5.0,5.0)),1.f,trans);
//            AddPhysicsProxyInModel(BoxDown,proxy_sphere_t);

//            physBody_Target_4->SetType(BodyType::DYNAMIC);
            //physBody_Target_4->SetCenterOfMassWorld( trans.GetPosition() + Vector3::Y );



//            IFixedJointInfo FixedJointInfo(physBody_s,physBody_Target_4,physBody_s->CenterOfMassWorld() - Vector3::Y * -3.f);
//            IFixedJoint* FixedJoint0 = static_cast<IFixedJoint*>(mDynamicsWorld->CreateJoint(FixedJointInfo));



            //----------------------------------------------------------//


//            mGimbalStabilization = new IEngineGimbalStabilization();
//            mComponents.push_back(mGimbalStabilization->mGimbalRoot);
//            mComponents.push_back(mGimbalStabilization->mGimbalConnectA);
//            mComponents.push_back(mGimbalStabilization->mGimbalConnectB);
//            mComponents.push_back(mGimbalStabilization->mGimbalConnectC);

            //mGimbalStabilization->mGimbalRoot->SetTransform(BoxDown->GetTransfom());
            //BoxDown->AddChild(mGimbalStabilization->mGimbalRoot);

//            Transform transform_5 = physBody_Target_4->GetTransform();
//            transform_5.SetPosition( transform_5.GetPosition() + transform_5.GetBasis() * Vector3::Y * 3.f );
//            IProxyShape* proxy_sphere_5 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(5.0,5.0,5.0)),1.f,transform_5);
//            AddPhysicsProxyInModel(mGimbalStabilization->mGimbalRoot,proxy_sphere_5);

        //            MeshGenerator::CuboidDescriptor cuboid_dscp_down_t(Vector3(5.0,5.0,5.0));
        //            IComponentMesh *BoxDown_t = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down_t));
        //            Transform transform_t = physBody_Target_4->GetTransform();
        //            transform_4.SetPosition(Vector3(0,0,0));
        //            BoxDown_t->SetTransform(transform_t);
        //            mComponents.push_back(BoxDown_t);


        //            trans.SetPosition( trans.GetPosition() + trans.GetBasis() * Vector3::Y * 3.f);
        //            IProxyShape* proxy_sphere_t = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(5.0,5.0,5.0)),1.f,trans);
        //            AddPhysicsProxyInModel(BoxDown_t,proxy_sphere_t);

            //IProxyShape* proxy_sphere_5 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(5.0,5.0,5.0)),1.f);
            //AddPhysicsProxyInModel(BoxDown_t,proxy_sphere_5);



        //            physBody_Target_4->SetType(BodyType::DYNAMIC);
        //            physBody_Target_4->setIsMove(false);

            //--------------------------------------//


}

void SceneEngineRobocar::render(float FrameTime)
{

        glEnable(GL_SCISSOR_TEST);



    mCameraAnglePitch = IClamp(mCameraAnglePitch,scalar(-M_PI * 0.495),scalar(M_PI * 0.495));
    Matrix4 MRot;
    Vector3 ZBuffLength = Vector3::Z * mCameraZDistance;
    MRot = Matrix4::CreateRotationAxis( Vector3::X , -mCameraAnglePitch);
    MRot = Matrix4::CreateRotationAxis( Vector3::Y , -mCameraAngleYaw) * MRot;
    mCamera->SetEye(ZBuffLength * MRot);
    mCamera->BeginLookAt();

    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, mWidth, mHeight);
    glScissor(0,0,mWidth,mHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera->getCamera2().ViewMatrix());

   //=================================================//


//    if( !mGimbalStabilization->m_isExtremal )
//    {
//        glPushMatrix();
//        glLineWidth(5);
//        glColor3f(1,0,0);
//        glBegin(GL_LINES);
//        glVertex3fv(mGimbalStabilization->mGimbalConnectB->GetTransformHierarchy().GetPosition());
//        glVertex3fv(NuzzleChoice->GetTransformMatrix().GetTranslation());
//        glEnd();
//        glPopMatrix();
//    }


    glPushMatrix();
    glColor3f(1,0,0);
    GLUquadric *quad;
    quad = gluNewQuadric();
    glTranslatef(m_PointS.x,m_PointS.y=-13.5,m_PointS.z);
    gluSphere(quad,2,100,20);
    glPopMatrix();

//    {
//        glPushMatrix();
//        glColor3f(1,0,0);
//        GLUquadric *quad;
//        quad = gluNewQuadric();
//        Vector3 p = physBody_Target_4->GetTransform().GetPosition() +
//                    physBody_Target_4->GetTransform().GetRotation() * Vector3::X * 10;
//        glTranslatef(p.x,p.y,p.z);
//        gluSphere(quad,2,100,20);
//        glPopMatrix();
//    }


    for( const auto &it : mPoints )
    {
        glPushMatrix();
        glColor3f(1,0,0);
        GLUquadric *quad;
        quad = gluNewQuadric();
        glTranslatef(it.x,it.y,it.z);
        gluSphere(quad,0.4,100,20);
        glPopMatrix();
    }


    for( int i=0; i < (int)mPoints.size(); ++i )
    {
        glPushMatrix();
        glColor3f(0,1,0);
        glLineWidth(3);
        glBegin(GL_LINES);
        glVertex3fv(mPoints[i]);
        glVertex3fv(mPoints[((i+1)%mPoints.size() != 0)? i+1 : 0]);
        glEnd();
        glPopMatrix();
    }

//    glPushMatrix();
//    glColor3f(1,0,0);
//    glBegin(GL_LINES);
//    glVertex3fv(m_PointS);
//    glVertex3fv(p);
//    glEnd();
//    glPopMatrix();

    if(mSceneDscriptor.m_IsSimulateDynamics)
    {
        Transform t = mRoboCar->physBody_Base->GetTransform();
        t.SetPosition( t.GetPosition() + t.GetBasis() * Vector3::Y * 6.f);
        mGimbalStabilization->mGimbalRoot->SetTransform(t);
    }



    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {
         Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();

         //-------------------------------------------------------------//
         if(mSceneDscriptor.m_IsSimulateDynamics)
         {
             auto iter_proxy = mProxyColliderConnects.find((*it));
             if(iter_proxy != mProxyColliderConnects.end())
             {
                 MultMatrix = iter_proxy->second->GetWorldTransform().GetTransformMatrix();
             }
         }
         //-------------------------------------------------------------//

        glPushMatrix();
        glMultMatrixf(MultMatrix);
        glColor3f(0.5,0.5,0.5);
        OpenGLRender::DrawComponentMeshFill(static_cast<IComponentMesh*>(*it)->Modelmesh());
        glColor3f(1,1,1);
        OpenGLRender::DrawComponentMeshLine(static_cast<IComponentMesh*>(*it)->Modelmesh());
        glPopMatrix();
    }

    glLoadIdentity();

   //=================================================//


    //=================================================//

    //glCullFace(GL_BACK);
    glCullFace(GL_FRONT);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, mWidth, mHeight);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera->getCamera2().ViewMatrix());

    if (mGizmoManipulator->GetGizmo())
    {
        mGizmoManipulator->GetGizmo()->SetCameraMatrix( mCamera->getCamera2().ViewMatrix() ,
                                                        mCamera->getCamera2().ProjectionMatrix() );

        if(mGizmoManipulator->mSelectedIndexID >= 0)
        {
            glLineWidth(5);
            mGizmoManipulator->GetGizmo()->Draw();
            glLineWidth(1);
        }
    }

    glLoadIdentity();



   //=================================================//


  // glFlush();

//    glClearIndex(1);

    //-----------------------------//

//    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_LIGHTING);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//    glDisable(GL_CULL_FACE);



    glClearColor(0, 0, 0, 0);

   //-------------------------------//
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glEnable(GL_STENCIL_TEST);
    glViewport(0,mHeight - mHeight/4, mWidth/4,mHeight/4);
    glScissor(0,mHeight - mHeight/4,mWidth/4,mHeight/4);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //


    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);



    /**
    Vector3 Translation =
           Vector3(0,0,4) * mGimbalStabilization->mGimbalConnectC->GetTransformMatrixHierarchy();
    Quaternion RotationConj = Quaternion::LookAtRH( Translation ,
                                                    NuzzleChoice->GetTransformMatrix().GetTranslation() ,
                                                    Vector3::Y );


    Matrix4 TransformMatrix;
    TransformMatrix.SetToIdentity();
    TransformMatrix.Rotate(RotationConj.GetConjugate());
    TransformMatrix.Translate(-Translation);

    glLoadMatrixf(TransformMatrix);
    **/

    Transform T = mGimbalStabilization->mGimbalConnectC->GetTransformHierarchy().GetInverse();
    glLoadMatrixf(T.GetTransformMatrix());


    int i = 0;
    for(auto it=mComponents.begin(); it<mComponents.end(); ++it , ++i)
    {
        if(i == 10) continue;

        Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();

        //-------------------------------------------------------------//
        if(mSceneDscriptor.m_IsSimulateDynamics)
        {
            auto iter_proxy = mProxyColliderConnects.find((*it));
            if(iter_proxy != mProxyColliderConnects.end())
            {
                MultMatrix = iter_proxy->second->GetWorldTransform().GetTransformMatrix();
            }
        }
        //-------------------------------------------------------------//

        glPushMatrix();
        glMultMatrixf(MultMatrix);
        glColor3f(0.5,0.5,0.5);
        OpenGLRender::DrawComponentMeshFill(static_cast<IComponentMesh*>(*it)->Modelmesh());
        glColor3f(1,1,1);
        OpenGLRender::DrawComponentMeshLine(static_cast<IComponentMesh*>(*it)->Modelmesh());
        glPopMatrix();
    }

    glLoadIdentity();

    //=================================================//

    glFlush();
}

void SceneEngineRobocar::update()
{


    //------------------------------------------------------------------------------------------------------------------//

    if(mGizmoManipulator->GetGizmo() && mGizmoManipulator->isSelectedIndex())
    {
        if( !mGizmoManipulator->mSelectedIndexIDs.empty() && mGizmoManipulator->mSelectedIndexID >= 0 )
        {
            for( auto it = mGizmoManipulator->mSelectedIndexIDs.begin(); it != mGizmoManipulator->mSelectedIndexIDs.end(); ++it )
            {
                Vector3 worldPoint =mGizmoManipulator->mTransformInitilization.GetTranslation();
                Matrix4 m = Matrix4::CreateTranslation( worldPoint) * mGizmoManipulator->GetGizmo()->GetSHIFTMatrix() *
                            Matrix4::CreateTranslation(-worldPoint) * (it->second);

                mComponents[it->first]->SetTransform(m);
            }
        }
    }

//    if(mKeys[Qt::Key_U])
//    {
//        m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
//        if((m_EndPoint - m_PointS).LengthSquare() < 0.2 )
//        {
//            num++;
//            if( num%(int)mPoints.size() == 0) num = 0;
//            m_EndPoint = mPoints[num];
//        }
//    }


    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {
        (*it)->Updatee();
    }


    if(mRoboCar)
    {
      if(m_IsDynamic_LQR)
      {
        m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
        if((m_EndPoint - m_PointS).LengthSquare() < 40.2 )
        {
            num++;
            if( num%(int)mPoints.size() == 0 ) num = 0;
            m_EndPoint = mPoints[num];
        }

        mRoboCar->Update(mTimeStep,m_PointS);
        mRoboCar->UpdateControlPointGuidance(m_PointS);

      }
      else
      {
        mRoboCar->Stop();
      }
    }


    if(mSceneDscriptor.m_IsSimulateDynamics)
    {
        mDynamicsWorld->UpdateFixedTimeStep(mTimeStep);
    }


    //--------------------------------------------------------------//

    mGimbalStabilization->Update(mGimbalStabilization->mGimbalRoot->GetTransfom().GetRotation(),
                                 NuzzleChoice->GetTransformHierarchy().GetPosition(),
                                 mGimbalStabilization->mGimbalConnectB->GetTransformHierarchy().GetPosition());


     m_AngleGimbal = mGimbalStabilization->mAngleGimbalStabilization;

    //--------------------------------------------------------------//
}

void SceneEngineRobocar::resize(float width, float height)
{
    mWidth  = width;
    mHeight = height;

    float aspect = mWidth / mHeight;
    float zNear  = 1.0;
    float zFar   = 1000;
    float fov    = 30.0;

    mCamera->InitPerspective( fov , aspect , zNear , zFar );

    //-------------------------//

    if(mGizmoManipulator.get())
    {
        mGizmoManipulator->Resize( width , height );
    }
}

void SceneEngineRobocar::mouseMove(float x, float y, int button)
{
    if(button){}

    data_mouse.mouseX = x;
    data_mouse.mouseY = y;

    if( mMouseButton == Qt::MouseButton::MidButton )
    {
        float speedX = (data_mouse.mouseX - data_mouse.mouseOldX);
        float speedY = (data_mouse.mouseY - data_mouse.mouseOldY);

        float coff = 1.f / 1000.0;

        mCameraAngleYaw   -= speedX * coff;
        mCameraAnglePitch += speedY * coff;

        data_mouse.mouseOldX = data_mouse.mouseX;
        data_mouse.mouseOldY = data_mouse.mouseY;

    }
    else
    {
        if(  mMouseButton == Qt::MouseButton::LeftButton )
        {
            if (mGizmoManipulator.get())
            {
                mGizmoManipulator->GetGizmo()->OnMouseMove( data_mouse.mouseX,
                                                            data_mouse.mouseY);
            }
        }
    }
}

void SceneEngineRobocar::mousePress(float x, float y, int button)
{
    mMouseButton = button;
    data_mouse.mouseX = data_mouse.mouseOldX = x;
    data_mouse.mouseY = data_mouse.mouseOldY = y;

    if(  mMouseButton == Qt::MouseButton::LeftButton )
    {
        if (mGizmoManipulator->GetGizmo())
        {

            if (mGizmoManipulator->OnMouseDown( data_mouse.mouseX, data_mouse.mouseY ))
            {

            }
            else
            {

                mSelectedIndexIds.clear();

    //                //-----------------------------------------------------------------------------//
    //                mGizmoManipulator->realase();
    //                //-----------------------------------------------------------------------------//
    //                IRay casting_ray = mGizmoManipulator->BuildRayMouse( data_mouse.mouseX , data_mouse.mouseY );

    //                /**
    //                mMengmentCollider.raycast( casting_ray , mGizmoManipulator.get() , 0x111 );
    //                //-----------------------------------------------------------------------------//
    //                if(!mGizmoManipulator.get()->mSelectedShape) return;

    //                unsigned int selected_index = -1;
    //                for(unsigned int i=0; i<mComponents.size(); ++i)
    //                {
    //                     if(mComponents[i]->Collid() == mGizmoManipulator.get()->mSelectedShape->GetBody())
    //                     {
    //                         selected_index = i;
    //                     }
    //                }
    //                **/


    //               // if(!mGizmoManipulator.get()->mSelectedShape) return;

    //                int selected_index = -1;
    //                float min = 10000000;
    //                for(int i=0; i<mComponents.size(); ++i)
    //                {
    //                    IRaycast raycast;

    //                    std::cout << mComponents[i]->GetAxisAlignedBoundingBox() << std::endl;


    //                     if(mComponents[i]->IntersectionRaycast(casting_ray,&raycast))
    //                     {
    //                         std::cout << min << "  " << raycast.distance << std::endl;
    //                         if(min > raycast.distance)
    //                         {
    //                             min = raycast.distance;
    //                             selected_index = i;
    //                         }
    //                     }
    //                }

    //                std::cout << selected_index << std::endl;

    //                mGizmoManipulator->mSelectedIndexID = static_cast<int>(selected_index);

    //                if(selected_index >= static_cast<unsigned int>(0)) mSelectedIndexIds.insert( static_cast<int>(selected_index) );
    //                //-----------------------------------------------------------------------------//
    //                mGizmoManipulator->mSelectedIndexIDs.clear();
    //                for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
    //                {
    //                    mGizmoManipulator->mSelectedIndexIDs.insert( std::make_pair( *it , mComponents[*it]->GetTransformMatrixHierarchy()) );
    //                }
    //                //-----------------------------------------------------------------------------//

    //                Matrix4 Interpolation = Matrix4::ZERO;
    //                for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
    //                {
    //                    mGizmoManipulator->mSelectedIndexIDs[*it] = mComponents[*it]->GetTransformMatrixHierarchy();
    //                    Interpolation += mComponents[*it]->GetTransformMatrixHierarchy();
    //                }
    //                mGizmoManipulator->mTransformInitilization = Interpolation / float(mSelectedIndexIds.size());
    //                mGizmoManipulator->GetGizmo()->SetEditMatrix(mGizmoManipulator->mTransformInitilization);

                }
        }
    }


}

void SceneEngineRobocar::mouseReleasePress(float x, float y, int button)
{
    mMouseButton = button;
    data_mouse.mouseX = data_mouse.mouseOldX = x;
    data_mouse.mouseY = data_mouse.mouseOldY = y;


    if(  mMouseButton == Qt::MouseButton::LeftButton )
    {
         //================ Save transform geometry components  =================//
         if (mGizmoManipulator->GetGizmo())
         {
            mGizmoManipulator->GetGizmo()->IndentitySHIFTMatrix();
            if( !mSelectedIndexIds.empty() )
            {
                    mGizmoManipulator->GetGizmo()->OnMouseUp( data_mouse.mouseX , data_mouse.mouseY );

                    Matrix4 Interpolation = Matrix4::ZERO;
                    for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
                    {
                        mGizmoManipulator->mSelectedIndexIDs[*it] = mComponents[*it]->GetTransformMatrixHierarchy();
                        Interpolation += mComponents[*it]->GetTransformMatrixHierarchy();
                    }
                    mGizmoManipulator->mTransformInitilization = Interpolation / float(mSelectedIndexIds.size());
             }

             mGizmoManipulator->mSelectedIndexIDs.clear();
             for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
             {
                 mGizmoManipulator->mSelectedIndexIDs.insert( std::make_pair( *it , mComponents[*it]->GetTransformMatrixHierarchy()) );
             }
         }
     }

}

void SceneEngineRobocar::mouseWheel(float delta)
{
    mCameraZDistance += (delta * 0.02f);
    mCameraZDistance  = IMath::IClamp(IMath::IAbs(mCameraZDistance),5.f,1024.f);
}



//static float ang = 0;
void SceneEngineRobocar::keyboard(int key)
{
    if(key == Qt::Key_1)
    {

        qDebug() << "Key_T";


        if(!isSelectedStatus)
        {
            isSelectedStatus = true;
            int selected_index = 6;
            mSelectedIndexIds.clear();
            mGizmoManipulator->mSelectedIndexID = static_cast<int>(selected_index);

            if(selected_index >= static_cast<unsigned int>(0)) mSelectedIndexIds.insert( static_cast<int>(selected_index) );
            //-----------------------------------------------------------------------------//
            mGizmoManipulator->mSelectedIndexIDs.clear();
            for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
            {
                mGizmoManipulator->mSelectedIndexIDs.insert( std::make_pair( *it , mComponents[*it]->GetTransformMatrixHierarchy()) );
            }
            //-----------------------------------------------------------------------------//

            Matrix4 Interpolation = Matrix4::ZERO;
            for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
            {
                mGizmoManipulator->mSelectedIndexIDs[*it] = mComponents[*it]->GetTransformMatrixHierarchy();
                Interpolation += mComponents[*it]->GetTransformMatrixHierarchy();
            }
            mGizmoManipulator->mTransformInitilization = Interpolation / float(mSelectedIndexIds.size());
            mGizmoManipulator->GetGizmo()->SetEditMatrix(mGizmoManipulator->mTransformInitilization);
        }
        else
        {
            isSelectedStatus = false;
            mSelectedIndexIds.clear();

        }


    }


    if(key == Qt::Key_Z)
    {
        MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(2.0,2.0,2.0));
        IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
        Transform transform_4 = Transform::Identity();
        transform_4.SetPosition(Vector3(rand()%250,10,rand()%250));
        //BoxDown->SetTransform(transform_4);
        mComponents.push_back(BoxDown);


        IRigidBody* physBody_Targ = mDynamicsWorld->CreateRigidBody(transform_4);

        Transform trans = Transform::Identity();
        IProxyShape* proxy_sphere_4 = physBody_Targ->AddCollisionShape(new ICollisionShapeBox(Vector3(2.0,2.0,2.0)),1.f,trans);
        AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);
        physBody_Targ->SetType(BodyType::DYNAMIC);
        //physBody_Target_4->setIsMove(false);
    }

}

void SceneEngineRobocar::destroy()
{

}

//-----------------------------------------------------------//


