#include "IEngineFactory.h"
#include "Scene/SceneEngineRobocar.h"
#include "SceneEngine.h"
#include "Robotics/VehicleRobotCar.h"

IEngineFactory::IEngineFactory(SceneMain *_EngineScene) :
    mSceneEngin(_EngineScene)
{

}

VehicleRobotCar *IEngineFactoryRobot::CreateRobot(RobotDescriptor _descriptor , const Transform &transform)
{

    SceneEngineRobocar *mScene = static_cast<SceneEngineRobocar*>(mSceneEngin);


    MeshGenerator::CuboidDescriptor cuboid_dscp_down(_descriptor.mSize);//Vector3(25.0,1.0,12.0));
    IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
    BoxDown->SetTransformMatrix(Matrix4::CreateTranslation( transform * Vector3(0,0,0)));
    mScene->mComponents.push_back(BoxDown);

    //Transform transform_4;// = BoxDown->GetTransformMatrixHierarchy();
    //transform_4.SetPosition(transform * Vector3(0,0,0));
    IRigidBody* physBody_Target_4 = mScene->mDynamicsWorld->CreateRigidBody(transform);

    Transform trans = Transform::Identity();
    trans.SetPosition( trans.GetPosition() + trans.GetBasis() * Vector3::Y * 1.5f);
    IProxyShape* proxy_sphere_4 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(cuboid_dscp_down.size),1.f,trans);
    mScene->AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);


    float height = -1.5;

    //--------------------------------------//

    Vector3 Position_A = transform.GetBasis() * Vector3(10,height,10) + physBody_Target_4->CenterOfMassWorld();

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_0(Vector3(_descriptor.mRadiusWheels));
    IMesh *Mesh_Wheel_0 = new IMeshGenerate(sphere_dscp_wheel_0);
    IComponentMesh *Wheel_0 = new IComponentMesh(Mesh_Wheel_0);
    Wheel_0->SetTransformMatrix(Matrix4::CreateTranslation(Position_A));
    mScene->mComponents.push_back(Wheel_0);

    Transform transform_0 = Mesh_Wheel_0->GetTransformMatrixHierarchy();
    transform_0.SetPosition(Position_A);
    IRigidBody* physBody_Wheel_0 = mScene->mDynamicsWorld->CreateRigidBody(transform_0);
    IProxyShape* proxy_sphere_0 = physBody_Wheel_0->AddCollisionShape(new ICollisionShapeSphere(_descriptor.mRadiusWheels),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_0,proxy_sphere_0);
    physBody_Wheel_0->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    Vector3 Position_B = transform.GetBasis() * Vector3(-10,height,10) + physBody_Target_4->CenterOfMassWorld();

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_1(Vector3(_descriptor.mRadiusWheels));
    IMesh *Mesh_Wheel_1 = new IMeshGenerate(sphere_dscp_wheel_1);
    IComponentMesh *Wheel_1 = new IComponentMesh(Mesh_Wheel_1);
    Wheel_1->SetTransformMatrix(Matrix4::CreateTranslation(Position_B));
    mScene->mComponents.push_back(Wheel_1);

    Transform transform_1 = Mesh_Wheel_1->GetTransformMatrixHierarchy();
    transform_1.SetPosition(Position_B);
    IRigidBody* physBody_Wheel_1 = mScene->mDynamicsWorld->CreateRigidBody(transform_1);
    IProxyShape* proxy_sphere_1 = physBody_Wheel_1->AddCollisionShape(new ICollisionShapeSphere(_descriptor.mRadiusWheels),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_1,proxy_sphere_1);
    physBody_Wheel_1->SetType(BodyType::DYNAMIC);



    //--------------------------------------//

    Vector3 Position_C = transform.GetBasis() * Vector3(10,height,-10) + physBody_Target_4->CenterOfMassWorld();

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_2(Vector3(_descriptor.mRadiusWheels));
    IMesh *Mesh_Wheel_2 = new IMeshGenerate(sphere_dscp_wheel_2);
    IComponentMesh *Wheel_2 = new IComponentMesh(Mesh_Wheel_2);
    Wheel_2->SetTransformMatrix(Matrix4::CreateTranslation(Position_C));
    mScene->mComponents.push_back(Wheel_2);

    Transform transform_2 = Mesh_Wheel_2->GetTransformMatrixHierarchy();
    transform_2.SetPosition(Position_C);
    IRigidBody* physBody_Wheel_2 = mScene->mDynamicsWorld->CreateRigidBody(transform_2);
    IProxyShape* proxy_sphere_2 = physBody_Wheel_2->AddCollisionShape(new ICollisionShapeSphere(_descriptor.mRadiusWheels),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_2,proxy_sphere_2);
    physBody_Wheel_2->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    Vector3 Position_D = transform.GetBasis() * Vector3(-10,height,-10) + physBody_Target_4->CenterOfMassWorld();

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_3(Vector3(_descriptor.mRadiusWheels));
    IMesh *Mesh_Wheel_3 = new IMeshGenerate(sphere_dscp_wheel_3);
    IComponentMesh *Wheel_3 = new IComponentMesh(Mesh_Wheel_3);
    Wheel_3->SetTransformMatrix(Matrix4::CreateTranslation(Position_D));
    mScene->mComponents.push_back(Wheel_3);

    Transform transform_3 = Mesh_Wheel_3->GetTransformMatrixHierarchy();
    transform_3.SetPosition(Position_D);
    IRigidBody* physBody_Wheel_3 = mScene->mDynamicsWorld->CreateRigidBody(transform_3);
    IProxyShape* proxy_sphere_3 = physBody_Wheel_3->AddCollisionShape(new ICollisionShapeSphere(_descriptor.mRadiusWheels),1.f);
    mScene->AddPhysicsProxyInModel(Wheel_3,proxy_sphere_3);
    physBody_Wheel_3->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

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
