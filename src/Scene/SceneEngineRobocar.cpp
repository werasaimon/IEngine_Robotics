#include "SceneEngineRobocar.h"

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"

#include <cmath>


SceneEngineRobocar::SceneEngineRobocar()
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

    MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(25.0,1.0,12.0));
    IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
    BoxDown->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
    mComponents.push_back(BoxDown);

    Transform transform_4 = BoxDown->GetTransformMatrixHierarchy();
    transform_4.SetPosition(Vector3(0,0,0));
    IRigidBody* physBody_Target_4 = mDynamicsWorld->CreateRigidBody(transform_4);
    IProxyShape* proxy_sphere_4 = physBody_Target_4->AddCollisionShape(new ICollisionShapeBox(Vector3(25.0,1.0,10.0)),1.f);
    AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);
    physBody_Target_4->SetType(BodyType::DYNAMIC);
    physBody_Target_4->setIsMove(false);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_0(Vector3(4));
    IMesh *Mesh_Wheel_0 = new IMeshGenerate(sphere_dscp_wheel_0);
    IComponentMesh *Wheel_0 = new IComponentMesh(Mesh_Wheel_0);
    Wheel_0->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,10)));
    mComponents.push_back(Wheel_0);

    Transform transform_0 = Mesh_Wheel_0->GetTransformMatrixHierarchy();
    transform_0.SetPosition(Vector3(10,0,10));
    IRigidBody* physBody_Wheel_0 = mDynamicsWorld->CreateRigidBody(transform_0);
    IProxyShape* proxy_sphere_0 = physBody_Wheel_0->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    AddPhysicsProxyInModel(Wheel_0,proxy_sphere_0);
    physBody_Wheel_0->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_1(Vector3(4));
    IMesh *Mesh_Wheel_1 = new IMeshGenerate(sphere_dscp_wheel_1);
    IComponentMesh *Wheel_1 = new IComponentMesh(Mesh_Wheel_1);
    Wheel_1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,10)));
    mComponents.push_back(Wheel_1);

    Transform transform_1 = Mesh_Wheel_1->GetTransformMatrixHierarchy();
    transform_1.SetPosition(Vector3(-10,0,10));
    IRigidBody* physBody_Wheel_1 = mDynamicsWorld->CreateRigidBody(transform_1);
    IProxyShape* proxy_sphere_1 = physBody_Wheel_1->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    AddPhysicsProxyInModel(Wheel_1,proxy_sphere_1);
    physBody_Wheel_1->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_2(Vector3(4));
    IMesh *Mesh_Wheel_2 = new IMeshGenerate(sphere_dscp_wheel_2);
    IComponentMesh *Wheel_2 = new IComponentMesh(Mesh_Wheel_2);
    Wheel_2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(10,0,-10.0)));
    mComponents.push_back(Wheel_2);

    Transform transform_2 = Mesh_Wheel_2->GetTransformMatrixHierarchy();
    transform_2.SetPosition(Vector3(10,0,-10));
    IRigidBody* physBody_Wheel_2 = mDynamicsWorld->CreateRigidBody(transform_2);
    IProxyShape* proxy_sphere_2 = physBody_Wheel_2->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    AddPhysicsProxyInModel(Wheel_2,proxy_sphere_2);
    physBody_Wheel_2->SetType(BodyType::DYNAMIC);

    //--------------------------------------//

    MeshGenerator::EllipsoidDescriptor sphere_dscp_wheel_3(Vector3(4));
    IMesh *Mesh_Wheel_3 = new IMeshGenerate(sphere_dscp_wheel_3);
    IComponentMesh *Wheel_3 = new IComponentMesh(Mesh_Wheel_3);
    Wheel_3->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-10,0,-10.0)));
    mComponents.push_back(Wheel_3);

    Transform transform_3 = Mesh_Wheel_3->GetTransformMatrixHierarchy();
    transform_3.SetPosition(Vector3(-10,0,-10));
    IRigidBody* physBody_Wheel_3 = mDynamicsWorld->CreateRigidBody(transform_3);
    IProxyShape* proxy_sphere_3 = physBody_Wheel_3->AddCollisionShape(new ICollisionShapeSphere(4),1.f);
    AddPhysicsProxyInModel(Wheel_3,proxy_sphere_3);
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


    mDynamicsWorld->addNoCollisionPair(physBody_Wheel_0,physBody_Target_4);
    mDynamicsWorld->addNoCollisionPair(physBody_Wheel_1,physBody_Target_4);
    mDynamicsWorld->addNoCollisionPair(physBody_Wheel_2,physBody_Target_4);
    mDynamicsWorld->addNoCollisionPair(physBody_Wheel_3,physBody_Target_4);

    //--------------------------------------//

    IHingeJointInfo(physBody_Target_4,physBody_Wheel_0,transform_0.GetPosition(),Vector3::X);
    IHingeJointInfo HingeJointInfoWheel0(physBody_Wheel_0,physBody_Target_4,transform_0.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel1(physBody_Wheel_1,physBody_Target_4,transform_1.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel2(physBody_Wheel_2,physBody_Target_4,transform_2.GetPosition(),Vector3::Z);
    IHingeJointInfo HingeJointInfoWheel3(physBody_Wheel_3,physBody_Target_4,transform_3.GetPosition(),Vector3::Z);

    HingeJointInfoWheel0.isMotorEnabled = true;
    HingeJointInfoWheel1.isMotorEnabled = true;
    HingeJointInfoWheel2.isMotorEnabled = true;
    HingeJointInfoWheel3.isMotorEnabled = true;

    IHingeJoint* Wheel_HingeJoint0 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel0));
    IHingeJoint* Wheel_HingeJoint1 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel1));
    IHingeJoint* Wheel_HingeJoint2 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel2));
    IHingeJoint* Wheel_HingeJoint3 = static_cast<IHingeJoint*>(mDynamicsWorld->CreateJoint(HingeJointInfoWheel3));

    Wheel_HingeJoint0->EnableMotor(true);
    Wheel_HingeJoint1->EnableMotor(true);
    Wheel_HingeJoint2->EnableMotor(true);
    Wheel_HingeJoint3->EnableMotor(true);

    Wheel_HingeJoint0->SetMaxMotorTorque(500);
    Wheel_HingeJoint1->SetMaxMotorTorque(500);
    Wheel_HingeJoint2->SetMaxMotorTorque(500);
    Wheel_HingeJoint3->SetMaxMotorTorque(500);


    mRoboCar = new VehicleRobotCar(physBody_Target_4,
                                   physBody_Wheel_0,
                                   physBody_Wheel_1,
                                   physBody_Wheel_2,
                                   physBody_Wheel_3,
                                   Wheel_HingeJoint0,
                                   Wheel_HingeJoint1,
                                   Wheel_HingeJoint2,
                                   Wheel_HingeJoint3);

    Transform t;
    t.SetBasis(Matrix3::CreateRotationAxis(Vector3::Y,M_PI * 4.0));
    t.SetPosition(Vector3::Y * 4 + m_EndPoint);
    mRoboCar->AddTransform(t);



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

}

void SceneEngineRobocar::render(float FrameTime)
{


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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera->getCamera2().ViewMatrix());

   //=================================================//


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

    glFlush();
}

void SceneEngineRobocar::update()
{

    if(mKeys[Qt::Key_U])
    {
        m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
        if((m_EndPoint - m_PointS).LengthSquare() < 0.2 )
        {
            num++;
            if( num%(int)mPoints.size() == 0) num = 0;
            m_EndPoint = mPoints[num];
        }
    }


    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {
        (*it)->Updatee();
    }


    if(mRoboCar && m_IsDynamic_LQR)
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


    if(mSceneDscriptor.m_IsSimulateDynamics)
    {
        mDynamicsWorld->UpdateFixedTimeStep(mTimeStep);
    }
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

        }
    }
}

void SceneEngineRobocar::mousePress(float x, float y, int button)
{
    mMouseButton = button;
    data_mouse.mouseX = data_mouse.mouseOldX = x;
    data_mouse.mouseY = data_mouse.mouseOldY = y;


}

void SceneEngineRobocar::mouseReleasePress(float x, float y, int button)
{
    mMouseButton = button;
    data_mouse.mouseX = data_mouse.mouseOldX = x;
    data_mouse.mouseY = data_mouse.mouseOldY = y;

}

void SceneEngineRobocar::mouseWheel(float delta)
{
    mCameraZDistance += (delta * 0.02f);
    mCameraZDistance  = IMath::IClamp(IMath::IAbs(mCameraZDistance),5.f,1024.f);
}



//static float ang = 0;
void SceneEngineRobocar::keyboard(int key)
{
    if(key == Qt::Key_T)
    {
    }

}

void SceneEngineRobocar::destroy()
{

}
