#include "SceneEngineRobocar.h"
#include "Engine/IAlgorithm/IQuickClipping.h"
#include "IEngineFactory.h"

#include <GL/freeglut.h>

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"
#include <freetype2/ft2build.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <cmath>


void wera_test( int bit_mask )
{
    std::cout << bit_mask << std::endl;
}


//----------------------------------------------------------//

VehicleRobotCar *FactoryMethod::CretaeRobotCar()
{

    //--------------------------------------//

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
    proxy_sphere_4->SetCollisionCategoryBits(0x01);
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
    proxy_sphere_0->SetCollisionCategoryBits(0x01);
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
    proxy_sphere_1->SetCollisionCategoryBits(0x01);
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
    proxy_sphere_2->SetCollisionCategoryBits(0x01);
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
    proxy_sphere_4->SetCollisionCategoryBits(0x01);
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


    numer_end = 0;
    //===========================//


    glClearColor(0.f,0.f,0.0f,1.f);

    //Font shaders
    if (!mProgramFont.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vshaderFont30.glsl"))
        qDebug() << "close()";
    if (!mProgramFont.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fshaderFont30.glsl"))
        qDebug() << "close()";
    if (!mProgramFont.link())
        qDebug() << "close()";

    mFontProvider.initializeFontProvider();


    // Max_Length = 25;
    Max_MotorPower = 100;

    m_PointS = Vector3::X * 5.f;
    m_IsDynamic_LQR = false;
    m_IsTrackingMove = false;
    m_IsConnectUDP = false;
    m_IsTracking = false;
    m_isHelp = true;

    num=0;
    m_EndPoint = m_PointS = Vector3(0,-13.5,0);

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

    Transform init_transform;
    init_transform.SetPosition(m_pickPoint = m_PointS = m_EndPoint + Vector3::Y * 3);

    Quaternion q = Quaternion::FromAngleAxis(Vector3::Y,M_PI * -2.f);
    init_transform.SetBasis(q.GetRotMatrix());

    mRoboCar = mFactoryMethod->CreateRobot(
                   RobotDescriptor(Vector3(25.0,1.0,12.0),4),init_transform);


    //--------------------------------------//

    MeshGenerator::CuboidDescriptor cuboid_dscp(Vector3(500.0,5.0,500.0));
    IMesh *BoxMeshBottom = new IMeshGenerate(cuboid_dscp);
    IComponentMesh *BoxBottom = new IComponentMesh(BoxMeshBottom);
    BoxBottom->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,-15,0)));
    mComponents.push_back(BoxBottom);

    Transform transform = BoxMeshBottom->GetTransfom();
    transform.SetPosition(Vector3::Y * -20.f);
    IRigidBody *physBody = mDynamicsWorld->CreateRigidBody(transform);
    IProxyShape* proxy = physBody->AddCollisionShape(new ICollisionShapeBox(Vector3(500.0,5.0,500.0)),10.f);
    AddPhysicsProxyInModel(BoxBottom,proxy);
    physBody->SetType(BodyType::STATIC);

    //--------------------------------------//


//    MeshGenerator::EllipsoidDescriptor cuboid_dscp_choice(Vector3(1.0,1.0,1.0));
//    IMesh *BoxMeshChoice = new IMeshGenerate(cuboid_dscp_choice);
//    IComponentMesh *BoxChoice = new IComponentMesh(BoxMeshChoice);
//    NuzzleChoice = BoxChoice;
//    BoxChoice->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,-10)));
//    mComponents.push_back(BoxChoice);

    //--------------------------------------//

    m_PointS = mRoboCar->physBody_Base->GetTransform().GetPosition();

    //--------------------------------------//

    m_TargetPoint = Vector3(0,4,0);


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
     // glEnable(GL_SCISSOR_TEST);
     // glScissor(0,0,mWidth,mHeight);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glLoadIdentity();
      glMatrixMode(GL_PROJECTION);
      glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

      glMatrixMode(GL_MODELVIEW);
      glLoadMatrixf(mCamera->getCamera2().ViewMatrix());

     //=================================================//

      /**/
      glPushMatrix();
      glColor3f(1,0,0);
      GLUquadric *quad;
      quad = gluNewQuadric();
      glTranslatef(m_PointS.x,m_PointS.y=-13.5,m_PointS.z);
      gluSphere(quad,2,100,20);
      glPopMatrix();
      /**/
      {
          glPushMatrix();
          glColor3f(1,0,0);
          GLUquadric *quad;
          quad = gluNewQuadric();
          Vector3 p = mRoboCar->physBody_Base->GetTransform().GetPosition();
          glTranslatef(p.x,p.y+5,p.z);
          gluSphere(quad,2,20,20);
          glPopMatrix();
      }

      //===========================================//

      for( const auto &it : mTrackerPoints )
      {
          glPushMatrix();
          glColor3f(1,0,0);
          GLUquadric *quad;
          quad = gluNewQuadric();
          glTranslatef(it.x,it.y,it.z);
          gluSphere(quad,0.4,100,20);
          glPopMatrix();
      }

      for( int i=0; i < (int)mTrackerPoints.size()-1; ++i )
      {
          glPushMatrix();
          glColor3f(0,1,0);
          glLineWidth(3);
          glBegin(GL_LINES);
          glVertex3fv(mTrackerPoints[i]);
          glVertex3fv(mTrackerPoints[((i+1)%mTrackerPoints.size() != 0)? i+1 : 0]);
          glEnd();
          glPopMatrix();
      }
      //===========================================//

      if(mSceneDscriptor.m_IsSimulateDynamics)
      {
          if(mRoboCar->physBody_Base)
          {

          }
      }

      //===========================================//

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

      glCullFace(GL_BACK);
      glEnable(GL_DEPTH_TEST);
      //glEnable(GL_SCISSOR_TEST);
      //glEnable(GL_STENCIL_TEST);
      //glViewport(0,mHeight - mHeight/4, mWidth/4,mHeight/4);
      //glScissor(0,mHeight - mHeight/4,mWidth/4,mHeight/4);
      //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glViewport(0,0, mWidth,mHeight);

      QVector4D fontColor(0.0f, 1.0f, 0.0f, (m_isHelp) ? 0.5f : 1.0f);
      mProgramFont.bind();

      QMatrix4x4 modelMatrix;
      modelMatrix.setToIdentity();

      QMatrix4x4 OrthoProjectionMatrix;
      OrthoProjectionMatrix.ortho(0,100,0,100,-1,1);

      mProgramFont.setUniformValue("mvp_matrix", OrthoProjectionMatrix * modelMatrix);
      mProgramFont.setUniformValue("textColor", fontColor);
      mProgramFont.setUniformValue("text", 0);


      float yaw = IMath::IRadiansToDegrees(mRoboCar->physBody_Base->GetTransform().GetRotation().GetEulerAngles3().y);
      QString stringToDisplay_Angle = QString("Angle: [ yaw: ") + QString::number(yaw, 'f', 2) + "° ]";

      Vector3 pos = mRoboCar->physBody_Base->GetTransform().GetPosition();
      QString stringToDisplay_Position = QString("Position[ x: %1   y: %2   z: %3 ]")
                                         .arg(QString::number(pos.x, 'f', 2))
                                         .arg(QString::number(pos.y, 'f', 2))
                                         .arg(QString::number(pos.z, 'f', 2));

      mFontProvider.drawFontGeometry(&mProgramFont,1.0f, 95.0f, stringToDisplay_Position, 0.05f);
      mFontProvider.drawFontGeometry(&mProgramFont,1.0f, 90.0f, stringToDisplay_Angle, 0.05f);


      if(m_isHelp)
      {
          mProgramFont.setUniformValue("textColor", QVector4D(0,1,0,0.95));
          mFontProvider.drawFontGeometry(&mProgramFont,80.0f, 95.0f, "Force Motor Up [Key_W]", 0.03f);
          mFontProvider.drawFontGeometry(&mProgramFont,80.0f, 90.0f, "Force Motor Down [Key_S]", 0.03f);
          mFontProvider.drawFontGeometry(&mProgramFont,80.0f, 85.0f, "Roatate Left [Key_A]", 0.03f);
          mFontProvider.drawFontGeometry(&mProgramFont,80.0f, 80.0f, "Roatate Right [Key_D]", 0.03f);
          mFontProvider.drawFontGeometry(&mProgramFont,80.0f, 75.0f, "Help Show [Key_H]", 0.03f);
      }



      mProgramFont.release();
      glLoadIdentity();


      //=================================================//

      if (mGizmoManipulator->GetGizmo())
      {
          mGizmoManipulator->GetGizmo()->SetCameraMatrix( mCamera->getCamera2().ViewMatrix() ,
                                                          mCamera->getCamera2().ProjectionMatrix() );
      }

      //=================================================//






      //=================================================//

  //    //glCullFace(GL_BACK);
  //    glCullFace(GL_FRONT);
  //    glEnable(GL_DEPTH_TEST);
  //    glViewport(0, 0, mWidth, mHeight);
  //    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


  //    glLoadIdentity();
  //    glMatrixMode(GL_PROJECTION);
  //    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

  //    glMatrixMode(GL_MODELVIEW);
  //    glLoadMatrixf(mCamera->getCamera2().ViewMatrix());

  //    if (mGizmoManipulator->GetGizmo())
  //    {
  //        mGizmoManipulator->GetGizmo()->SetCameraMatrix( mCamera->getCamera2().ViewMatrix() ,
  //                                                        mCamera->getCamera2().ProjectionMatrix() );

  //        if(mGizmoManipulator->mSelectedIndexID >= 0)
  //        {
  //            glLineWidth(5);
  //            mGizmoManipulator->GetGizmo()->Draw();
  //            glLineWidth(1);
  //        }
  //    }

  //    glLoadIdentity();


      glFlush();

}

void SceneEngineRobocar::update()
{

    //------------------------------------------------------------------------------------------------------------------//

//    if(mGizmoManipulator->GetGizmo() && mGizmoManipulator->isSelectedIndex())
//    {
//        if( !mGizmoManipulator->mSelectedIndexIDs.empty() && mGizmoManipulator->mSelectedIndexID >= 0 )
//        {
//            for( auto it = mGizmoManipulator->mSelectedIndexIDs.begin(); it != mGizmoManipulator->mSelectedIndexIDs.end(); ++it )
//            {
//                Vector3 worldPoint =mGizmoManipulator->mTransformInitilization.GetTranslation();
//                Matrix4 m = Matrix4::CreateTranslation( worldPoint) * mGizmoManipulator->GetGizmo()->GetSHIFTMatrix() *
//                            Matrix4::CreateTranslation(-worldPoint) * (it->second);

//                mComponents[it->first]->SetTransform(m);
//            }
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
          if((m_EndPoint - m_PointS).LengthSquare() < 0.5f )
          {
              if(num < (int)mTrackerPoints.size() )
              {
                  m_EndPoint = mTrackerPoints[num++];
              }
          }

         // if((mTrackerPoints[mTrackerPoints.size()-1] - m_PointS).LengthSquare() > 10)
          {
              mRoboCar->mDispatcherAttribute.extrem_MIN_MAX = Max_MotorPower;
              mRoboCar->UpdateControlPointGuidance(m_PointS);
          }

      }
      else
      {

          if(m_IsTrackingMove)
          {
              Vector3 p = mRoboCar->physBody_Base->GetTransform().GetPosition();
              if( (p - m_pickPoint).Length() > 20 )
              {
                  m_pickPoint = p;
                  mTrackerPoints.push_back(m_pickPoint);
              }
          }


          if(m_IsConnectUDP)
          {
              float coff = data_trransmission.kd;
              float MotorPower = data_trransmission.speed_PWM_X = IMath::IClamp(data_trransmission.speed_PWM_X * coff, -Max_MotorPower , Max_MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorA(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorB(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorC(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorD(MotorPower);
              //          Vector3 Dir = mRoboCar->physBody_Base->GetTransform().GetRotation() * Vector3::X;
              //          mRoboCar->UpdateControlPointGuidance(m_PointS += data_trransmission.speed_PWM_X * Dir * coff * 0.01);

              mRoboCar->mFixOrientation = Quaternion::FromAngleAxis( Vector3::Y , angle_yaw += data_trransmission.turn * 0.0001);
            //mRoboCar->Stop();
          }
          else// if(!m_IsTracking)
          {

              if(keyDown(Qt::Key_W))
              {
                 data_trransmission.speed_PWM_X++;
                 qDebug() << "Key_W";
              }

              if(keyDown(Qt::Key_S))
              {
                 data_trransmission.speed_PWM_X--;
                 qDebug() << "Key_S";
              }

              if(keyDown(Qt::Key_A))
              {
                 mRoboCar->mFixOrientation = Quaternion::FromAngleAxis( Vector3::Y , angle_yaw -= 50 * 0.0001);
                 qDebug() << "Key_A";
              }

              if(keyDown(Qt::Key_D))
              {
                 mRoboCar->mFixOrientation = Quaternion::FromAngleAxis( Vector3::Y , angle_yaw += 50 * 0.0001);
                 qDebug() << "Key_D";
              }

              float coff = 1.0;
              float MotorPower = data_trransmission.speed_PWM_X = IMath::IClamp(data_trransmission.speed_PWM_X * coff, -Max_MotorPower , Max_MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorA(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorB(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorC(MotorPower);
              mRoboCar->mVehicleRobot->setPos_motorD(MotorPower);

          }
      }

      if(m_IsFix == true)
      {
          qDebug() << "Stop";
          mRoboCar->Stop();
          m_IsFix = false;
      }
    }


    if(mSceneDscriptor.m_IsSimulateDynamics)
    {
        mDynamicsWorld->UpdateFixedTimeStep(mTimeStep);
        mRoboCar->Update(mTimeStep,m_PointS);
    }


    //--------------------------------------------------------------//


    if(mKeys[Qt::Key_U])
    {
        m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
        if((m_EndPoint - m_PointS).LengthSquare() < 0.5f )
        {
            num++;
            if( num%(int)mTrackerPoints.size() == 0) num = 0;
        }
    }


    if(keyDown(Qt::Key_2))
    {
       qDebug() << "Key_2 Stop";
       mRoboCar->Stop();
    }

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
                IRay casting_ray = mGizmoManipulator->BuildRayMouse( data_mouse.mouseX , data_mouse.mouseY );

                int raycastWithCategoryMaskBits = 0x0001;
                IRaycastInfoOutput RayOutput(raycastWithCategoryMaskBits);
                mDynamicsWorld->raycast(casting_ray,&RayOutput,raycastWithCategoryMaskBits);

                m_EndPoint = m_PointS = mRoboCar->physBody_Base->GetTransform().GetPosition();
                m_TargetPoint = RayOutput.raycastInfo().worldPoint + Vector3::Y * 4.f;

                num=0;
                mTrackerPoints.clear();
                mTrackerPoints.push_back(m_PointS);
                mTrackerPoints.push_back(m_TargetPoint);

                //-------------------------------------------//

                IRay casting_ray_robot_track(m_PointS, (m_TargetPoint - m_PointS).Normalized());

                int raycastWithCategoryMaskBits2 = 555;
                IRaycastInfoOutput RayOutput2(raycastWithCategoryMaskBits2);
                mDynamicsWorld->raycast(casting_ray_robot_track,&RayOutput2,raycastWithCategoryMaskBits2);

                hull.clear();
                auto proxy = RayOutput2.raycastInfo().proxyShape;
                auto worldPoint = RayOutput2.raycastInfo().worldPoint;

                if(proxy)
                {
                    Vector3 targetAxis = IMath::Cross(m_PointS - m_TargetPoint, Vector3::Y);
                    targetAxis.Normalize();

                    Vector3 Axis = (isRevert)? -Vector3::Y : Vector3::Y;
                    IPlane plane(Vector3::Y , proxy->GetWorldTransform().GetPosition());


                    Vector3 min,max;
                    proxy->GetCollisionShape()->GetLocalBounds(min,max);

                    Vector3 a_max = worldPoint - casting_ray_robot_track.Direction * 1;
                    Vector3 b_min = worldPoint + casting_ray_robot_track.Direction * 5;
                    b_min += ((max - min)).Length() * casting_ray_robot_track.Direction;
                    a_max -= ((max - min)).Length() * casting_ray_robot_track.Direction;

                    qDebug() << "MAX - MIN : " <<(max - min).Length();


                    hull.clear();
                    CallbeckSupprtCollide cl_proxy(proxy, b_min, a_max);
//                    IGrahamScan2dConvexHull::ScanConvexHull2D(cl_proxy,hull,IPlane(Axis,m_TargetPoint * Vector3::Y));

                    Vector3 a_sp = plane.ClosestPoint(cl_proxy.SupportPoint( targetAxis));
                    Vector3 b_sp = plane.ClosestPoint(cl_proxy.SupportPoint(-targetAxis));


                    scalar ll_max = (m_PointS - a_sp).Dot(-targetAxis);
                    scalar rr_max = (m_PointS - b_sp).Dot( targetAxis);

                    std::cout << "ll_max: " << ll_max << std::endl;
                    std::cout << "rr_max: " << rr_max << std::endl;

                    if(ll_max > rr_max) Axis = -Axis;

//                    Vector3 aa = worldPoint - casting_ray_robot_track.Direction * 5;
//                    Vector3 bb = worldPoint + casting_ray_robot_track.Direction * 5;

                    hull.clear();
                    IScanColsingPath::ScanClosingPath(cl_proxy,
                                                   hull,
                                                   IPlane(Axis,m_TargetPoint * Vector3::Y),
                                                   a_max,
                                                   b_min);
                    qDebug() << "count hull points: " << (numer_end=hull.size());


                    if(!hull.empty())
                    {

                        num=0;
                        mTrackerPoints.clear();
                        mTrackerPoints.push_back(m_PointS);
                        for (int i = hull.size()-1; i >= 0; --i)
                        {
                             mTrackerPoints.push_back( hull[i] );
                        }
                        mTrackerPoints.push_back(m_TargetPoint);
                    }





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


}

void SceneEngineRobocar::mouseReleasePress(float x, float y, int button)
{
    mMouseButton = button;
    data_mouse.mouseX = data_mouse.mouseOldX = x;
    data_mouse.mouseY = data_mouse.mouseOldY = y;


//    if(  mMouseButton == Qt::MouseButton::LeftButton )
//    {
//         //================ Save transform geometry components  =================//
//         if (mGizmoManipulator->GetGizmo())
//         {
//            mGizmoManipulator->GetGizmo()->IndentitySHIFTMatrix();
//            if( !mSelectedIndexIds.empty() )
//            {
//                    mGizmoManipulator->GetGizmo()->OnMouseUp( data_mouse.mouseX , data_mouse.mouseY );

//                    Matrix4 Interpolation = Matrix4::ZERO;
//                    for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
//                    {
//                        mGizmoManipulator->mSelectedIndexIDs[*it] = mComponents[*it]->GetTransformMatrixHierarchy();
//                        Interpolation += mComponents[*it]->GetTransformMatrixHierarchy();
//                    }
//                    mGizmoManipulator->mTransformInitilization = Interpolation / float(mSelectedIndexIds.size());
//             }

//             mGizmoManipulator->mSelectedIndexIDs.clear();
//             for( auto it = mSelectedIndexIds.begin(); it != mSelectedIndexIds.end(); ++it )
//             {
//                 mGizmoManipulator->mSelectedIndexIDs.insert( std::make_pair( *it , mComponents[*it]->GetTransformMatrixHierarchy()) );
//             }
//         }
//     }

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
        MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(20.0,10.0,20));
        IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
        Transform transform_4 = Transform::Identity();

        int min = -250;
        int max =  250;
        int range = max - min + 1;
        int rand_x = rand() % range + min;
        int rand_y = rand() % range + min;
        transform_4.SetPosition(Vector3(rand_x,10,rand_y));
        //BoxDown->SetTransform(transform_4);
        mComponents.push_back(BoxDown);


        IRigidBody* physBody_Targ = mDynamicsWorld->CreateRigidBody(transform_4);

        Transform trans = Transform::Identity();
        IProxyShape* proxy_sphere_4 = physBody_Targ->AddCollisionShape(new ICollisionShapeBox(cuboid_dscp_down.size),1.f,trans);
        proxy_sphere_4->SetCollisionCategoryBits(555);
        AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);
        physBody_Targ->SetType(BodyType::DYNAMIC);
        //physBody_Target_4->setIsMove(false);

        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_0);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_1);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_2);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_3);
    }



    if(key == Qt::Key_X)
    {
        MeshGenerator::CylinderDescriptor cylinder_dsc(Vector2(8.0,4.0),8,IVector2D<int>(9,9));
        IComponentMesh *CylinderGeometry = new IComponentMesh(new IMeshGenerate(cylinder_dsc));
        Transform transform = Transform::Identity();

        int min = -250;
        int max =  250;
        int range = max - min + 1;
        int rand_x = rand() % range + min;
        int rand_y = rand() % range + min;
        transform.SetPosition(Vector3(rand_x,10,rand_y));
        //transform.SetPosition(Vector3(250 - rand()%250,10, 250 - rand()%250));
        //BoxDown->SetTransform(transform_4);
        mComponents.push_back(CylinderGeometry);


        IHullDescriptor hull_dsc(new ICalcModelHull(CylinderGeometry->Modelmesh()->GenerateSTLVertices()));

        IRigidBody* physBody_Targ = mDynamicsWorld->CreateRigidBody(transform);

        Transform trans = Transform::Identity();
        IProxyShape* proxy_sphere_4 = physBody_Targ->AddCollisionShape(new ICollisionShapeHull(hull_dsc),1.f,trans);
        proxy_sphere_4->SetCollisionCategoryBits(555);
        AddPhysicsProxyInModel(CylinderGeometry,proxy_sphere_4);
        physBody_Targ->SetType(BodyType::DYNAMIC);
        //physBody_Targ->setIsMove(false);

        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_0);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_1);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_2);
        mDynamicsWorld->addNoCollisionPair(physBody_Targ, mRoboCar->physBody_Wheel_3);
    }



    if(key == Qt::Key_B)
    {
        isRevert = !isRevert;
    }


    if(key == Qt::Key_H)
    {
       m_isHelp = !m_isHelp;
    }
}

void SceneEngineRobocar::destroy()
{
   // mPoints.clear();
    mTrackerPoints.clear();
    mDynamicsWorld->Destroy();

}

void SceneEngineRobocar::Stop()
{
    if(mRoboCar)
    {
        mRoboCar->Stop();
    }
}

//-----------------------------------------------------------//








