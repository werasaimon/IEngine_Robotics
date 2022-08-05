#include "SceneEngineRobocar.h"
#include "IEngineFactory.h"

#include <GL/freeglut.h>

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"

#include <freetype2/ft2build.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <cmath>


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


    //Font shaders
    if (!mProgramFont.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vshaderFont30.glsl"))
        qDebug() << "close()";
    if (!mProgramFont.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fshaderFont30.glsl"))
        qDebug() << "close()";
    if (!mProgramFont.link())
        qDebug() << "close()";

    mFontProvider.initializeFontProvider();


    Max_Length = 25;

    m_PointS = Vector3::X * 5.f;
    m_IsDynamic_LQR = false;
    m_IsTrackingMove = false;

    //===========================//

//    for( int i=0; i < 5; ++i )
//    {
//        float x = i * 30;
//        float y = cos(i*2) * 20;
//        float z =-13.5;
//        mPoints.push_back( m_PointS + Vector3(x,z,y) - Vector3::Z * 20);
//    }

//    Vector3 end = mPoints[mPoints.size()-1];
//    for( int i=0; i < 4; ++i )
//    {
//        float x = i * 30;
//        float y = sin(i*2) * 20;
//        float z =-13.5;
//        mPoints.push_back( Vector3(y,0,x) );
//    }


    mPoints.push_back( Vector3(50,-13.5,50) + Vector3::X * 15.);
    mPoints.push_back( Vector3(-50,-13.5,50) + Vector3::X * 15.);
    mPoints.push_back( Vector3(-50,-13.5,-50) + Vector3::X * 15.);
    mPoints.push_back( Vector3(50,-13.5,-50) + Vector3::X * 15.);

    m_EndPoint = mPoints[num=0];
    m_PointS = mPoints[0];

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

            mRoboCar = mFactoryMethod->CreateRobot(RobotDescriptor(Vector3(25.0,1.0,12.0),4),init_transform);


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

            MeshGenerator::CuboidDescriptor cuboid_lidar(Vector3(5.0,5.0,5.0));
            mBoxLidar = new IComponentMesh(new IMeshGenerate(cuboid_lidar));
            mBoxLidar->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
            mComponents.push_back(mBoxLidar);


            //RaycastingCallback = new  IRaycastCallbackInformer();

            //--------------------------------------//

            m_PointS = mRoboCar->physBody_Base->GetTransform().GetPosition();

            //-------------------------------------//


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

//    glPushMatrix();
//    glColor3f(1,0,0);
//    glBegin(GL_LINES);
//    glVertex3fv(m_PointS);
//    glVertex3fv(p);
//    glEnd();
//    glPopMatrix();


     float distance;
     float angle;

    if(mSceneDscriptor.m_IsSimulateDynamics)
    {
        if(mRoboCar->physBody_Base)
        {
//            Transform t = mRoboCar->physBody_Base->GetTransform();
//            t.SetPosition( t.GetPosition() + t.GetBasis() * Vector3::Y * 9.f);
//            t.SetBasis( Matrix3::CreateRotationAxis(Vector3::Y, mAngleEyeLidar += 0.1f ));
//            mBoxLidar->SetTransform(t);

            Transform rot( Vector3::ZERO, Matrix3::CreateRotationAxis(Vector3::Y, mAngleEyeLidar += 0.1f ));
            Transform shift(Vector3::Y * 8, Matrix3::IDENTITY);
            Transform t = mRoboCar->physBody_Base->GetTransform() * shift * rot;
            mBoxLidar->SetTransform(t);

            //t.SetBasis( mRoboCar->physBody_Base->GetTransform().GetBasis().GetInverse() * t.GetBasis());



            qDebug() << MaxDistanceLIDAR;

            ISensorLIDAR mSensorLidare( mDynamicsWorld , /*mRoboCar->physBody_Base->GetTransform().GetBasis() **/ (Vector3::X + Vector3::Y * -0.1f) , t);
            distance = mSensorLidare.CalculateDistance(MaxDistanceLIDAR);
            angle = mSensorLidare.LAngleRotation(t.GetBasis() * Vector3::Y);
            angle = IMath::IRadiansToDegrees(-angle + M_PI);

            // qDebug() << "Angle: " << angle;// + 180 * IMath::ISign(angle);

            if( 350 > angle )
            {
                qDebug() << "Angle: " << angle;
                mLiDARPoints.push_back(PointLIDAR(distance,angle,t.GetPosition() + mSensorLidare.WorldDirection() * distance));
            }
            else if(!mLiDARPoints.empty())
            {
               mLiDARPoints.clear();
               qDebug() << "Clear";
            }



            auto p1 = t.GetPosition();
            auto p2 = t.GetPosition() + mSensorLidare.WorldDirection() * distance;

            glPushMatrix();
            glColor3f(0,1,0);
            glLineWidth(3);
            glBegin(GL_LINES);
            glVertex3fv( p1 );
            glVertex3fv( p2 );
            glEnd();
            glPopMatrix();

            glPushMatrix();
            glColor3f(1,0,0);
            GLUquadric *quad;
            quad = gluNewQuadric();
            glTranslatef(p2.x,p2.y,p2.z);
            gluSphere(quad,0.4,100,20);
            glPopMatrix();
        }


        for(unsigned int ii = 0; ii < mLiDARPoints.size(); ii++)
        {
            Vector3 p = mLiDARPoints[ii].point;
            glPushMatrix();
            glColor3f(1,0,0);
            GLUquadric *quad;
            quad = gluNewQuadric();
            glTranslatef(p.x,p.y,p.z);
            gluSphere(quad,0.4,100,20);
            glPopMatrix();

        }
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


  //    glDisable(GL_DEPTH_TEST);
  //    glDisable(GL_LIGHTING);
  //    glEnable(GL_BLEND);
  //    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  //    glDisable(GL_CULL_FACE);

      glClearColor(0, 0, 0, 0);
     //
      glCullFace(GL_BACK);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_SCISSOR_TEST);
      glEnable(GL_STENCIL_TEST);
      glViewport(0,mHeight - mHeight/4, mWidth/4,mHeight/4);
      glScissor(0,mHeight - mHeight/4,mWidth/4,mHeight/4);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      //
      glLoadIdentity();


//      auto b = mRoboCar->physBody_Base->GetTransform().GetBasis();
//      float shift_angle = Vector3::AngleSigned(Vector3::X , b * Vector3::X, b * Vector3::Y);





      char buff[255] = {0};
      sprintf(buff , "LIDAR DISTANCE : %f" , distance);

      QVector4D fontColor(1.0f, 0.0f, 0.0f, 1.0f);
      mProgramFont.bind();

      QMatrix4x4 modelMatrix;
      modelMatrix.setToIdentity();

      QMatrix4x4 OrthoProjectionMatrix;
      OrthoProjectionMatrix.ortho(0,100,0,100,-1,1);

      mProgramFont.setUniformValue("mvp_matrix", OrthoProjectionMatrix * modelMatrix);
      mProgramFont.setUniformValue("textColor", fontColor);
      mProgramFont.setUniformValue("text", 0);
      QString stringToDisplay = QString::fromUtf8(buff);
      mFontProvider.drawFontGeometry(&mProgramFont,1.0f, 90.0f, stringToDisplay, 0.1f);
      mProgramFont.release();


      float World_SIZE = 1;

      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();

      glOrtho(-World_SIZE, World_SIZE, -World_SIZE, World_SIZE, -1.0, 1.0);
      glMatrixMode(GL_MODELVIEW);

      glColor3f(0.0f, 1.0f, 0.0f);

      glPushMatrix();
      glLineWidth(4);
      glutWireCube(World_SIZE * 2.0);
      glLineWidth(1);
      glPopMatrix();


      glColor3f(1.0f, 0.0f, 0.0f);

      glPushMatrix();
      glPointSize(4);
      glBegin(GL_POINTS);
      for(unsigned int ii = 0; ii < mLiDARPoints.size(); ii++)
      {
          float theta = IMath::IDegreesToRadians( mLiDARPoints[ii].angle );
          float radius = mLiDARPoints[ii].distance / 100;

          float x = radius * sinf(theta);//calculate the x component
          float y = radius * cosf(theta);//calculate the y component

          auto v = /*mRoboCar->physBody_Base->GetTransform().GetBasis() **/ Vector3(x,y,0);

          glVertex2f(v.x,v.y);//output verte

      }
      glEnd();
      glPopMatrix();


      glPushMatrix();
      glColor3f(1.0f, 1.0f, 1.0f);
      glPointSize(4);
      glBegin(GL_LINES);

      Vector2 prev_point;
      for(unsigned int ii = 0; ii < mLiDARPoints.size(); ii++)
      {
          float theta = IMath::IDegreesToRadians( mLiDARPoints[ii].angle );
          float radius = mLiDARPoints[ii].distance / 100;

          float x = radius * sinf(theta);//calculate the x component
          float y = radius * cosf(theta);//calculate the y component

          if(ii > 0)
          {
             glVertex2f(x,y);//output vertex
             glVertex2f(prev_point.x,prev_point.y);//output vertex
          }

          prev_point = Vector2(x,y);
      }
      glEnd();
      glPopMatrix();


      glLoadIdentity();


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
//          m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
//          if((m_EndPoint - m_PointS).LengthSquare() < 10 )
//          {
//              num++;
//              if( num%(int)mPoints.size() == 0 ) num = 0;
//              m_EndPoint = mPoints[num];
//          }

          Vector3 p = mRoboCar->physBody_Base->GetTransform().GetPosition();
          if( (p - m_PointS).Length() < Max_Length )
          {
              m_PointS += (m_EndPoint - m_PointS).Normalized() * 0.05 * speed_point;
          }

          if((m_EndPoint - m_PointS).LengthSquare() < 10 )
          {
              if(num < (int)mTrackerPoints.size() )
              {
                m_EndPoint = mTrackerPoints[num++];
              }
          }

          mRoboCar->UpdateControlPointGuidance(m_PointS);

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


          float coff = data_trransmission.kd;
          mRoboCar->mVehicleRobot->setPos_motorA(data_trransmission.speed_PWM_X * coff);
          mRoboCar->mVehicleRobot->setPos_motorB(data_trransmission.speed_PWM_X * coff);
          mRoboCar->mVehicleRobot->setPos_motorC(data_trransmission.speed_PWM_X * coff);
          mRoboCar->mVehicleRobot->setPos_motorD(data_trransmission.speed_PWM_X * coff);

          //          Vector3 Dir = mRoboCar->physBody_Base->GetTransform().GetRotation() * Vector3::X;
          //          mRoboCar->UpdateControlPointGuidance(m_PointS += data_trransmission.speed_PWM_X * Dir * coff * 0.01);

          //qDebug() << data_trransmission.turn;
          mRoboCar->mFixOrientation = Quaternion::FromAngleAxis( Vector3::Y , angle_yaw += data_trransmission.turn * 0.0001);
        //mRoboCar->Stop();


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

    if(keyDown(Qt::Key_2))
    {
       qDebug() << "Key_2";
       mRoboCar->Stop();
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
        MeshGenerator::CuboidDescriptor cuboid_dscp_down(Vector3(10.0,10.0,10.0));
        IComponentMesh *BoxDown = new IComponentMesh(new IMeshGenerate(cuboid_dscp_down));
        Transform transform_4 = Transform::Identity();
        transform_4.SetPosition(Vector3(rand()%250,10,rand()%250));
        //BoxDown->SetTransform(transform_4);
        mComponents.push_back(BoxDown);


        IRigidBody* physBody_Targ = mDynamicsWorld->CreateRigidBody(transform_4);

        Transform trans = Transform::Identity();
        IProxyShape* proxy_sphere_4 = physBody_Targ->AddCollisionShape(new ICollisionShapeBox(cuboid_dscp_down.size),1.f,trans);
        AddPhysicsProxyInModel(BoxDown,proxy_sphere_4);
        physBody_Targ->SetType(BodyType::DYNAMIC);
        //physBody_Target_4->setIsMove(false);
    }


}

void SceneEngineRobocar::destroy()
{
    mPoints.clear();
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





