#include "SceneEngineNuzzleGimbal.h"

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"

const Vector2 &SceneEngineNuzzleGimbal::getAngleGimbal() const
{
    return AngleGimbal;
}

EngineGimbalStabilization *SceneEngineNuzzleGimbal::gimbalStabilization() const
{
    return mGimbalStabilization;
}

SceneEngineNuzzleGimbal::SceneEngineNuzzleGimbal()
{

}

const OrientationSensor &SceneEngineNuzzleGimbal::getSensor() const
{
    return Sensor;
}


void SceneEngineNuzzleGimbal::initCamera()
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

void SceneEngineNuzzleGimbal::initialization()
{

    glClearColor(0.f,0.f,0.0f,1.f);

    //===========================//

    NullAllKey();
    initCamera();


    QQL = Quaternion::IDENTITY;

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


    //----------------------- Init Gizmo ------------------------//

    mGizmoManipulator = std::auto_ptr<IGizmoManipulator>(new IGizmoManipulator());
    mGizmoManipulator->InitilizationDefault();
    mGizmoManipulator->Resize(mWidth,mHeight);
    mGizmoManipulator->DisplayScale(2.0);

    mGimbalStabilization = new EngineGimbalStabilization();
    mComponents.push_back(mGimbalStabilization->mGimbalRoot);
    mComponents.push_back(mGimbalStabilization->mGimbalConnectA);
    mComponents.push_back(mGimbalStabilization->mGimbalConnectB);
    mComponents.push_back(mGimbalStabilization->mGimbalConnectC);


    /**

    //-----------------------------------------------------------//

    MeshGenerator::CuboidDescriptor cuboid_dscp2(Vector3(5.0,5.0,5.0));
    IMesh *BoxMesh2 = new IMeshGenerate(cuboid_dscp2);
    IComponentMesh *Box2 = new IComponentMesh(BoxMesh2);
    Nuzzle = Box2;
    Box2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
    mComponents.push_back(Box2);


    //MeshGenerator::CuboidDescriptor cuboid_dscp3(Vector3(10.0,1.0,10.0));

    MeshGenerator::CylinderDescriptor cilinder_dscp(Vector2(5,5),1);
    IMesh *CilinderMesh3 = new IMeshGenerate(cilinder_dscp);
    IComponentMesh *Box3 = new IComponentMesh(CilinderMesh3);
    NuzzleChild1 = Box3;
    Box3->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,3.3f,0)));
    mComponents.push_back(Box3);
    Box2->AddChild(Box3);


    MeshGenerator::CuboidDescriptor cuboid_sub0(Vector3(1.0,4.5,1.0));
    IMesh *BoxMeshSub0 = new IMeshGenerate(cuboid_sub0);
    IComponentMesh *BoxSub0 = new IComponentMesh(BoxMeshSub0);
    BoxSub0->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(4.5,2.8,0)));
    mComponents.push_back(BoxSub0);
    Box3->AddChild(BoxSub0);


    MeshGenerator::CuboidDescriptor cuboid_sub1(Vector3(1.0,4.5,1.0));
    IMesh *BoxMeshSub1 = new IMeshGenerate(cuboid_sub1);
    IComponentMesh *BoxSub1 = new IComponentMesh(BoxMeshSub1);
    BoxSub1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-4.5,2.8,0)));
    mComponents.push_back(BoxSub1);
    Box3->AddChild(BoxSub1);


    //============================================================//

    MeshGenerator::CuboidDescriptor cuboid_dscp4(Vector3(8.0,5.0,1.0));
    IMesh *BoxMesh4 = new IMeshGenerate(cuboid_dscp4);
    IComponentMesh *Box4 = new IComponentMesh(BoxMesh4);
    NuzzleChild2 = Box4;
    Box4->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,0)));
    mComponents.push_back(Box4);
    Box3->AddChild(Box4);


    MeshGenerator::CuboidDescriptor cuboid_dscp_nuzzle(Vector3(3.0,3.0,3.0));
    IMesh *BoxMeshNuzzle = new IMeshGenerate(cuboid_dscp_nuzzle);
    IComponentMesh *BoxNuzzle = new IComponentMesh(BoxMeshNuzzle);
    BoxNuzzle->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,0,0)));
    mComponents.push_back(BoxNuzzle);
    Box4->AddChild(BoxNuzzle);


    /**/


    //============================================================//


    MeshGenerator::EllipsoidDescriptor cuboid_dscp6(Vector3(1.0,1.0,1.0));
    IMesh *BoxMesh6 = new IMeshGenerate(cuboid_dscp6);
    IComponentMesh *Box6 = new IComponentMesh(BoxMesh6);
    NuzzleChoice = Box6;
    Box6->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,-10)));
    mComponents.push_back(Box6);

    //============================================================//

//    MeshGenerator::CuboidDescriptor cuboid_dscp_eye(Vector3(3.0,3.0,3.0));
//    IMesh *BoxMeshEye = new IMeshGenerate(cuboid_dscp_eye);
//    IComponentMesh *BoxEye = new IComponentMesh(BoxMeshEye);
//    EyeCube = BoxEye;
//    BoxEye->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,15,0)));
//    mComponents.push_back(BoxEye);

    //============================================================//



    /**

    MeshGenerator::CuboidDescriptor cuboid_dscp33(Vector3(40.0,10.0,10.0));
    IMesh *BoxMesh33 = new IMeshGenerate(cuboid_dscp33);
    IComponentMesh *Box33 = new IComponentMesh(BoxMesh33);
    Box33->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-20,20,0)));
    mComponents.push_back(Box33);
    Box3->AddChild(Box33);



    MeshGenerator::CuboidDescriptor cuboid_dscp4(Vector3(20.0,55.0,10.0));
    IMesh *BoxMesh4 = new IMeshGenerate(cuboid_dscp4);
    IComponentMesh *Box4 = new IComponentMesh(BoxMesh4);
    NuzzleChild2 = Box4;
    Box4->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(-30,20,0)));
    mComponents.push_back(Box4);
    Box3->AddChild(Box4);


    MeshGenerator::CuboidDescriptor cuboid_dscp5(Vector3(35.0,10.0,35.0));
    IMesh *BoxMesh5 = new IMeshGenerate(cuboid_dscp5);
    IComponentMesh *Box5 = new IComponentMesh(BoxMesh5);
    NuzzleChild3 = Box5;
    Box5->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,35,0)));
    mComponents.push_back(Box5);
    Box4->AddChild(Box5);



    /**

    MeshGenerator::CuboidDescriptor cuboid_dscp5(Vector3(50.0,10.0,50.0));
    IMesh *BoxMesh5 = new IMeshGenerate(cuboid_dscp5);
    IComponentMesh *Box5 = new IComponentMesh(BoxMesh5);
    Box5->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,50,-30)));
    NuzzleChild2 = Box5;
    mComponents.push_back(Box5);

    Box3->AddChild(Box5);


    //-----------------------------------------------------------------------//

    MeshGenerator::CuboidDescriptor cuboid_dscp6(Vector3(50.0,10.0,50.0));
    IMesh *BoxMesh6 = new IMeshGenerate(cuboid_dscp6);
    IComponentMesh *Box6 = new IComponentMesh(BoxMesh6);
    Box6->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,15,0)));
    NuzzleChild3 = Box6;
    mComponents.push_back(Box6);

    Box5->AddChild(Box6);

    /**/

}

void SceneEngineNuzzleGimbal::render(float FrameTime)
{

    glClearColor(0, 0, 0, 1);

    mCameraAnglePitch = IClamp(mCameraAnglePitch,scalar(-M_PI * 0.495),scalar(M_PI * 0.495));
    Matrix4 MRot;
    Vector3 ZBuffLength = Vector3::Z * mCameraZDistance;
    MRot = Matrix4::CreateRotationAxis( Vector3::X , -mCameraAnglePitch);
    MRot = Matrix4::CreateRotationAxis( Vector3::Y , -mCameraAngleYaw) * MRot;
    mCamera->SetEye(ZBuffLength * MRot);
    mCamera->BeginLookAt();

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, mWidth, mHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(mCamera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(mCamera->getCamera2().ViewMatrix());



    /**/

    if( !mGimbalStabilization->m_isExtremal )
    {
        glPushMatrix();
        glLineWidth(5);
        glColor3f(1,0,0);
        glBegin(GL_LINES);
        glVertex3fv(mGimbalStabilization->mGimbalConnectB->GetTransformHierarchy().GetPosition());
        glVertex3fv(NuzzleChoice->GetTransformMatrix().GetTranslation());
        glEnd();
        glPopMatrix();
    }






    /**

    glPushMatrix();
    glMultMatrixf(NuzzleChoice->GetTransformMatrixHierarchy());
    glPointSize(5);
    glColor3f(1,0,0);
    glBegin(GL_POINTS);
    glVertex3f(0,0,0);
    glEnd();
    glPopMatrix();


    glPushMatrix();
    glMultMatrixf(NuzzleChild2->GetTransformMatrixHierarchy());
    glPointSize(5);
    glColor3f(1,0,0);
    glBegin(GL_POINTS);
    glVertex3f(0,0,0);
    glEnd();
    glPopMatrix();



    glPushMatrix();
    glMultMatrixf(EyeCube->GetTransformMatrixHierarchy());
    glPointSize(5);
    glColor3f(1,0,0);
    glBegin(GL_POINTS);
    glVertex3f(0,0,0);
    glEnd();
    glPopMatrix();


    /**


    glPushMatrix();
    glMultMatrixf(Nuzzle->GetTransformMatrixHierarchy());
    glLineWidth(1);
    glColor3f(1,0,0);
    glBegin(GL_LINES);
    glVertex3f(30 - 5,20,0);
    glVertex3f(30 + 20,20,0);
    glEnd();
    glPopMatrix();


    glPushMatrix();
    glMultMatrixf(NuzzleChild1->GetTransformMatrixHierarchy());
    glLineWidth(1);
    glColor3f(0,1,0);
    glBegin(GL_LINES);
    glVertex3f(-30,20,5);
    glVertex3f(-30,20,5+20);
    glEnd();
    glPopMatrix();



    glPushMatrix();
    glMultMatrixf(NuzzleChild2->GetTransformMatrixHierarchy());
    glLineWidth(1);
    glColor3f(0,0,1);
    glBegin(GL_LINES);
    glVertex3f(0,35 - 10,0);
    glVertex3f(0,35 + 20,0);
    glEnd();
    glPopMatrix();


    glPushMatrix();
    glMultMatrixf(NuzzleChild3->GetTransformMatrixHierarchy());
    glTranslatef(-20,10,-20);
    glLineWidth(5);
    int size = 20;
    glColor3f(0,0,1);
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(size,0,0);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,size,0);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,size);
    glEnd();
    glPopMatrix();

    **/

    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {
        Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();

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


    //-----------------------------//

    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, 250, 250);

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
        if(i == 3) continue;

        Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();

        glPushMatrix();
        glMultMatrixf(MultMatrix);
        glColor3f(0.5,0.5,0.5);
        //OpenGLRender::DrawComponentMeshFill(static_cast<IComponentMesh*>(*it)->Modelmesh());
        //glColor3f(1,1,1);
        OpenGLRender::DrawComponentMeshLine(static_cast<IComponentMesh*>(*it)->Modelmesh());
        glPopMatrix();
    }

    glLoadIdentity();

   //=================================================//

   glFlush();
}

void SceneEngineNuzzleGimbal::update()
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

    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {
        (*it)->Updatee();
    }

    //--------------------------------------------------------------//

    mGimbalStabilization->Update(mGimbalStabilization->mGimbalRoot->GetTransfom().GetRotation(),
                                 NuzzleChoice->GetTransformHierarchy().GetPosition(),
                                 mGimbalStabilization->mGimbalConnectB->GetTransformHierarchy().GetPosition());;

    //--------------------------------------------------------------//


//    Quaternion QLook = Quaternion::LookAtLH(EyeCube->GetTransformMatrixHierarchy().GetTranslation(),
//                                            NuzzleChoice->GetTransformMatrixHierarchy().GetTranslation(),
//                                            Vector3::Y);

//    Transform T;
//    T.SetPosition(NuzzleChild2->GetTransformHierarchy().GetPosition());
//    T.SetBasis(QLook.GetRotMatrix());

//    EyeCube->SetTransform(T);

    //EyeCube->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,15,0)));
    //NuzzleChild1->RotateAroundWorldPoint(Vector3::Y , -Sensor.EulerAngle().y , Vector3(0,3.3,0));


    //--------------------------------------------------------------//


    /**

    QQL = Quaternion::LookAtLH(NuzzleChild2->GetTransformMatrixHierarchy().GetTranslation(),
                               NuzzleChoice->GetTransformMatrixHierarchy().GetTranslation(),
                               Vector3::Y).GetConjugate();




    Quaternion Q(Nuzzle->GetTransfom().GetBasis());
    Sensor.setQuat(QQL * Q);
    Sensor.Update();

    NuzzleChild1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,3.3,0)));
    NuzzleChild1->RotateAroundWorldPoint(Vector3::Y, -Sensor.EulerAngle().y, Vector3(0,3.3,0));

    Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y, AngleGimbal.y = Sensor.EulerAngle().y);
    QI.Normalize();

    Sensor.setQuat(QQL * (Q * QI.GetInverse()));
    Sensor.Update();

    NuzzleChild2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,0)));
    NuzzleChild2->RotateAroundWorldPoint(Vector3::X, AngleGimbal.x = -Sensor.EulerAngle().x, Vector3(0,5,0));

    **/


        //Quaternion Qk(NuzzleChild1->GetTransfom().GetBasis());

        //Quaternion Q2(NuzzleChild1->GetTransformMatrixHierarchy());
        //Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y,Sensor.EulerAngle().y);


    //    Quaternion Q2(NuzzleChild1->GetTransformMatrixHierarchy());
    //    Quaternion QI = Quaternion::FromAngleAxis(Vector3::Y,Sensor.EulerAngle().y);


    //    Sensor.setQuat(QQL * Q);// *  Q2 * (QI * QQL.GetInverse()).GetConjugate());
    //    Sensor.Update();

//        Sensor.setQuat(QI.GetConjugate() * QQL * Q);
//        Sensor.Update2();

//    NuzzleChild2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,5,0)));
//    NuzzleChild2->RotateAroundWorldPoint(Vector3::X, -Sensor.EulerAngle().x, Vector3(0,5,0));


//    NuzzleChild3->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,35,0)));
//    NuzzleChild3->RotateAroundWorldPoint(Vector3::Y , -Sensor.EulerAngle().y , Vector3(0,35,0));

   //-----------------------------------------------------------------------------//

}

void SceneEngineNuzzleGimbal::resize(float width, float height)
{
    mWidth  = width;
    mHeight = height;

    float aspect = mWidth / mHeight;
    float zNear  = 1.0;
    float zFar   = 1000;
    float fov    = 30.0;

    mCamera->InitPerspective( fov , aspect , zNear , zFar );

    if(mGizmoManipulator.get())
    {
        mGizmoManipulator->Resize( width , height );
    }
}

void SceneEngineNuzzleGimbal::mouseMove(float x, float y, int button)
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

void SceneEngineNuzzleGimbal::mousePress(float x, float y, int button)
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

                //-----------------------------------------------------------------------------//
                mGizmoManipulator->realase();
                //-----------------------------------------------------------------------------//
                IRay casting_ray = mGizmoManipulator->BuildRayMouse( data_mouse.mouseX , data_mouse.mouseY );

                /**
                mMengmentCollider.raycast( casting_ray , mGizmoManipulator.get() , 0x111 );
                //-----------------------------------------------------------------------------//
                if(!mGizmoManipulator.get()->mSelectedShape) return;

                unsigned int selected_index = -1;
                for(unsigned int i=0; i<mComponents.size(); ++i)
                {
                     if(mComponents[i]->Collid() == mGizmoManipulator.get()->mSelectedShape->GetBody())
                     {
                         selected_index = i;
                     }
                }
                **/


               // if(!mGizmoManipulator.get()->mSelectedShape) return;

                int selected_index = -1;
                float min = 10000000;
                for(int i=0; i<mComponents.size(); ++i)
                {
                    IRaycast raycast;

                    std::cout << mComponents[i]->GetAxisAlignedBoundingBox() << std::endl;


                     if(mComponents[i]->IntersectionRaycast(casting_ray,&raycast))
                     {
                         std::cout << min << "  " << raycast.distance << std::endl;
                         if(min > raycast.distance)
                         {
                             min = raycast.distance;
                             selected_index = i;
                         }
                     }
                }

                std::cout << selected_index << std::endl;

                if(selected_index == -1) return;
                if(selected_index == 1) return;
                if(selected_index == 2) return;

                std::cout << selected_index << std::endl;

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
        }
    }
}

void SceneEngineNuzzleGimbal::mouseReleasePress(float x, float y, int button)
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

void SceneEngineNuzzleGimbal::mouseWheel(float delta)
{
    mCameraZDistance += (delta * 0.02f);
    mCameraZDistance  = IMath::IClamp(IMath::IAbs(mCameraZDistance),5.f,1024.f);
}

void SceneEngineNuzzleGimbal::keyboard(int key)
{
    if(key == Qt::Key_W)
    {
        qDebug() << "key_W";

        Nuzzle->SetTransformMatrix( Nuzzle->GetTransformMatrix() * Matrix4::CreateTranslation(Vector3::Z));
    }

    if(key == Qt::Key_S)
    {
        qDebug() << "key_S";
        Nuzzle->SetTransformMatrix( Nuzzle->GetTransformMatrix() * Matrix4::CreateTranslation(-Vector3::Z));
    }

    if(key == Qt::Key_A)
    {
        qDebug() << "key_A";
        Nuzzle->SetTransformMatrix( Nuzzle->GetTransformMatrix() * Matrix4::CreateTranslation(-Vector3::X));
    }

    if(key == Qt::Key_D)
    {
        qDebug() << "key_D";
        Nuzzle->SetTransformMatrix( Nuzzle->GetTransformMatrix() * Matrix4::CreateTranslation(Vector3::X));
    }


    if(key == Qt::Key_Z)
    {
        qDebug() << "key_Z";
//        NuzzleChild1->SetTransformMatrix(NuzzleChild1->GetTransformMatrixHierarchy() *
//                                         Matrix4::CreateRotationAxis(/*NuzzleChild1->GetTransformMatrixHierarchy().GetRotMatrix() **/ Vector3::Y,-2)*
//                                         NuzzleChild1->GetTransformMatrixHierarchy().GetInverse());



//        NuzzleChild1->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(30,20,0)));
//        NuzzleChild1->RotateAroundWorldPoint(Vector3::X , ang2 += 0.01 , Vector3(30,0,0));


        //

//         Quaternion Q(Nuzzle->GetTransfom().GetBasis());
//         OrientationSensor Sensor(&Q);
//         Sensor.Update();

//         std::cout <<  Sensor.EulerAngle().y << std::endl;

//        Nuzzle->SetTransformMatrix(Nuzzle->GetTransformMatrix() *
//                                    Matrix4::CreateRotationAxis(Vector3::Y,2));




    }

    if(key == Qt::Key_X)
    {
        qDebug() << "key_X";
        Nuzzle->RotateLocal(Vector3::X,0.1);

        //auto T = Nuzzle->GetTransformHierarchy();
        //T.SetBasis( Matrix3::CreateRotationAxis(Vector3::Y , ang2 += 0.01f));
    }

    if(key == Qt::Key_C)
    {
        qDebug() << "key_C";
        Nuzzle->RotateLocal(Vector3::X,-0.1);
    }


    if(key == Qt::Key_V)
    {
        qDebug() << "key_V";
        Nuzzle->RotateLocal(Vector3::Z,0.1);
    }

    if(key == Qt::Key_B)
    {
        qDebug() << "key_B";
        Nuzzle->RotateLocal(Vector3::Z,-0.1);
    }


    if(key == Qt::Key_N)
    {
        qDebug() << "key_N";
        //NuzzleChild3->RotateLocal(Vector3::Z,-0.1);
        QQL = QQL * Quaternion::FromAngleAxis(Vector3::X,0.005f);
    }

    if(key == Qt::Key_M)
    {
        qDebug() << "key_M";
        //NuzzleChild3->RotateLocal(Vector3::Z,-0.1);
        QQL = QQL * Quaternion::FromAngleAxis(Vector3::X,-0.005f);
    }


    if(key == Qt::Key_K)
    {
        qDebug() << "key_B";
        //NuzzleChild3->RotateLocal(Vector3::Z,-0.1);
        QQL = QQL * Quaternion::FromAngleAxis(Vector3::X,-0.005f);
    }


    if(key == Qt::Key_M)
    {
        qDebug() << "key_B";
        //NuzzleChild3->RotateLocal(Vector3::Z,-0.1);
        QQL = QQL * Quaternion::FromAngleAxis(Vector3::X,-0.005f);
    }

//    if(key == Qt::Key_N)
//    {
//        qDebug() << "key_N";
//        Quaternion Q(Nuzzle->GetTransfom().GetBasis());
//        OrientationSensor Sensor(&Q);
//        Sensor.Update();

//        Transform T2(NuzzleChild2->GetTransformMatrix().GetTranslation(),
//                    Matrix4::CreateRotationAxis(Vector3::Z , Sensor.EulerAngle().z));

//        std::cout << Sensor.EulerAngle().z << std::endl;

//        NuzzleChild2->SetTransform(T2);
//    }
}

void SceneEngineNuzzleGimbal::destroy()
{

}

IGizmoManipulator *SceneEngineNuzzleGimbal::gizmoManipulator() const
{
   return mGizmoManipulator.get();
}
