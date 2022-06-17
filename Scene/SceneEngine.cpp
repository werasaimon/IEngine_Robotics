#include "SceneEngine.h"
#include <Qt>
#include <QFile>

#include "OpenGL/geometry_opengl.h"
#include "OpenGL/OpenGLRender.h"

namespace
{

    bool CopyFileFromToResources( QFile &mFile, const char* fileName )
    {
        QFile FileRead(fileName);

        bool success = true;
        success &=  FileRead.open( QFile::ReadOnly );
        success &= mFile.open( QFile::WriteOnly | QFile::Truncate );
        success &= mFile.write(  FileRead.readAll() ) >= 0;
        FileRead.close();
        mFile.close();

        return success;
    }


    bool isShaderRender = true;
}


SceneEngine::SceneEngine()
{
    initializeOpenGLFunctions();
}

SceneEngine::~SceneEngine()
{

}

void SceneEngine::initCamera()
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


    mCameras.push_back(camera);
    mComponents.push_back(camera);

}


void SceneEngine::initShader()
{
    //------------------------------------------------------------------------------------//

    // Compile vertex shader
    if (!mProgramShader.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/object.vsh"))
        qDebug() << "Error";

    // Compile fragment shader
    if (!mProgramShader.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/object.fsh"))
        qDebug() << "Error";

    // Link shader pipeline
    if (!mProgramShader.link())
        qDebug() << "Error";

    //------------------------------------------------------------------------------------//

    if(! mProgramDepth.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/depth.vsh"))
       qDebug() << "Error";

    if(! mProgramDepth.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/depth.fsh"))
        qDebug() << "Error";

    if(! mProgramDepth.link())
        qDebug() << "Error";

    //------------------------------------------------------------------------------------//
}

void SceneEngine::initLights()
{
//          mLights.append(new ILight(ILight::LightType::Direct));
//          mLights.last()->setPosition(Vector4(20.0f, 15.0f, 55.0f, 1.0f));
//          mLights.last()->setDirection(Vector4(-1.0f, -1.0f, -1.0f, 0.0f));

          IComponentLight  *LightComp0 = new IComponentLight(ILight::LightType::Direct);
          LightComp0->light()->SetPosition(Vector4(5.f, 10.f, 25.f, 1.f));
          LightComp0->light()->SetDirection(Vector4(-1.0f, -1.0f, -1.0f, 0.0f));
          LightComp0->CreateShadowFrameBuffer(1024,1024);
          LightComp0->SetVolumeShadow(250);
          mLights.push_back(LightComp0);

          LightComp0->SetTransformMatrix(Matrix4::CreateTranslation(LightComp0->light()->Position().GetXYZ()));
          mComponents.push_back(LightComp0);


//          IComponentLight  *LightComp1 = new IComponentLight(ILight::LightType::Spot);
//          LightComp1->light()->setPosition(Vector4(light_pos, 1.0f));
//          LightComp1->light()->setDirection(Vector4(-1.0f, -1.0f, -1.0f, 0.0f));
//          LightComp1->light()->setDiffuseColor(Vector3(1.0f, 1.0f, 1.0f));
//          LightComp1->light()->setCutoff(20.0f / 180.0f * static_cast<float>(M_PI));
//          mLights.push_back(LightComp1);


//          // дополнительное освещение
//          mLights.append(new ILight(ILight::LightType::Spot));
//          mLights.last()->setPosition(Vector4(20.0f, 10.0f, 5.0f, 1.0f));
//          mLights.last()->setDirection(Vector4(-1.0f, -1.0f, -1.0f, 0.0f));
//          mLights.last()->setDiffuseColor(Vector3(0.0f, 1.0f, 0.0f));
//          mLights.last()->setCutoff(15.0f / 180.0f * static_cast<float>(M_PI));
}

void SceneEngine::initialization()
{

    glClearColor(0.f,0.f,0.0f,1.f);

    initLights();

   //==========================//

    m_ShadowTextureSlot = GL_TEXTURE2;  // 0 и 1 текстурные слоты уже заняты

    mSceneDscriptor.m_ShadowPointCloudFilteringQuality = 1.5f; // влияет на производительность, можно параметризировать
    mSceneDscriptor.m_ShadowMapSize = 1024;// влияет на производительность, можно параметризировать
    mSceneDscriptor.m_MainLightPower = 0.9f; // мощность основного освещения, можно параметризировать
    mSceneDscriptor.m_IsDrawShadow = true;
    mSceneDscriptor.m_IsDrawLines = true;
    mSceneDscriptor.m_IsDrawShader = true;

    //===========================//

    NullAllKey();
    initShader();
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

    mCamera->SetEye(Vector3(0,0,mCameraZDistance=425));
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
    //============================================================//




    IMaterial *mtl = new IMaterial;
    //mtl->setDiffuseMap(":/datafiles/kirpich.jpg");
    //mtl->setNormalMap(":/datafiles/kirpich_normal.jpg");
    mtl->setShines(10.0f);
    mtl->setDiffuseColor(Vector3(0.5f, 0.5f, 0.5f));
    mtl->setAmbienceColor(Vector3(1.0f, 1.0f, 1.0f));
    mtl->setSpecularColor(Vector3(1.0f, 1.0f, 1.0f));



    IMaterial *mtl2 = new IMaterial;
    mtl2->setDiffuseMap(":/datafiles/megalodon.png");
   // mtl2->setNormalMap(":/datafiles/kirpich_normal.jpg");
    mtl2->setShines(10.0f);
    mtl2->setDiffuseColor(Vector3(0.1f, 0.1f, 0.1f));
    mtl2->setAmbienceColor(Vector3(1.0f, 1.0f, 1.0f));
    mtl2->setSpecularColor(Vector3(1.0f, 1.0f, 1.0f));

    //--------------------------------------//

    const char fileName[] = "megalodon.3DS";
    QFile mFilePro(fileName);
    CopyFileFromToResources( mFilePro , ":/datafiles/megalodon.3DS" );


    for(int i=0; i < 1; ++i)
    {
        IMesh *StoneMesh = new IMeshLoader(fileName);
        IComponentMesh *Stone = new IComponentMesh(StoneMesh);
        mComponents.push_back(Megalodon=Stone);

        Stone->SetTransformMatrix(Matrix4::CreateScale(0.1,0.1,0.1) * Matrix4::CreateRotationAxis(Vector3::Y,-M_PI/2.f) );
        Stone->SetMaterial(*mtl2);
    }


//    for (int i=0; i<11; ++i)
//    {
//        MeshGenerator::CuboidDescriptor cuboid_dscp(Vector3(2.f));
//        IMesh *BoxMesh = new IMeshGenerate(cuboid_dscp);
//        IComponentMesh *Box = new IComponentMesh(BoxMesh);
//        Box->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,4.0*i,0)));
//        mComponents.push_back(Box);
//        mProxyMaterials[Box] = mtl;
//    }


    MeshGenerator::CuboidDescriptor cuboid_dscp2(Vector3(50.0,10.0,50.0));
    IMesh *BoxMesh2 = new IMeshGenerate(cuboid_dscp2);
    IComponentMesh *Box2 = new IComponentMesh(BoxMesh2);
    Box2->SetTransformMatrix(Matrix4::CreateTranslation(Vector3(0,-13,0)));
    mComponents.push_back(Box2);

}



void SceneEngine::render(float FrameTime)
{

    mCameraAnglePitch = IClamp(mCameraAnglePitch,scalar(-M_PI * 0.495),scalar(M_PI * 0.495));
    Matrix4 MRot;
    Vector3 ZBuffLength = Vector3::Z * mCameraZDistance;
    MRot = Matrix4::CreateRotationAxis( Vector3::X , -mCameraAnglePitch);
    MRot = Matrix4::CreateRotationAxis( Vector3::Y , -mCameraAngleYaw) * MRot;
    mCamera->SetEye(ZBuffLength * MRot);


    Megalodon->SetTransformMatrix( Megalodon->GetTransformMatrix() * Matrix4::CreateRotationAxis(Vector3::Y,0.02) );


    mCamera->BeginLookAt();


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(mSceneDscriptor.m_IsDrawLines)
    {
        renderLines(mCamera,mWidth,mHeight);
    }

    //--------------------------------------------------------------------------//

    if(mSceneDscriptor.m_IsDrawShader)
    {
      renderShadows();
      renderShader(mCamera,mWidth,mHeight,&mProgramShader,true);
    }

    //--------------------------------------------------------------------------//

    renderGizmo(mCamera,mWidth,mHeight);

    //--------------------------------------------------------------------------//

}



void SceneEngine::renderShadows()
{
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glEnable(GL_DEPTH_TEST);


    // отрисовка во фреймбуфер
    if(mSceneDscriptor.m_IsDrawShadow) // отрисовка теней
    {

        for(unsigned int i=0; i<mLights.size();++i)
        {
            auto fb = mLights.at(i)->ShadowFrameBuffer();

            fb->bind();

            glViewport(0, 0, fb->width(), fb->height());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mProgramDepth.bind();
            mProgramDepth.UniformValue("u_ProjectionLightMatrix", mLights.at(i)->light()->CameraShadows().ProjectionMatrix());
            mProgramDepth.UniformValue("u_ShadowLightMatrix", mLights.at(i)->light()->CameraShadows().ViewMatrix());


            for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
            {
                Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();
                if(!mSceneDscriptor.m_IsSimulateDynamics)
                {
                    auto iter_proxy = mProxyColliderConnects.find((*it));
                    if(iter_proxy != mProxyColliderConnects.end())
                    {
                        MultMatrix = iter_proxy->second->GetWorldTransform().GetTransformMatrix();
                    }
                }


                if((*it)->type() == TYPE_COMPONENT::MESH_MODEL)
                {
                    mProgramDepth.UniformValue("u_ModelMatrix"  , MultMatrix);

                    IComponentMesh* cmesh = static_cast<IComponentMesh*>(*it);
                    geometry_opengl *geometry_engine_render = new geometry_opengl(cmesh->Modelmesh());
                    geometry_engine_render->drawCubeGeometry(&mProgramShader);
                    delete geometry_engine_render;
                }

            }

            mProgramDepth.release();
            fb->release();

            //glFlush();
        }
    }
}

void SceneEngine::renderGizmo(IComponentCamera *_camera, float _width , float _height)
{
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, _width, _height);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(_camera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(_camera->getCamera2().ViewMatrix());

    if (mGizmoManipulator->GetGizmo())
    {
        mGizmoManipulator->GetGizmo()->SetCameraMatrix( _camera->getCamera2().ViewMatrix() ,
                                                        _camera->getCamera2().ProjectionMatrix() );

        if(mGizmoManipulator->mSelectedIndexID >= 0)
        {
            glLineWidth(5);
            mGizmoManipulator->GetGizmo()->Draw();
            glLineWidth(1);
        }
    }

   glLoadIdentity();
}

void SceneEngine::renderLines(IComponentCamera *_camera , float _width , float _height)
{
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, _width, _height);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(_camera->getCamera2().ProjectionMatrix());

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(_camera->getCamera2().ViewMatrix());

    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {

        //-------------------------------------------------------------//
        Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();
        if(!mSceneDscriptor.m_IsSimulateDynamics)
        {
            auto iter_proxy = mProxyColliderConnects.find((*it));
            if(iter_proxy != mProxyColliderConnects.end())
            {
                MultMatrix = iter_proxy->second->GetWorldTransform().GetTransformMatrix();
            }
        }
        //-------------------------------------------------------------//
        if((*it)->type() == TYPE_COMPONENT::MESH_MODEL)
        {
            glPushMatrix();
            glMultMatrixf(MultMatrix);
            glColor3f(1,1,1);
            OpenGLRender::DrawComponentMeshLineee(static_cast<IComponentMesh*>(*it)->Modelmesh());
            glPopMatrix();

        }
        else if((*it)->type() == TYPE_COMPONENT::LIGHT)
        {
            glColor3f(1,1,1);
            IComponentLight* clight = static_cast<IComponentLight*>(*it);
            OpenGLRender::DrawComponentAABB(clight->GetAxisAlignedBoundingBox(),MultMatrix);
        }
        else if((*it)->type() == TYPE_COMPONENT::CAMERA)
        {

            glColor3f(1,0,1);
            IComponentCamera* camera = static_cast<IComponentCamera*>(*it);
            OpenGLRender::DrawComponentAABB(camera->GetAxisAlignedBoundingBox(),MultMatrix);

            OpenGLRender::DrawLine(camera->GetTransfom().GetPosition(),
                                   camera->GetTransfom().GetPosition() -
                                   Vector3::Z * camera->GetTransfom().GetBasis().GetTranspose() * camera->getCamera2().mFar);


            std::vector<ILineSegment3D> lines = camera->getCamera2().GetFrustumLines();
            for(auto it : lines) OpenGLRender::DrawLine( it.GetEndpoint0() , it.GetEndpoint1());

        }
        //-------------------------------------------------------------//
    }

    glLoadIdentity();
    glFlush();
}

void SceneEngine::renderShader(IComponentCamera *_camera, float _width, float _height, GLShaderProgram *_ProgramShader , bool is_look_at)
{
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, _width , _height);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    //----------------------------------------------------------------------------//

    _ProgramShader->bind();

    _ProgramShader->UniformValue("u_ProjectionMatrix"  , _camera->getCamera2().ProjectionMatrix());
    _ProgramShader->UniformValue("u_ViewMatrix"        , (is_look_at)? _camera->getCamera2().ViewMatrix() :
                                                                       _camera->getCamera2().ViewMatrix().GetInverse());

    _ProgramShader->UniformValue("u_eyePosition" , _camera->getCamera2().GetEye());

    _ProgramShader->UniformValue("u_MainLightPower",mSceneDscriptor.m_MainLightPower);
    _ProgramShader->setUniformValue("u_IsDrawShadow", mSceneDscriptor.m_IsDrawShadow);
    _ProgramShader->setUniformValue("u_ShadowMapSize", static_cast<float>(mSceneDscriptor.m_ShadowMapSize));
    _ProgramShader->setUniformValue("u_ShadowPointCloudFilteringQuality", mSceneDscriptor.m_ShadowPointCloudFilteringQuality);


    _ProgramShader->setUniformValue("u_CountLights", int(mLights.size()));


    //-------------------------------------------------------------//

    for(unsigned int i = 0; i < mLights.size(); i++)
    {
        _ProgramShader->setUniformValue(("u_ShadowMapTexture["+std::to_string(i)+"]").c_str(), (m_ShadowTextureSlot+i) - GL_TEXTURE0);

        _ProgramShader->UniformValue(("u_ShadowProperty["+std::to_string(i)+"].ProjectionLightMatrix").c_str(), mLights.at(i)->light()->CameraShadows().ProjectionMatrix());
        _ProgramShader->UniformValue(("u_ShadowProperty["+std::to_string(i)+"].ShadowLightMatrix").c_str(), mLights.at(i)->light()->CameraShadows().ViewMatrix());

        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].AmbienceColor").c_str(), mLights.at(i)->light()->AmbienceColor());
        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].DiffuseColor").c_str(), mLights.at(i)->light()->DiffuseColor());
        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].SpecularColor").c_str(), mLights.at(i)->light()->SpecularColor());
        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].ReflectionColor").c_str(), mLights.at(i)->light()->ReflectionColor());
        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].Position").c_str(), mLights.at(i)->light()->Position());
        _ProgramShader->UniformValue(("u_LightProperty["+std::to_string(i)+"].Direction").c_str(), mLights.at(i)->light()->Direction());
        _ProgramShader->setUniformValue(("u_LightProperty["+std::to_string(i)+"].Cutoff").c_str(), mLights.at(i)->light()->Cutoff());
        _ProgramShader->setUniformValue(("u_LightProperty["+std::to_string(i)+"].Power").c_str(), mLights.at(i)->light()->Power());
        _ProgramShader->setUniformValue(("u_LightProperty["+std::to_string(i)+"].Type").c_str(), int(mLights.at(i)->light()->Type()));


        GLuint texture = mLights.at(i)->ShadowFrameBuffer()->texture();
        glActiveTexture(m_ShadowTextureSlot + i);
        glBindTexture(GL_TEXTURE_2D, texture);
    }


    for(auto it=mComponents.begin(); it<mComponents.end(); ++it)
    {

       //-------------------------------------------------------------//
        Matrix4 MultMatrix = (*it)->GetTransformMatrixHierarchy();
        if(!mSceneDscriptor.m_IsSimulateDynamics)
        {
            auto iter_proxy = mProxyColliderConnects.find((*it));
            if(iter_proxy != mProxyColliderConnects.end())
            {
                MultMatrix = iter_proxy->second->GetWorldTransform().GetTransformMatrix();
            }
        }

        //-------------------------------------------------------------//
        if((*it)->type() == TYPE_COMPONENT::MESH_MODEL)
        {
            glPushMatrix();
            glMultMatrixf(MultMatrix);

            _ProgramShader->UniformValue("u_ModelMatrix"  , MultMatrix);
            _ProgramShader->UniformValue("u_NormalMatrix"  , MultMatrix.GetInverse().GetTranspose());

            IMaterial materia = (static_cast<IComponentMesh*>(*it)->Material());
            IMaterial *material = &materia;


            _ProgramShader->UniformValue("u_MaterialProperty.DiffuseColor", material->DiffuseColor());
            _ProgramShader->UniformValue("u_MaterialProperty.AmbienceColor", material->AmbienceColor());
            _ProgramShader->UniformValue("u_MaterialProperty.SpecularColor", material->SpecularColor());
            _ProgramShader->UniformValue("u_MaterialProperty.Shines", material->Shines());
            _ProgramShader->UniformValue("u_IsUseDiffuseMap", material->isUseDiffuseMap());
            _ProgramShader->UniformValue("u_IsUseNormalMap",  material->isUseNormalMap());

            if( material->isUseDiffuseMap())
            {
                material->DiffuseMapTexture()->bind(0);
                _ProgramShader->setUniformValue("u_DiffuseMapTexture", 0);
            }


            if( material->isUseNormalMap())
            {
                material->NormalMapTexture()->bind(1);
                _ProgramShader->setUniformValue("u_NormalMapTexture", 1);
            }


            IComponentMesh* cmesh = static_cast<IComponentMesh*>(*it);
            geometry_opengl *geometry_engine_render = new geometry_opengl(cmesh->Modelmesh());
            geometry_engine_render->drawCubeGeometry(&mProgramShader);
            delete geometry_engine_render;

            glPopMatrix();
        }
    }

    _ProgramShader->release();
    //--------------------------------------------------//

    glLoadIdentity();
    glFlush();
}

void SceneEngine::update()
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

    if(!mSceneDscriptor.m_IsSimulateDynamics)
    {
        mDynamicsWorld->UpdateFixedTimeStep(mTimeStep);
    }
    //------------------------------------------------------------------------------------------------------------------//

}



void SceneEngine::resize(float width, float height)
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

void SceneEngine::mouseMove(float x, float y, int button)
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

void SceneEngine::mousePress(float x, float y, int button)
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

                if(!mKeys[Qt::Key_I]) mSelectedIndexIds.clear();

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

                unsigned int selected_index = -1;
                float min = 100000000;
                for(unsigned int i=0; i<mComponents.size(); ++i)
                {
                    IRaycast raycast;
                     if(mComponents[i]->IntersectionRaycast(casting_ray,&raycast))
                     {
                         if(min > raycast.distance)
                         {
                             min = raycast.distance;
                             selected_index = i;
                         }
                     }
                }

                if(selected_index == -1) return;

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

void SceneEngine::mouseReleasePress(float x, float y, int button)
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

void SceneEngine::mouseWheel(float delta)
{
    mCameraZDistance += (delta * 0.02f);
    mCameraZDistance  = IMath::IClamp(IMath::IAbs(mCameraZDistance),5.f,1024.f);
}

void SceneEngine::keyboard(int key)
{
    if(key == Qt::Key_T)
    {
        for (int i = 0; i < 100; ++i)
        {
            float rand_x = 200 - rand()%400;
            float rand_y = 200 - rand()%400;
            float rand_z = 200 - rand()%400;

            MeshGenerator::CuboidDescriptor cuboid_desript(Vector3(2,2,2));
            IMesh *mesh = new IMeshGenerate(cuboid_desript);
            IComponentMesh *mesh_model = new IComponentMesh(mesh);
            mesh_model->TranslateWorld(Vector3(rand_x,rand_y,rand_z));
            mComponents.push_back(mesh_model);
        }
    }


    if(key == Qt::Key_Y)
    {
        mSceneDscriptor.m_IsSimulateDynamics = !mSceneDscriptor.m_IsSimulateDynamics;
    }

    if(key == Qt::Key_1)
    {
        mSceneDscriptor.m_IsDrawLines = !mSceneDscriptor.m_IsDrawLines;
    }

    if(key == Qt::Key_2)
    {
        mSceneDscriptor.m_IsDrawShadow = !mSceneDscriptor.m_IsDrawShadow;
    }

    if(key == Qt::Key_3)
    {
       mSceneDscriptor.m_IsDrawShader = !mSceneDscriptor.m_IsDrawShader;
    }


    //-------------------------//

    if( key == Qt::Key_V )
    {
        mCamera->DollyZoomFOV(-0.1);
        mGizmoManipulator->DisplayScale(mCamera->getCamera2().AngleOfViewSize() * 2.0);

    }


    if( key == Qt::Key_J )
    {
        qDebug() << "Key_J";
        isShaderRender = !isShaderRender;
    }
}

void SceneEngine::destroy()
{

}

IGizmoManipulator *SceneEngine::gizmoManipulator() const
{
   return mGizmoManipulator.get();
}
