#ifndef SCENEENGINE_H
#define SCENEENGINE_H

#include <set>
#include <QOpenGLFramebufferObject>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include "SceneMain.h"
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



struct SceneDscriptor
{
    float m_MainLightPower;
    float m_ShadowPointCloudFilteringQuality; // качество теней, точность PointCloudFiltering
    int   m_ShadowMapSize;                      // качество теней, размер карты теней;

    bool  m_IsDrawShadow; // отрисовка теней
    bool  m_IsDrawLines;
    bool  m_IsDrawShader;
    bool  m_IsSimulateDynamics;

};



class SceneEngine : public SceneMain , protected QOpenGLFunctions
{

private:

    //=======================//

    unsigned int m_ShadowTextureSlot;


    SceneDscriptor mSceneDscriptor;


    //=======================//

    struct Mouse
    {
        float mouseOldX;
        float mouseOldY;

        float mouseX;
        float mouseY;

    }  data_mouse;

    float mWidth;
    float mHeight;

    //---------------- Manipulator -------------------//
    std::auto_ptr<IGizmoManipulator> mGizmoManipulator;
    std::set<int>                    mSelectedIndexIds;


    //-----------------------------//

    /// Shaders shines
    GLShaderProgram mProgramShader;
    GLShaderProgram mProgramDepth;


    //-----------------------------//

    Matrix4 mGizmoMatrix;

    //-----------------------------//
    float mCameraZDistance;
    float mCameraAngleYaw;
    float mCameraAnglePitch;

    //-----------------------------//

    IComponentCamera *mCamera;


    std::vector<IComponentCamera*> mCameras;
    std::vector<IComponentLight*>  mLights;

    // initialize container
    std::vector<IComponentAbstract*>            mComponents;
    std::map<IComponentAbstract*, IProxyShape*> mProxyColliderConnects;


    float mTimeStep;
    IDynamicsWorld *mDynamicsWorld;

    //-----------------------------//

    void  AddPhysicsProxyInModel( IComponentAbstract *model , IProxyShape* shape_colliders )
    {
        mProxyColliderConnects.insert( std::make_pair(model,shape_colliders));
    }


    IComponentMesh *Megalodon;


public:
     SceneEngine();
    ~SceneEngine();


    void initCamera();
    void initShader();
    void initLights();

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

private:

    void renderShadows();
    void renderGizmo(IComponentCamera *_camera , float _width , float _height);
    void renderLines(IComponentCamera *_camera , float _width , float _height);
    void renderShader(IComponentCamera *_camera , float _width , float _height , GLShaderProgram* _ProgramShader, bool is_look_at);

};

#endif // SCENEENGINE_H
