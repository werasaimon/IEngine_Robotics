#ifndef SCENEENGINEROBOCAR_H
#define SCENEENGINEROBOCAR_H

#include "SceneMain.h"

#include "Scene/Sensors/OrientationSensor.h"
#include "Scene/Sensors/EncoderSensor.h"
#include "Engine/engine.hpp"

#include "Engine/ICommon/IMemory/IList.h"
#include "Engine/ICommon/IMemory/IMem.h"
#include "Engine/ICommon/IMemory/IStack.h"
#include "Engine/IComponent/IComponent.h"
#include "IGizmoManipulator.h"
#include "EngineComponent/IEngineComponent.hpp"
#include "Shader/Shader.h"

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

#include "Sensors/EncoderSensor.h"
#include "Sensors/OrientationSensor.h"
#include "Sensors/VehicleRobotCar.h"

using namespace IMath;
using namespace IEngine;





struct SceneDscriptorr
{
//    float m_MainLightPower;
//    float m_ShadowPointCloudFilteringQuality; // качество теней, точность PointCloudFiltering
//    int   m_ShadowMapSize;                      // качество теней, размер карты теней;

//    bool  m_IsDrawShadow; // отрисовка теней
//    bool  m_IsDrawLines;
//    bool  m_IsDrawShader;
    bool  m_IsSimulateDynamics;

};



class SceneEngineRobocar : public SceneMain
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

    void AddPhysicsProxyInModel( IComponentAbstract *model , IProxyShape* shape_colliders )
    {
        mProxyColliderConnects.insert( std::make_pair(model,shape_colliders));
    }

    //----------------------------//

public:


    SceneDscriptorr mSceneDscriptor;
    VehicleRobotCar *mRoboCar;



    bool m_IsDynamic_LQR;
    int num;
    Vector3 m_PointS;
    Vector3 m_EndPoint;
    std::vector<Vector3> mPoints;
    float speed_point = 1.0;


public:
    SceneEngineRobocar();

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
};

#endif // SCENEENGINEROBOCAR_H
