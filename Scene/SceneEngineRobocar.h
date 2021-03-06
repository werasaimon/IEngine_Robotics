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

#include "IEngineGimbalStabilization.h"

#include "IEngineFactory.h"

using namespace IMath;
using namespace IEngine;



class SceneEngineRobocar;
class FactoryMethod;

//-----------------------------------------//

// Структура данных для ROBO_CAR
// Структура кнотроля робокара
struct DataPacketRemote
{
    int  speed_PWM_X=0; // скорость вращения на ШИМ _А
    int  speed_PWM_Y=0; // скорость вращения на ШИМ _В
    int  turn = 0;
    bool is_null_pos;

    float kp;
    float kd;
    float ki;
    float kf;
    float kt;

    DataPacketRemote( int _x=0, int _y=0, bool _is_null_pos=false)
    :speed_PWM_X(_x),
     speed_PWM_Y(_y),
     is_null_pos(_is_null_pos)
    {

    }

};


//-----------------------------------------//


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

    friend class FactoryMethod;
    friend class IEngineFactoryRobot;

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

    IEngineGimbalStabilization *mGimbalStabilization;

    ////---------------------////

    IComponentMesh *NuzzleChoice;


    //----------------------------//

    //---------------- Manipulator -------------------//
    std::auto_ptr<IGizmoManipulator> mGizmoManipulator;
    std::set<int>                    mSelectedIndexIds;

    bool isSelectedStatus;

    //----------------------------//

    IEngineFactoryRobot *mFactoryMethod;


    //---------------------------//


public:

    DataPacketRemote data_trransmission;

    Vector3 m_AngleGimbal;

    SceneDscriptorr  mSceneDscriptor;
    VehicleRobotCar *mRoboCar;


    bool m_IsFix;
    bool m_IsDynamic_LQR;
    int num;
    Vector3 m_PointS;
    Vector3 m_EndPoint;
    std::vector<Vector3> mPoints;
    float speed_point = 1.0;
    float angle_yaw;


    bool m_IsTrackingMove;
    std::vector<Vector3> mTrackerPoints;
    Vector3 m_pickPoint;


    float Max_Length;


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


    void Stop();
};


//-----------------------------------------//



class FactoryMethod
{

    public:

        FactoryMethod( SceneEngineRobocar *_scene) :
        mScene(_scene)
        {

        }


        VehicleRobotCar *CretaeRobotCar();

        SceneEngineRobocar *mScene;
};


//-----------------------------------------//

#endif // SCENEENGINEROBOCAR_H
