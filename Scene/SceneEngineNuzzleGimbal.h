#ifndef SCENEENGINENUZZLEGIMBAL_H
#define SCENEENGINENUZZLEGIMBAL_H

#include "SceneMain.h"

#include "Scene/Sensors/OrientationSensor.h"
#include "Engine/engine.hpp"

#include "Engine/ICommon/IMemory/IList.h"
#include "Engine/ICommon/IMemory/IMem.h"
#include "Engine/ICommon/IMemory/IStack.h"
#include "Engine/IComponent/IComponent.h"
#include "IGizmoManipulator.h"
#include "EngineComponent/IEngineComponent.hpp"
#include "Shader/Shader.h"

#include "IEngineGimbalStabilization.h"

using namespace IMath;
using namespace IEngine;






class SceneEngineNuzzleGimbal : public SceneMain
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

    Quaternion QQL;

    IComponentMesh *Nuzzle;
    IComponentMesh *NuzzleChild1;
    IComponentMesh *NuzzleChild2;
    IComponentMesh *NuzzleChild3;

    IComponentMesh *NuzzleChoice;
    IComponentMesh *EyeCube;

    //---------------- Manipulator -------------------//
    std::auto_ptr<IGizmoManipulator> mGizmoManipulator;
    std::set<int>                    mSelectedIndexIds;


    //---------------------------------------------//

    Vector2 AngleGimbal;
    IEngineGimbalStabilization *mGimbalStabilization;

public:
    SceneEngineNuzzleGimbal();

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

    IGizmoManipulator *gizmoManipulator() const;
    const OrientationSensor &getSensor() const;
    const Vector2 &getAngleGimbal() const;
    IEngineGimbalStabilization *gimbalStabilization() const;
};
#endif // SCENEENGINENUZZLEGIMBAL_H
