#ifndef SCENEENGINEROBOCAR_H
#define SCENEENGINEROBOCAR_H

#include "SceneMain.h"

#include "Scene/Sensors/ISensorOrientation.h"
#include "Scene/Sensors/ISensorEncoder.h"
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

#include "Sensors/ISensorOrientation.h"
#include "Sensors/ISensorEncoder.h"
#include "Sensors/ISensorLIDAR.h"
#include "Robotics/VehicleRobotCar.h"

#include "IEngineGimbalStabilization.h"
#include "IEngineFactory.h"

#include "../fontprovider.h"
#include "../Engine/IAlgorithm/IScanColsingPath.h"



using namespace IMath;
using namespace IEngine;



class SceneEngineRobocar;
class FactoryMethod;



class IRaycastInfoOutput : public IRaycastCallback
{
    private:


       // -------------------- Attributes -------------------- //

       ///! is bind caculate LASER
       bool  mIsBind;

       unsigned short mBitMask;

       ///! Fraction distance of the hit point between point1 and point2 of the ray
       ///! The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
       scalar mDistance;

       ///! Maximun distance to object
       scalar mMaximumDistance;

       ///! Raycast distance to object
       IRaycastInfo mRaycastInfo;

    public:

        //-------------------- Methods --------------------//

        IRaycastInfoOutput(unsigned short BitMask = 0x0001) :
         mIsBind(true),
         mBitMask(BitMask),
         mDistance(1),
         mMaximumDistance(1)
        {

        }

        ~IRaycastInfoOutput()
        {

        }

        void Bind(const scalar& _maximunm_distance)
        {
            mIsBind = true;
            mDistance = _maximunm_distance;
            mMaximumDistance = _maximunm_distance;
        }

        /// This method will be called for each ProxyShape that is hit by the
        /// ray. You cannot make any assumptions about the order of the
        /// calls. You should use the return value to control the continuation
        /// of the ray. The returned value is the next maxFraction value to use.
        /// If you return a fraction of 0.0, it means that the raycast should
        /// terminate. If you return a fraction of 1.0, it indicates that the
        /// ray is not clipped and the ray cast should continue as if no hit
        /// occurred. If you return the fraction in the parameter (hitFraction
        /// value in the IRaycastInfo object), the current ray will be clipped
        /// to this fraction in the next queries. If you return -1.0, it will
        /// ignore this ProxyShape and continue the ray cast.
        /**
         * @param IRaycastInfo Information about the raycast hit
         * @return Value that controls the continuation of the ray after a hit
         */
        scalar notifyRaycastHit(const IRaycastInfo& IRaycastInfo)
        {
            if(IRaycastInfo.proxyShape->GetCollisionCategoryBits() == mBitMask)
            {
                /// Minimal distance [sorting]
                if(mDistance > IRaycastInfo.hitFraction || mIsBind)
                {
                    mDistance = (IRaycastInfo.hitFraction <= mMaximumDistance) ? IRaycastInfo.hitFraction : mMaximumDistance;
                    mIsBind = false;
                    mRaycastInfo.worldPoint = IRaycastInfo.worldPoint;
                    mRaycastInfo.worldNormal = IRaycastInfo.worldNormal;
                    mRaycastInfo.hitFraction = IRaycastInfo.hitFraction;
                    mRaycastInfo.meshSubpart = IRaycastInfo.meshSubpart;
                    mRaycastInfo.triangleIndex = IRaycastInfo.triangleIndex;
                    mRaycastInfo.body = IRaycastInfo.body;
                    mRaycastInfo.proxyShape = IRaycastInfo.proxyShape;
                }

            }

            return IRaycastInfo.hitFraction;
        }

        ///
        /// \brief distance
        /// \return Minimal distance Lidar
        ///
        float getDistance() const
        {
            return mDistance;
        }

        const IRaycastInfo &raycastInfo() const
        {
            return mRaycastInfo;
        }

        bool isBind() const
        {
            return mIsBind;
        }
};


//-----------------------------------------//


class CallbeckSupprtModelee : public CallbeckSupprtModel
{

        std::vector<Vector3> mPoints;

    public:

        CallbeckSupprtModelee(std::vector<Vector3> _points)
        : mPoints(_points)
        {
        }

        Vector3 SupportPoint(const Vector3& AxisDirection) const override
        {
            unsigned int index = 0;
            scalar max = (mPoints[0].Dot(AxisDirection));
            for (unsigned int i = 1; i < mPoints.size(); i++)
            {
                scalar d = (mPoints[i].Dot(AxisDirection));
                if (d > max)
                {
                    max = d;
                    index = i;
                }
            }
            return mPoints[index];
        }

        virtual const Matrix4 GetTransform() const override
        {
            return Matrix4::IDENTITY;
        }
};



class CallbeckSupprtCollide : public CallbeckSupprtModel
{

       IProxyShape *mProxyShape;

       Vector3 a,b;

    public:

        CallbeckSupprtCollide( IProxyShape *_ProxyShape , const Vector3& _a , const Vector3& _b)
            :  mProxyShape(_ProxyShape), a(_a), b(_b)
        {
        }

        Vector3 SupportPoint2(const Vector3& direction) const
        {
            auto q = mProxyShape->GetWorldTransform().GetBasis();
            return mProxyShape->GetWorldTransform() *
                   (mProxyShape->GetCollisionShape()->GetLocalSupportPoint(q.GetTranspose() * direction) * 1.0);
        }

        Vector3 SupportPoint(const Vector3& direction) const override
        {
            Vector3 sp[] = { a , b , SupportPoint2(direction)};
            int index = 0;
            float max = sp[index].Dot(direction);
            for (int i = 1; i < 3; ++i)
            {
                float d = sp[i].Dot(direction);
                if(d > max)
                {
                    max = d;
                    index = i;
                }
            }
            return sp[index];
        }


        virtual const Matrix4 GetTransform() const override
        {
            return mProxyShape->GetWorldTransform().GetTransformMatrix();
        }
};

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



//----------------------------------------//

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

    int numer_end;

    //---------------- Manipulator -------------------//
    std::auto_ptr<IGizmoManipulator> mGizmoManipulator;
    std::set<int>                    mSelectedIndexIds;

    bool isSelectedStatus;

    //----------------------------//

    IEngineFactoryRobot *mFactoryMethod;

    //---------------------------//

//    scalar mAngleEyeLidar;
//    IComponentMesh *mBoxLidar;


//    struct PointLIDAR
//    {
//       PointLIDAR(float _distance , float _angle , Vector3 _point) :
//           distance(_distance) ,
//           angle(_angle),
//           point(_point)
//       {}

//       float distance;
//       float angle;
//       Vector3 point;
//    };

//    std::vector<PointLIDAR> mLiDARPoints;


    FontProvider mFontProvider;
    QOpenGLShaderProgram mProgramFont;

    //---------------------------//

    Vector3 m_TargetPoint;
    IMesh *m_BoxMeshBottom;
    std::vector<Vector3> hull;
    std::vector<Vector3> polygon;





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

    float speed_point = 1.0;
    float angle_yaw;

    float Max_MotorPower;


    bool m_IsTrackingMove;
    std::vector<Vector3> mTrackerPoints;
    Vector3 m_pickPoint;


//    float Max_Length;
//    float MaxDistanceLIDAR = 50;
//    std::vector<Vector3> mClipPoints;
//    std::vector<Vector3> mClipPoints2;

    bool isRevert = false;

    bool m_IsConnectUDP;
    bool m_IsTracking;
    bool m_isHelp;

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
