#ifndef IENGINEFACTORY_H
#define IENGINEFACTORY_H


#include "Engine/engine.hpp"


using namespace IEngine;

class VehicleRobotCar;
class SceneMain;

class IEngineFactory
{
    public:

        IEngineFactory(SceneMain *_EngineScene);

        SceneMain *mSceneEngin;
};



struct RobotDescriptor
{
   Vector3 mSize;
   float mRadiusWheels;


   RobotDescriptor(const Vector3 _size , float _wheels_radius) :
   mSize(_size),
   mRadiusWheels(_wheels_radius)
   {

   }

};


class IEngineFactoryRobot : public IEngineFactory
{

    public:



    public:

        IEngineFactoryRobot(SceneMain *_EngineScene)
        : IEngineFactory(_EngineScene)
        {

        }


        VehicleRobotCar *CreateRobot(RobotDescriptor _descriptor, const Transform& transform);







};

#endif // IENGINEFACTORY_H
