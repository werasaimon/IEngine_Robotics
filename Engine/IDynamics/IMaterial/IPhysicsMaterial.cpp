#include "IPhysicsMaterial.h"
#include "../../ICommon/ISettings.h"

namespace IEngine
{

// Constructor
 IPhysicsMaterial::IPhysicsMaterial()
         : mFrictionCoefficient(DEFAULT_FRICTION_COEFFICIENT),
           mRollingResistance(DEFAULT_ROLLING_RESISTANCE),
           mBounciness(DEFAULT_BOUNCINESS)
{

}

// Copy-constructor
 IPhysicsMaterial:: IPhysicsMaterial(const IPhysicsMaterial& material)
         : mFrictionCoefficient(material.mFrictionCoefficient),
           mRollingResistance(material.mRollingResistance),
           mBounciness(material.mBounciness)
{

}

// Destructor
 IPhysicsMaterial::~IPhysicsMaterial()
{

}


}
