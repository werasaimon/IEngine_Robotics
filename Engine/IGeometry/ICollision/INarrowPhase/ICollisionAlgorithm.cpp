#include "ICollisionAlgorithm.h"

namespace IEngine
{

// Constructor
ICollisionAlgorithm::ICollisionAlgorithm()
{

}

// Destructor
ICollisionAlgorithm::~ICollisionAlgorithm()
{

}


IOutputCollisionInfo ICollisionAlgorithm::GetOutputCollisionInfo() const
{
    return mOutputCollisionInfo;
}




}
