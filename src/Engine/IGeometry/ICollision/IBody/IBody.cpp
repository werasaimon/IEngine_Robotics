#include "IBody.h"

namespace IEngine
{

// Constructor
/**
 * @param id ID of the new body
 */
IBody::IBody(bodyindex id)
 : mID(id),
   mIsAlreadyInIsland(false),
   mIsAllowedToSleep(true),
   mIsActive(true),
   mIsSleeping(false),
   mSleepTime(0),
   mUserData(nullptr)
{

}

// Destructor
IBody::~IBody()
{

}


}
