#include "IContactPoint.h"


namespace IEngine
{

// Constructor
IContactPoint::IContactPoint(const IContactPointInfo& contactInfo)
  : mNormal(contactInfo.normal),
    mPenetration(contactInfo.penetration),
    mLocalPointOnBody1(contactInfo.localPoint1),
    mLocalPointOnBody2(contactInfo.localPoint2),
    mWorldPointOnBody1((contactInfo.localPoint1)),
    mWorldPointOnBody2( contactInfo.localPoint2),
    mIsRestingContact(false)
{

}

// Destructor
IContactPoint::~IContactPoint()
{

}


}
