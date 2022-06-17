#include "IOverlappingPair.h"

namespace IEngine
{


// Constructor
IOverlappingPair::IOverlappingPair(IProxyShape* shape1, IProxyShape* shape2, i32 nbMaxContactManifolds)
: mShape1(shape1) ,
  mShape2(shape2) ,
  mContactManifoldSet(shape1, shape2, nbMaxContactManifolds)
{

}

// Destructor
IOverlappingPair::~IOverlappingPair()
{
    mContactManifoldSet.Clear();
}

/// Add new contact
void IOverlappingPair::AddContact(IContactPoint* contact)
{
     mContactManifoldSet.AddContactPoint(contact);
}

/// Update of repair delete contact
void IOverlappingPair::Update()
{
    mContactManifoldSet.Update();
}

void IOverlappingPair::Update_delete_not_uset_contact()
{
   mContactManifoldSet.Update_delete_not_uset_contact();
}

///Clear all contact
void IOverlappingPair::ClearContactPoints()
{
    mContactManifoldSet.Clear();
}


}
