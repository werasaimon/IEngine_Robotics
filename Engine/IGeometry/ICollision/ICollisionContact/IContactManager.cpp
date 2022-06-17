#include "IContactManager.h"
#include "../../ICollision/IBroadPhase/IBroadPhase.h"
#include "../../ICollision/INarrowPhase/ICollisionAlgorithmGjkEpa.h"


//#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/freeglut.h>

//#include <iostream>
//using namespace std;


namespace IEngine
{



IContactManager::IContactManager()
 : mBroadPhaseAlgorithm( this )
{

}


IContactManager::~IContactManager()
{
   mBroadPhaseAlgorithm.~IBroadPhase();
}


void IContactManager::FindNewContacts()
{
    // Compute the broad-phase collision detection
    ComputeBroadPhase();

    // Compute the narrow-phase collision detection
    ComputeNarrowPhase();
}


void IContactManager::ComputeBroadPhase()
{

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded)
    {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
         mBroadPhaseAlgorithm.ComputeOverlappingPairs();
    }

}

void IContactManager::ComputeNarrowPhase()
{

    ///-----------------------------------///
    for( auto pair : mContactOverlappingPairs )
    {
        //pair.second->isFakeCollision = true;
        mPairsIsRemoves[pair.first] = true;
    }

    /*********************************
     * clear memory all pairs
     ********************************
    if(!mContactOverlappingPairs.empty())
    {
        for( auto pair : mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mContactOverlappingPairs.clear();
    }
    /*********************************/


    i32 CollisionPairNbCount = 0;

    // For each possible collision pair of bodies
    // std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
    {

        IOverlappingPair* pair = it->second;

        IProxyShape* shape1 = pair->GetShape1();
        IProxyShape* shape2 = pair->GetShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (((shape1->GetCollideWithMaskBits() & shape2->GetCollisionCategoryBits()) == 0  ||
             (shape1->GetCollisionCategoryBits() & shape2->GetCollideWithMaskBits()) == 0) ||
             !mBroadPhaseAlgorithm.TestOverlappingShapes(shape1, shape2))
        {

            std::map<overlappingpairid, IOverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            delete itToRemove->second;
            mOverlappingPairs.erase(itToRemove);

            continue;
        }
        else
        {
            ++it;
        }



        ICollisionBody* const body1 = shape1->GetBody();
        ICollisionBody* const body2 = shape2->GetBody();

        // Update the contact cache of the overlapping pair
        pair->Update();

        // Check that at least one body is awake and not static
        bool isBody1Active = !body1->IsSleeping() && body1->GetType() != STATIC;
        bool isBody2Active = !body2->IsSleeping() && body2->GetType() != STATIC;
        if (!isBody1Active && !isBody2Active) continue;

        // Check if the bodies are in the set of bodies that cannot collide between each other
        bodyindexpair bodiesIndex = IOverlappingPair::ComputeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        CollisionPairNbCount++;




        /**********************************************************************/

        //rpCollisionAlgorithm* narrowPhaseAlgorithm = new  mCollisionMatrix[shape1Type][shape2Type];

        ICollisionAlgorithm* narrowPhaseAlgorithm  = new  ICollisionAlgorithmGjkEpa;
        IContactGenerator* generate_contact = new  IContactGenerator( shape1 , shape2 , narrowPhaseAlgorithm );

        overlappingpairid pairId = IOverlappingPair::ComputeID(shape1,  shape2);

        if( generate_contact->ComputeCollisionAndFindContactPoints() && generate_contact->GetContactPoints() != nullptr )
        {

            bool isValidNewContactInfo = false;
            if( mContactOverlappingPairs.find(pairId) == mContactOverlappingPairs.end())
            {
                isValidNewContactInfo = true;
                ///Compute the maximum number of contact manifolds for this pair
                int nbMaxManifolds = ICollisionShape::ComputeNbMaxContactManifolds(shape1->GetCollisionShape()->GetType(),
                                                                                   shape2->GetCollisionShape()->GetType());

                mContactOverlappingPairs.insert( std::make_pair( pairId , new IOverlappingPair(shape1,shape2,nbMaxManifolds)) );
            }

            /// Update world transform to contscts
            mContactOverlappingPairs[pairId]->Update();

            ///Compute contacts solve and warmsart c-store
            ComputeContacts( shape1 ,
                             shape2 ,
                             mContactOverlappingPairs[pairId] ,
                             generate_contact->GetContactPoints() ,
                             generate_contact->GetCountContactPoints() ,
                             isValidNewContactInfo);

             /// Delete non-used to contacts
             mContactOverlappingPairs[pairId]->Update_delete_not_uset_contact();


            //mContactOverlappingPairs[pairId]->update();
            //mContactOverlappingPairs[pairId]->isFakeCollision = false;
            mContactOverlappingPairs[pairId]->SetCachedSeparatingAxis(generate_contact->GetCachedSeparatingAxis());
            mPairsIsRemoves[pairId] = false;

        }

        delete generate_contact;
        delete narrowPhaseAlgorithm;

        /*********************************************************************/
    }

//    // Delete Pairs
//    for( auto pair : mContactOverlappingPairs )
//    {
//        if(pair.second->isFakeCollision)
//        {
//            delete pair.second;
//            mContactOverlappingPairs.erase(pair.first);
//        }
//    }

    for( auto it : mPairsIsRemoves )
    {
        if(it.second == true)
        {
           delete mContactOverlappingPairs[it.first];
           mContactOverlappingPairs.erase(it.first);
        }
    }

    // Add all the contact manifolds (between colliding bodies) to the bodies
    AddAllContactManifoldsToBodies();

}

void IContactManager::ComputeContacts(  const IProxyShape* Shape1 ,
                                        const IProxyShape* Shape2 ,
                                        IOverlappingPair*  OverlappingPair,
                                        IContactVertex* NewContactPoints ,
                                        u32 _NbNewContactPoints ,
                                        bool _isValidNewContactInfo )
{

      Transform transform_1 = Shape1->GetWorldTransform();
      Transform transform_2 = Shape2->GetWorldTransform();

       struct TempContactManiflod
       {
           Vector3 OldFriction1;
           Vector3 OldFriction2;

           scalar   AccumulatedFriction1Impulse;
           scalar   AccumulatedFriction2Impulse;
           scalar   AccumulatedFrictionTwistImpulse;
           Vector3  AccumulatedRollingResistanceImpulse;

       };


       const u32 NbCountStoreManiflod = OverlappingPair->GetNbContactManifolds();
       TempContactManiflod storee[NbCountStoreManiflod];
       std::map<short int , int> mapIndexNormalId;
       for( u32 i = 0; i < NbCountStoreManiflod; ++i )
       {
           IContactManifold *m =  OverlappingPair->GetContactManifoldSet().GetContactManifold(i);

           mapIndexNormalId[m->GetNormalDirectionId()] = i;

           storee[i].AccumulatedFriction1Impulse = m->GetAccumulatedFrictionImpulse1();
           storee[i].AccumulatedFriction2Impulse = m->GetAccumulatedFrictionImpulse2();
           storee[i].AccumulatedFrictionTwistImpulse = m->GetAccumulatedFrictionTwistImpulse();
           storee[i].AccumulatedRollingResistanceImpulse = m->GetAccumulatedRollingResistanceImpulse();

           storee[i].OldFriction1 = m->GetFrictionVector1();
           storee[i].OldFriction2 = m->GetFrictionVector2();
       }


       std::vector<IContactPoint> storyContacts = OverlappingPair->GetContactPoints();
       OverlappingPair->ClearContactPoints();



       //std::cout << _NbNewContactPoints << std::endl;

       const scalar MIN_EPS = 0.04;
       for (u32 i = 0; i < _NbNewContactPoints; ++i)
       {

           /**
           glPushMatrix();
           glColor3f(1,0,0);
           glTranslatef(  NewContactPoints[i].point1.x ,
                          NewContactPoints[i].point1.y ,
                          NewContactPoints[i].point1.z );
           glutWireSphere( 0.05 , 10 , 10);
           glPopMatrix();


           glPushMatrix();
           glColor3f(0,1,0);
           glTranslatef(  NewContactPoints[i].point2.x ,
                          NewContactPoints[i].point2.y ,
                          NewContactPoints[i].point2.z );
           glutWireSphere( 0.05 , 10 , 10);
           glPopMatrix();

           glColor3f(1,1,1);
           /**/



           Vector3 prev_A =   NewContactPoints[i].point1;
           Vector3 prev_B =   NewContactPoints[i].point2;

           NewContactPoints[i].point1 = ((transform_1.GetInverse() * NewContactPoints[i].point1));
           NewContactPoints[i].point2 = ((transform_2.GetInverse() * NewContactPoints[i].point2));


           ///*************************************************************************************************///

           IContactPoint *contact = new IContactPoint( IContactPointInfo( NewContactPoints[i].normal      ,
                                                                          NewContactPoints[i].penetration ,
                                                                          NewContactPoints[i].point1   ,
                                                                          NewContactPoints[i].point2 ));

           contact->SetWorldPointOnBody1(prev_A);
           contact->SetWorldPointOnBody2(prev_B);

           contact->SetIsRestingContact( /*!_isValidNewContactInfo*/ false);
           for( u32 it = 0; it < storyContacts.size() && !_isValidNewContactInfo; ++it )
           {

               if((contact->GetWorldPointOnBody1() - transform_1 * storyContacts[it].GetLocalPointOnBody1()).LengthSquare() <= MIN_EPS ||
                  (contact->GetWorldPointOnBody2() - transform_2 * storyContacts[it].GetLocalPointOnBody2()).LengthSquare() <= MIN_EPS)
                 {
                   // Get the cached accumulated impulses from the previous step
                   contact->SetIsRestingContact( true );
                   contact->SetAccumulatedPenetrationImpulse(storyContacts[it].GetAccumulatedPenetrationImpulse());
                   contact->SetAccumulatedFrictionImpulse1(storyContacts[it].GetAccumulatedFrictionImpulse1());
                   contact->SetAccumulatedFrictionImpulse2(storyContacts[it].GetAccumulatedFrictionImpulse2());
                   contact->SetAccumulatedRollingResistanceImpulse(storyContacts[it].GetAccumulatedRollingResistanceImpulse());

                   contact->SetFrictionVector1(storyContacts[it].GetFrictionVector1());
                   contact->SetFrictionVector2(storyContacts[it].GetFrictionVector2());
                   break;
                 }
           }


           this->CreateContact( OverlappingPair , contact );
         }



       for( int i = 0; i < OverlappingPair->GetNbContactManifolds() && !_isValidNewContactInfo; ++i )
       {
          IContactManifold *m =  OverlappingPair->GetContactManifoldSet().GetContactManifold(i);


          if(mapIndexNormalId.find(m->GetNormalDirectionId()) != mapIndexNormalId.end())
          {
              int indexMap = mapIndexNormalId.find(m->GetNormalDirectionId())->second;


              m->SetAccumulatedFrictionImpulse1( storee[indexMap].AccumulatedFriction1Impulse );
              m->SetAccumulatedFrictionImpulse2( storee[indexMap].AccumulatedFriction2Impulse );
              m->SetAccumulatedFrictionTwistImpulse( storee[indexMap].AccumulatedFrictionTwistImpulse );
              m->SetAccumulatedRollingResistanceImpulse( storee[indexMap].AccumulatedRollingResistanceImpulse );

              m->SetFrictionVector1( storee[indexMap].OldFriction1 );
              m->SetFrictionVector2( storee[indexMap].OldFriction2 );
          }

              m->SetIsNewContactInfoManiflod(_isValidNewContactInfo);
       }

      mapIndexNormalId.clear();
      storyContacts.clear();

}

void IContactManager::AddAllContactManifoldsToBodies()
{
    for (auto it = mContactOverlappingPairs.begin(); it != mContactOverlappingPairs.end(); ++it)
    {
          // Add all the contact manifolds of the pair into the list of contact manifolds
          // of the two bodies involved in the contact
          AddContactManifoldToBody(it->second);
    }

}

void IContactManager::AddContactManifoldToBody(IOverlappingPair *pair)
{
    assert(pair != NULL);

    ICollisionBody* body1 = pair->GetShape1()->GetBody();
    ICollisionBody* body2 = pair->GetShape2()->GetBody();
    const IContactManifoldSet& manifoldSet = pair->GetContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    for (i32 i=0; i<manifoldSet.GetNbContactManifolds(); i++)
    {

        IContactManifold* contactManifold = manifoldSet.GetContactManifold(i);

        assert(contactManifold->GetNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement *listElement1 = new ContactManifoldListElement( contactManifold , body1->mContactManifoldsList , nullptr );
        body1->mContactManifoldsList = listElement1;


        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        ContactManifoldListElement *listElement2 = new ContactManifoldListElement( contactManifold , body2->mContactManifoldsList , nullptr );
        body2->mContactManifoldsList = listElement2;

    }

}

void IContactManager::BroadPhaseNotifyOverlappingPair(void *_shape1, void *_shape2)
{
    IProxyShape *shape1 = static_cast<IProxyShape*>(_shape1);
    IProxyShape *shape2 = static_cast<IProxyShape*>(_shape2);

    assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

    // If the two proxy collision shapes are from the same body, skip it
    if (shape1->GetBody()->GetID() == shape2->GetBody()->GetID()) return;

    // Check if the collision filtering allows collision between the two shapes
    if ((shape1->GetCollideWithMaskBits() & shape2->GetCollisionCategoryBits()) == 0 ||
        (shape1->GetCollisionCategoryBits() & shape2->GetCollideWithMaskBits()) == 0) return;

    // Compute the overlapping pair ID
    overlappingpairid pairID = IOverlappingPair::ComputeID(shape1, shape2);

    // Check if the overlapping pair already exists
    if (mOverlappingPairs.find(pairID) != mOverlappingPairs.end()) return;


    // Create the overlapping pair and add it into the set of overlapping pairs
    IOverlappingPair* newPair = new IOverlappingPair(shape1, shape2);
    assert(newPair != NULL);

//#ifndef NDEBUG
//    std::pair<std::map<overlappingpairid, IOverlappingPair*>::iterator, bool> check =
//#endif
    mOverlappingPairs.insert(make_pair(pairID, newPair));

//    assert(check.second);

    // Wake up the two bodies
    shape1->GetBody()->SetIsSleeping(false);
    shape2->GetBody()->SetIsSleeping(false);

}



void IContactManager::CreateContact(IOverlappingPair *overlappingPair, IContactPoint *contact)
{
    // Add the contact to the contact manifold set of the corresponding overlapping pair
    overlappingPair->AddContact(contact);

    // Add the overlapping pair into the set of pairs in contact during narrow-phase
    overlappingpairid pairId = IOverlappingPair::ComputeID(overlappingPair->GetShape1(),
                                                           overlappingPair->GetShape2());
    mContactOverlappingPairs[pairId] = overlappingPair;

}

/**
 * Delete all the contact points in the currently overlapping pairs
 **/
void IContactManager::ClearContactPoints()
{
    // For each overlapping pair
     for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it)
     {
         it->second->ClearContactPoints();
     }
}

void IContactManager::AddProxyCollisionShape(IProxyShape *proxyShape, const IAABBox3D &aabb)
{
    // Add the body to the broad-phase
    mBroadPhaseAlgorithm.AddProxyCollisionShape(proxyShape, aabb);
    mIsCollisionShapesAdded = true;

}

void IContactManager::RemoveProxyCollisionShape(IProxyShape *proxyShape)
{
    // Remove all the overlapping pairs involving this proxy shape
    for ( auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
    {
      if (it->second->GetShape1()->mBroadPhaseID == proxyShape->mBroadPhaseID||
          it->second->GetShape2()->mBroadPhaseID == proxyShape->mBroadPhaseID)
      {
          std::map<overlappingpairid, IOverlappingPair*>::iterator itToRemove = it;
          ++it;

          // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

          // Destroy the overlapping pair
          //itToRemove->second->~IOverlappingPair();
          itToRemove->second->mContactManifoldSet.Clear();

          // Destroy the overlapping pair
          delete itToRemove->second;
          mOverlappingPairs.erase(itToRemove);
      }
      else
      {
          ++it;
      }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.RemoveProxyCollisionShape(proxyShape);
}

void IContactManager::UpdateProxyCollisionShape(IProxyShape *shape, const IAABBox3D &aabb, const Vector3 &displacement, bool forceReinsert)
{
    mBroadPhaseAlgorithm.UpdateProxyCollisionShape(shape, aabb, displacement , forceReinsert );
}

void IContactManager::AddNoCollisionPair(ICollisionBody *body1, ICollisionBody *body2)
{
    mNoCollisionPairs.insert(IOverlappingPair::ComputeBodiesIndexPair(body1, body2));
}

void IContactManager::RemoveNoCollisionPair(ICollisionBody *body1, ICollisionBody *body2)
{
    mNoCollisionPairs.erase(IOverlappingPair::ComputeBodiesIndexPair(body1, body2));
}

void IContactManager::AskForBroadPhaseCollisionCheck(IProxyShape *shape)
{
    mBroadPhaseAlgorithm.AddMovedCollisionShape(shape->mBroadPhaseID);
}

void IContactManager::Raycast(IRaycastCallback *raycastCallback, const IRay &ray, unsigned short raycastWithCategoryMaskBits) const
{
    IRaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each proxy shape hit by the ray in the broad-phase
    mBroadPhaseAlgorithm.Raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

std::map<overlappingpairid, IOverlappingPair *> IContactManager::ContactOverlappingPairs() const
{
    return mContactOverlappingPairs;
}




}
