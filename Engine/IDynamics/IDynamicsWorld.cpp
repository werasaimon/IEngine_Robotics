#include "IDynamicsWorld.h"
#include "IIntegrateUtil.h"


//#include <GL/gl.h>


namespace IEngine
{


using namespace IMath;

IDynamicsWorld::IDynamicsWorld(const Vector3& gravity)
: mSplitVelocities(nullptr),
  mConstrainedVelocities(nullptr),
  mConstrainedPositions(nullptr),
  mContactSolver(mMapBodyToConstrainedVelocityIndex) ,
  mConstraintSolver(mMapBodyToConstrainedVelocityIndex) ,
  mTimer( scalar(1.0) ),
  mIsSleepingEnabled(SLEEPING_ENABLED),
  mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
  mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
  mGravity(gravity) ,
  mNbBodiesCapacity(0) ,
  mNbIslandsCapacity(0) ,
  mNbIslands(0) ,
  mIslands(nullptr)
{
    resetContactManifoldListsOfBodies();
    mTimer.start();
}


// Destroy all the  world
IDynamicsWorld::~IDynamicsWorld()
{
  Destroy();
}




// Destroy all the  world
void IDynamicsWorld::Destroy()
{

    // Stop timer
    mTimer.stop();


    // Destroy all the joints that have not been removed
    for (auto itJoints = mPhysicsJoints.begin(); itJoints != mPhysicsJoints.end();)
    {
        std::set<IJoint*>::iterator itToRemove = itJoints;
        ++itJoints;
        DestroyJoint(*itToRemove);
    }



    // Destroy all the rigid bodies that have not been removed
    for (auto itRigidBodies = mPhysicsBodies.begin(); itRigidBodies != mPhysicsBodies.end();)
    {
        std::set<IRigidBody*>::iterator itToRemove = itRigidBodies;
        ++itRigidBodies;
        DestroyBody(*itToRemove);
    }



    // Destroy all pair collisions that have not been removed
    if(!mCollisionDetection.mContactOverlappingPairs.empty())
    {
        for( auto pair : mCollisionDetection.mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mCollisionDetection.mContactOverlappingPairs.clear();
    }

    assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);
    assert(mCollisionDetection.mContactOverlappingPairs.empty());


    // Release the memory allocated for the bodies velocity arrays
    if (mNbBodiesCapacity > 0)
    {
        delete[] mSplitVelocities;
        delete[] mConstrainedVelocities;
        delete[] mConstrainedPositions;
    }

    //assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);


}


void IDynamicsWorld::UpdateFixedTimeStep(scalar timeStep)
{
   mTimer.setTimeStep(timeStep);

  if(mTimer.getIsRunning())
  {
     mTimer.update();

     while( mTimer.isPossibleToTakeStep() )
     {
         Update(timeStep);

         // next step simulation
         mTimer.nextStep();
     }
  }

}

void IDynamicsWorld::Update(scalar timeStep)
{

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Overlapping pairs in contact (during the current Narrow-phase collision detection)
    mCollisionDetection.FindNewContacts();

    // Compute the islands (separate groups of bodies with constraints between each others)
    ComputeIslands();

    //------------------------------------------------------------------------//

    // Initialize the bodies velocity arrays
    InitVelocityArrays();

    // Integrate the velocities
    IntegrateVelocities(timeStep);

    // Solve the contacts and constraints
    SolveContactsAndConstraints(timeStep);

    // Integrate the motion of each body
    IntegratePositions(timeStep);

    // Solve the position correction for constraints
    SolvePositionCorrection();

    // Update the state (positions and velocities) of the bodies
    UpdateBodiesState(timeStep);

    // Sleeping for all bodies
    if(mIsSleepingEnabled)
    {
       UpdateSleepingBodies(timeStep);
    }

    // Reset the external force and torque applied to the bodies
    ResetBodiesForceAndTorque();



}



/**/
//// Put bodies to sleep if needed.
///// For each island, if all the bodies have been almost still for a long enough period of
///// time, we put all the bodies of the island to sleep.
void IDynamicsWorld::UpdateSleepingBodies(scalar timeStep)
{

    const scalar sleepLinearVelocitySquare  = (DEFAULT_SLEEP_LINEAR_VELOCITY * DEFAULT_SLEEP_LINEAR_VELOCITY);
    const scalar sleepAngularVelocitySquare = (DEFAULT_SLEEP_ANGULAR_VELOCITY * DEFAULT_SLEEP_ANGULAR_VELOCITY);
    //const scalar sleepAngularSplitSquare    = (DEFAULT_SLEEP_SPLIT  * DEFAULT_SLEEP_SPLIT);


    // For each island of the world
    for (u32 i=0; i<mNbIslands; i++)
    {

        scalar minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        IRigidBody** bodies = mIslands[i]->GetBodies();
        for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
        {

            // Skip static bodies
            if (bodies[b]->GetType() == STATIC) continue;

            // If the body is velocity is large enough to stay awake

            // If the body is velocity is large enough to stay awake
            if (bodies[b]->mLinearVelocity.LengthSquare()  >  sleepLinearVelocitySquare   ||
                bodies[b]->mAngularVelocity.LengthSquare() >  sleepAngularVelocitySquare  ||
               !bodies[b]->IsAllowedToSleep())
            {
                // Reset the sleep time of the body
                bodies[b]->mSleepTime = scalar(0.0);
                minSleepTime = scalar(0.0);
            }
            else
            {  // If the body velocity is bellow the sleeping velocity threshold

                // Increase the sleep time
                bodies[b]->mSleepTime += timeStep;
                if (bodies[b]->mSleepTime < minSleepTime)
                {
                    minSleepTime = bodies[b]->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if ( minSleepTime >= DEFAULT_TIME_BEFORE_SLEEP )
        {

            // Put all the bodies of the island to sleep
            for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
            {
                bodies[b]->SetIsSleeping(true);
            }
        }
    }


}



// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void IDynamicsWorld::ComputeIslands()
{

    u32 nbBodies = mPhysicsBodies.size();
    u32 nbJoints = mPhysicsJoints.size();


    // Clear all the islands
    for (u32 i=0; i<mNbIslands; i++)
    {
        // Call the island destructor
        // Release the allocated memory for the island
        delete mIslands[i];
    }

    // Allocate and create the array of islands
    if (mNbIslandsCapacity != nbBodies && nbBodies > 0)
    {
        if (mNbIslandsCapacity > 0)
        {
             //std::cout << "Size_T  " << sizeof(mIslands) << std::endl;
            delete[] mIslands;
        }

        mNbIslandsCapacity = nbBodies;
        mIslands = new IIsland*[mNbIslandsCapacity];
    }

    mNbIslands = 0;
    u32 nbContactManifolds = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {
        i32 nbBodyManifolds = (*it)->ResetIsAlreadyInIslandAndCountManifolds();
        nbContactManifolds += nbBodyManifolds;
    }

    for (auto it = mPhysicsJoints.begin(); it !=  mPhysicsJoints.end(); ++it)
    {
        (*it)->mIsAlreadyInIsland = false;
    }


    IRigidBody** stackBodiesToVisit = new IRigidBody*[nbBodies];
    int i=0;
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
    {
        IRigidBody* body = *it;
        stackBodiesToVisit[i++] =    static_cast<IRigidBody*>(body);
    }


    // For each rigid body of the world
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {

        IRigidBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is static, we go to the next body
        if (body->GetType() == STATIC) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->IsSleeping() || !body->IsActive()) continue;

        // Reset the stack of bodies to visit
        u32 stackIndex = 0;
        stackBodiesToVisit[stackIndex] = static_cast<IRigidBody*>(body);
        stackIndex++;
        body->mIsAlreadyInIsland = true;

        // Create the new island
        mIslands[mNbIslands] = new IIsland( nbBodies , nbContactManifolds , nbJoints );

        // While there are still some bodies to visit in the stack
        while (stackIndex > 0)
        {

            // Get the next body to visit from the stack
            stackIndex--;
            IRigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->IsActive());

            // Awake the body if it is slepping
            bodyToVisit->SetIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->AddBody(bodyToVisit);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            if (bodyToVisit->GetType() == STATIC) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != nullptr;  contactElement = contactElement->GetNext())
            {
                IContactManifold* contactManifold = contactElement->GetPointer();

                assert(contactManifold->GetNbContactPoints() > 0);

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;


                // Add the contact manifold into the island
                mIslands[mNbIslands]->AddContactManifold(contactManifold);
                contactManifold->mIsAlreadyInIsland = true;


                // Get the other body of the contact manifold
                IRigidBody* body1 = static_cast<IRigidBody*>(contactManifold->GetBody1());
                IRigidBody* body2 = static_cast<IRigidBody*>(contactManifold->GetBody2());
                IRigidBody* otherBody = (body1->GetID() == bodyToVisit->GetID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }



            /**/
            IListElement<IJoint>* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != nullptr; jointElement = jointElement->GetNext())
            {
                IJoint* joint = jointElement->GetPointer();

                // Check if the current joint has already been added into an island
                if (joint->IsAlreadyInIsland()) continue;

                // Add the joint into the island
                mIslands[mNbIslands]->AddJoint(joint);
                joint->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                IRigidBody* body1 = static_cast<IRigidBody*>(joint->GetBody1());
                IRigidBody* body2 = static_cast<IRigidBody*>(joint->GetBody2());
                IRigidBody* otherBody = (body1->GetID() == bodyToVisit->GetID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }
            /**/
        }



        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (u32 i=0; i < mIslands[mNbIslands]->mNbBodies; i++)
        {
            if (mIslands[mNbIslands]->mBodies[i]->GetType() == STATIC)
            {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;

    }

    delete[] stackBodiesToVisit;

}



IRigidBody *IDynamicsWorld::CreateRigidBody(const Transform &transform)
{
    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<bodyindex>::max());

    // Create the rigid body
    IRigidBody* rigidBody = new IRigidBody(transform, &mCollisionDetection, bodyID);
    assert(rigidBody != NULL);


    // Add the rigid body to the physics world
    mBodies.insert(rigidBody);
    mPhysicsBodies.insert(rigidBody);

    // Return the pointer to the rigid body
    return rigidBody;
}

void IDynamicsWorld::DestroyBody(IRigidBody *rigidBody)
{

    // Remove all the collision shapes of the body
    rigidBody->RemoveAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->GetID());


    /**/
    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    if(rigidBody->mJointsList != nullptr )
    {
      for (element = rigidBody->mJointsList; element != nullptr; element = element->GetNext() )
      {
           DestroyJoint(element->GetPointer());
           element = nullptr;
      }
    }
    rigidBody->mJointsList = nullptr;
    /**/


    // Reset the contact manifold list of the body
    rigidBody->ResetContactManifoldsList();


    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mPhysicsBodies.erase(rigidBody);


    // Call the destructor of the rigid body
    // Free the object from the memory allocator
    delete rigidBody;

}


/**/
IJoint* IDynamicsWorld::CreateJoint(const IJointInfo& jointInfo)
{

       IJoint* newJoint = nullptr;


       switch (jointInfo.type)
       {

          case BALLSOCKETJOINT:
          {
              const IBallAndSocketJointInfo& info = static_cast<const IBallAndSocketJointInfo&>(jointInfo);
              newJoint = new IBallAndSocketJoint(info);

              break;
          }


          case HINGEJOINT:
          {
              const IHingeJointInfo& info = static_cast<const IHingeJointInfo&>(jointInfo);
              newJoint = new IHingeJoint(info);

              break;
          }

          case SLIDERJOINT:
          {
              const ISliderJointInfo& info = static_cast<const ISliderJointInfo&>(jointInfo);
              newJoint = new ISliderJoint(info);

              break;
          }

          case FIXEDJOINT:
          {
              const IFixedJointInfo& info = static_cast<const IFixedJointInfo&>(jointInfo);
              newJoint = new IFixedJoint(info);

              break;
          }

                default:
            //cout << "is not init joint info " <<endl;
            break;
        }



        // If the collision between the two bodies of the constraint is disabled
        if (!jointInfo.isCollisionEnabled)
        {
            // Add the pair of bodies in the set of body pairs that cannot collide with each other
            mCollisionDetection.AddNoCollisionPair(jointInfo.body1, jointInfo.body2);
        }

        // Add the joint into the world
        mPhysicsJoints.insert(newJoint);


        // Add the joint into the joint list of the bodies involved in the joint
        AddJointToBody(newJoint);

        // Return the pointer to the created joint
        return newJoint;
}
/**/


/**/
void IDynamicsWorld::DestroyJoint(IJoint* joint)
{

    assert(joint != NULL);
    // If the collision between the two bodies of the constraint was disabled
    if (!joint->IsCollisionEnabled())
    {
        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.RemoveNoCollisionPair(joint->GetBody1(), joint->GetBody2());
    }

    // Wake up the two bodies of the joint
    joint->GetBody1()->SetIsSleeping(false);
    joint->GetBody2()->SetIsSleeping(false);

    // Remove the joint from the world
    mPhysicsJoints.erase(joint);

    // Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->RemoveJointFromJointsList( joint );
    joint->mBody2->RemoveJointFromJointsList( joint );

    // Call the destructor of the joint
    delete joint;

}
/**/


/**/
void IDynamicsWorld::AddJointToBody(IJoint *joint)
{
    assert(joint != NULL);

    // Add the joint at the beginning of the linked list of joints of the first body
    JointListElement* jointListElement1 = new JointListElement(joint , joint->mBody1->mJointsList , nullptr );
    joint->mBody1->mJointsList = jointListElement1;


    // Add the joint at the beginning of the linked list of joints of the second body
    JointListElement* jointListElement2 = new JointListElement(joint , joint->mBody2->mJointsList , nullptr );
    joint->mBody2->mJointsList = jointListElement2;


}
/**/



u32 IDynamicsWorld::GetNbIterationsVelocitySolver() const
{
    return mNbVelocitySolverIterations;
}

void IDynamicsWorld::SetNbIterationsVelocitySolver(u32 nbIterations)
{
    mNbVelocitySolverIterations = nbIterations;
}

u32 IDynamicsWorld::GetNbIterationsPositionSolver() const
{
    return mNbPositionSolverIterations;
}


void IDynamicsWorld::SetNbIterationsPositionSolver(u32 nbIterations)
{
     mNbPositionSolverIterations = nbIterations;
}




//*************************************************************************************//


bool IDynamicsWorld::isSleepingEnabled() const
{
    return mIsSleepingEnabled;
}

void IDynamicsWorld::SetIsSleepingEnabled(bool isSleepingEnabled)
{
    mIsSleepingEnabled = isSleepingEnabled;
}

bool IDynamicsWorld::isGravityEnabled() const
{
    return mIsGravityEnabled;
}

void IDynamicsWorld::SetIsGravityEnabled(bool isGravityEnabled)
{
    mIsGravityEnabled = isGravityEnabled;
}

void IDynamicsWorld::SetGravity(const Vector3 &gravity)
{
    mGravity = gravity;
}

void IDynamicsWorld::InitVelocityArrays()
{
    // Allocate memory for the bodies velocity arrays
    u32 nbBodies = mPhysicsBodies.size();
    if (mNbBodiesCapacity != nbBodies && nbBodies > 0)
    {
        if (mNbBodiesCapacity > 0)
        {
            delete[] mSplitVelocities;
            delete[] mConstrainedVelocities;
            delete[] mConstrainedPositions;
        }

        mNbBodiesCapacity = nbBodies;


        // TODO : Use better memory allocation here
        mSplitVelocities       = new IVelocity[mNbBodiesCapacity];
        mConstrainedVelocities = new IVelocity[mNbBodiesCapacity];
		mConstrainedPositions  = new IMovement[mNbBodiesCapacity];
        assert(mSplitVelocities != NULL);
        assert(mConstrainedVelocities != NULL);
        assert(mConstrainedPositions  != NULL);

    }



  // Reset the velocities arrays
  for (u32 i=0; i<mNbBodiesCapacity; i++)
  {
      mSplitVelocities[i].v.SetToZero();
      mSplitVelocities[i].w.SetToZero();
  }

  // Initialize the map of body indexes in the velocity arrays
  mMapBodyToConstrainedVelocityIndex.clear();
  u32 indexBody = 0;
  for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
  {
      // Add the body into the map
      mMapBodyToConstrainedVelocityIndex.insert(std::make_pair(*it, indexBody));
      indexBody++;
  }
}


// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void IDynamicsWorld::IntegrateVelocities(scalar TimeStep)
{

  // For each island of the world
  for (u32 i=0; i < mNbIslands; i++)
  {

     IRigidBody** bodies = mIslands[i]->GetBodies();

      // For each body of the island
      for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
      {

          // Insert the body into the map of constrained velocities
          u32 indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

          assert(mSplitVelocities[indexBody].v == Vector3(0, 0, 0));
          assert(mSplitVelocities[indexBody].w == Vector3(0, 0, 0));


          // If the gravity has to be applied to this rigid body
          //if (bodies[b]->isGravityEnabled() && mIsGravityEnabled)
          if(bodies[b]->mType == BodyType::DYNAMIC)
          {

              // Integrate the gravity force and acceleration
              Vector3 LinearAcceleration  =  (bodies[b]->mExternalForce + mGravity) * TimeStep;
              Vector3 AngularAcceleration =   bodies[b]->mExternalTorque * TimeStep;


              // Integrate the external force to get the new velocity of the body
              mConstrainedVelocities[indexBody].v = bodies[b]->GetLinearVelocity()  + LinearAcceleration;
              mConstrainedVelocities[indexBody].w = bodies[b]->GetAngularVelocity() + AngularAcceleration;

          }

          // If it is a static body
          if (bodies[b]->mType == STATIC || bodies[b]->mType == KINEMATIC)
          {
              // Reset the velocity to zero
              mConstrainedVelocities[indexBody].v=Vector3(0,0,0);
              mConstrainedVelocities[indexBody].w=Vector3(0,0,0);

          }

          indexBody++;
      }
  }

}


// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void IDynamicsWorld::IntegratePositions(scalar TimeStep)
{

  // For each island of the world
     for (u32 i=0; i < mNbIslands; i++)
     {

        IRigidBody** bodies = mIslands[i]->GetBodies();

         // For each body of the island
         for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
         {

             // Get the constrained velocity
             u32 indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;


             /*********************************************
              *          Damping  velocity
              ********************************************/
             scalar linDampingFactor = bodies[b]->GetLinearDamping();
             scalar angDampingFactor = bodies[b]->GetAngularDamping();
             IIntegrateUtil::Damping( mConstrainedVelocities[indexArray].v , MINIMUM_FOR_DAPING , linDampingFactor  );
             IIntegrateUtil::Damping( mConstrainedVelocities[indexArray].w , MINIMUM_FOR_DAPING , angDampingFactor  );


             // Get current position and orientation of the body
             const Vector3&    currentPosition    = bodies[b]->mCenterOfMassWorld;
             const Quaternion& currentOrientation = bodies[b]->GetTransform().GetRotation();


             Vector3 newLinVelocity = mConstrainedVelocities[indexArray].v;
             Vector3 newAngVelocity = mConstrainedVelocities[indexArray].w;

//             // Add the split impulse velocity from Contact Solver (only used
//             // to update the position)
             if (mContactSolver.IsSplitImpulseActive())
             {

                 newLinVelocity += mSplitVelocities[indexArray].v;
                 newAngVelocity += mSplitVelocities[indexArray].w;

             }


//             glPushMatrix();
//             glPointSize(10);
//             glColor3f(1,0,0);
//             glBegin(GL_POINTS);
//                glVertex3fv(mConstrainedPositions[indexArray].x);
//             glEnd();
//             glPopMatrix();

             /**
			 /// Translation Object
             Transform resulTransform( currentPosition , currentOrientation);
             resulTransform = IIntegrateUtil::IntegrateTransform(resulTransform,newLinVelocity,newAngVelocity,TimeStep);
			 resulTransform = IIntegrateUtil::IntegrateTransform(resulTransform,mSplitVelocities[indexArray].v,mSplitVelocities[indexArray].w,TimeStep);
			 mConstrainedPositions[indexArray].x = resulTransform.GetPosition();
			 mConstrainedPositions[indexArray].q = resulTransform.GetRotation();

             **/

             // Update the new constrained position and orientation of the body
             if(bodies[b]->isMove())
             {
                mConstrainedPositions[indexArray].x = currentPosition + newLinVelocity * TimeStep;
             }

             if(bodies[b]->IsRotate())
             {
                mConstrainedPositions[indexArray].q = currentOrientation + Quaternion(0, newAngVelocity) * currentOrientation * scalar(0.5) * TimeStep;
             }
             /**/

        }
     }

}



// Update the postion/orientation of the bodies
void IDynamicsWorld::UpdateBodiesState( scalar TimeStep )
{
  // For each island of the world
  for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
  {
      // For each body of the island
      IRigidBody** bodies = mIslands[islandIndex]->GetBodies();

      for (u32 b=0; b < mIslands[islandIndex]->GetNbBodies(); b++)
      {

          u32 index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

          // Update the linear and angular velocity of the body
          bodies[b]->mLinearVelocity  = mConstrainedVelocities[index].v;
          bodies[b]->mAngularVelocity = mConstrainedVelocities[index].w;

          // Update the position of the center of mass of the body
          bodies[b]->mCenterOfMassWorld = mConstrainedPositions[index].x;

          // Update the orientation of the body
          bodies[b]->mTransform.SetBasis(mConstrainedPositions[index].q.GetRotMatrix());

          // Update the transform of the body (using the new center of mass and new orientation)
          bodies[b]->UpdateTransformWithCenterOfMass();

          // Update the broad-phase state of the body
          //bodies[b]->updateBroadPhaseState();
          bodies[b]->UpdateBroadPhaseStatee(TimeStep);

      }
   }
}




/**/
void IDynamicsWorld::SolveContactsAndConstraints( scalar TimeStep )
{

     if(mPhysicsBodies.empty() && mPhysicsJoints.empty()) return;


     // ---------- Solve velocity constraints for joints and contacts ---------- //

     // Set the velocities arrays
     mContactSolver.SetSplitVelocitiesArrays(mSplitVelocities);
     mContactSolver.SetConstrainedVelocitiesArrays(mConstrainedVelocities);


     mConstraintSolver.SetConstrainedVelocitiesArrays(mConstrainedVelocities);
     mConstraintSolver.SetConstrainedPositionsArrays(mConstrainedPositions);



     // For each island of the world
     for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
     {

         // Check if there are contacts and constraints to solve
         bool isConstraintsToSolve = mIslands[islandIndex]->GetNbJoints() > 0;
         bool isContactsToSolve = mIslands[islandIndex]->GetNbContactManifolds() > 0;
         if (!isContactsToSolve && !isConstraintsToSolve) continue;

         // If there are contacts in the current island
         if (isContactsToSolve)
         {
             // Initialize the solver
             mContactSolver.InitializeForIsland( TimeStep , mIslands[islandIndex] );
             // Warm start the contact solver
             mContactSolver.WarmStart();
         }

         // If there are constraints
         if (isConstraintsToSolve)
         {
             // Initialize the constraint solver
             mConstraintSolver.InitializeForIsland(TimeStep , mIslands[islandIndex]);
         }



         // For each iteration of the velocity solver
         for (u32 i=0; i<mNbVelocitySolverIterations; i++)
         {
             // Solve the contacts
             if (isContactsToSolve) mContactSolver.SolveVelocityConstraint();

             // Solve the constraints
             if (isConstraintsToSolve)  mConstraintSolver.SolveVelocityConstraints(mIslands[islandIndex]);

         }


         // For each iteration of the split velocity solver
         for (u32 i=0; i<mNbPositionSolverIterations; i++)
         {
             // Solve the contacts
             if (isContactsToSolve) mContactSolver.SolveSplitVelocityConstraint();
         }


//         // For each iteration of the position (error correction) solver
//         for (u32 i=0; i<mNbPositionSolverIterations; i++)
//         {
//             // Solve the position constraints
//             mContactSolver.slovePositionConstraint();
//         }

         // Cache the lambda values in order to use them in the next
         // step and cleanup the contact solver
         if (isContactsToSolve)
         {
             mContactSolver.StoreImpulsesWarmstart();
             mContactSolver.Cleanup();
         }
     }
}


// Solve the position error correction of the constraints
void IDynamicsWorld::SolvePositionCorrection()
{
    // Do not continue if there is no constraints
    if (mPhysicsJoints.empty()) return;

    // For each island of the world
    for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
    {

        // ---------- Solve the position error correction for the constraints ---------- //
        if(!mIslands[islandIndex]->GetNbJoints()) continue;

        // For each iteration of the position (error correction) solver
        for (u32 i=0; i<mNbPositionSolverIterations; i++)
        {
            // Solve the position constraints
            mConstraintSolver.SolvePositionConstraints(mIslands[islandIndex]);
        }
    }
}

void IDynamicsWorld::ResetBodiesForceAndTorque()
{
  // For each body of the world
  for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
  {
      (*it)->mExternalForce = Vector3(0,0,0);
      (*it)->mExternalTorque = Vector3(0,0,0);
  }
}
/**/



}
