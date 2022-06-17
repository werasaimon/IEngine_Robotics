#ifndef IDYNAMICSWORLD_H
#define IDYNAMICSWORLD_H



#include "ITimer.h"
#include "IRigidBody.h"
#include "IIsland.h"

#include "../IGeometry/ICollision/ICollisionWorld.h"
//#include "IIntegrateUtil.h"

#include "IContactSolver.h"
#include "IConstraintSolver.h"

#include "IJoints/IJoint.h"
#include "IJoints/IBallAndSocketJoint.h"
#include "IJoints/IHingeJoint.h"
#include "IJoints/ISliderJoint.h"
#include "IJoints/IFixedJoint.h"

#include "IStep.h"


namespace IEngine
{



class IDynamicsWorld : public ICollisionWorld
{

protected:

     //---------------------- Attributes -----------------------//

    /// Map body to their index in the constrained velocities array
    std::map<IRigidBody*, u32> mMapBodyToConstrainedVelocityIndex;

    /// Split velocities for the position contact solver (split impulse)
    IVelocity* mSplitVelocities;

    /// Array of constrained velocities (state of the velocities
    /// after solving the constraints)
    IVelocity* mConstrainedVelocities;

    /// Array of constrained motion (for position error correction)
    IMovement* mConstrainedPositions;


    /// Contact solver
    IContactSolver    mContactSolver;

    /// Constraint solver
    IConstraintSolver mConstraintSolver;

    //-----------------------------------------------------------//

    /// Gravitation status
    bool mIsGravityEnabled;

    /// Update time step correctly interval
    ITimer mTimer;

    /// True if the spleeping technique for inactive bodies is enabled
    bool mIsSleepingEnabled;

    /// Number of iterations for the velocity solver of the Sequential Impulses technique
    u32 mNbVelocitySolverIterations;

    /// Number of iterations for the position solver of the Sequential Impulses technique
    u32 mNbPositionSolverIterations;

    /// Gravity force
    Vector3 mGravity;



    /// All the joints of the world
    std::set<IJoint*>     mPhysicsJoints;

    /// All the rigid bodies of the physics world
    std::set<IRigidBody*> mPhysicsBodies;



    /// Current allocated capacity for the bodies
    u32 mNbBodiesCapacity;

    /// Current allocated capacity for the islands
    u32 mNbIslandsCapacity;




    /// Number of islands in the world
    u32 mNbIslands;

    /// Array with all the islands of awaken bodies
    IIsland** mIslands;



    // -------------------- Methods -------------------- //


    /// Private copy-constructor
    IDynamicsWorld(const IDynamicsWorld& world);

    /// Private assignment operator
    IDynamicsWorld& operator=(const IDynamicsWorld& world);


    /// Initilisation array velocity and movement
    void InitVelocityArrays();


    /// Put bodies to sleep if needed.
    void UpdateSleepingBodies(scalar timeStep);


    //// Compute the islands of awake bodies.
    void ComputeIslands();


    /// Integrate the Velocity .
    virtual void IntegrateVelocities(scalar TimeStep);


    /// Integrate the motion .
    virtual void IntegratePositions(scalar TimeStep);


    virtual void UpdateBodiesState(scalar TimeStep);

    //***************************************************//


    //// Compute Solver
    void SolveContactsAndConstraints(scalar TimeStep);


    /// Solve the position error correction of the constraints
    void SolvePositionCorrection();



    void ResetBodiesForceAndTorque();


public:

    IDynamicsWorld( const Vector3& gravity );

    virtual ~IDynamicsWorld();

    //------------------------------------------------//

    void TransformInsert( IRigidBody* rigid_body , Transform transform)
    {
        u32 index = mMapBodyToConstrainedVelocityIndex[rigid_body];
        mConstrainedPositions[index].q = transform.GetRotation();
        mConstrainedPositions[index].x = transform.GetPosition();
    }

    //------------------------------------------------//


    ///  Destroy
    void Destroy();

    //***************************************************//

     /// Create a rigid body into the physics world.
    virtual IRigidBody *CreateRigidBody(const Transform& transform);


    /// Destroy a rigid body and all the joints which it belongs
    void DestroyBody(IRigidBody* rigidBody);


    /// Create a joint between two bodies in the world and return a pointer to the new joint
    IJoint* CreateJoint(const IJointInfo& jointInfo);


    /// Destroy a joint
    void DestroyJoint(IJoint* joint);


    /// Add the joint to the list of joints of the two bodies involved in the joint
    void AddJointToBody(IJoint* joint);


    //***************************************************//


    ///  Update Physics simulation - Real-Time ( Semi-AntiFixed timestep )
    void Update(scalar timeStep);

    ///  Update Physics simulation - Real-Time ( Fixed timestep )
    void UpdateFixedTimeStep(scalar timeStep);

    /// Get the number of iterations for the velocity constraint solver
    u32  GetNbIterationsVelocitySolver() const;

    /// Set the number of iterations for the velocity constraint solver
    void SetNbIterationsVelocitySolver(u32 nbIterations);

    /// Get the number of iterations for the position constraint solver
    u32  GetNbIterationsPositionSolver() const;

    /// Set the number of iterations for the position constraint solver
    void SetNbIterationsPositionSolver(u32 nbIterations);

    bool isSleepingEnabled() const;
    void SetIsSleepingEnabled(bool isSleepingEnabled);


    bool isGravityEnabled() const;
    void SetIsGravityEnabled(bool isGravityEnabled);
    void SetGravity(const Vector3 &gravity);
};


}

#endif // IDYNAMICSWORLD_H
