#include "IConstraintSolver.h"
#include "IIsland.h"
#include "IJoints/IJoint.h"

namespace IEngine
{


// Constructor
IConstraintSolver::IConstraintSolver(const std::map<IRigidBody*, u32>& mapBodyToVelocityIndex)
  : mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
    mIsWarmStartingActive(true),
    mConstraintSolverData(mapBodyToVelocityIndex)
{

}

// Destructor
IConstraintSolver::~IConstraintSolver()
{

}

// Initialize the constraint solver for a given island
void IConstraintSolver::InitializeForIsland(scalar dt, IIsland* island)
{

    assert(island != NULL);
    assert(island->GetNbBodies() > 0);
    assert(island->GetNbJoints() > 0);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;



    // For each joint of the island
    IJoint** joints = island->GetJoints();
    for (u32 i=0; i<island->GetNbJoints(); i++)
    {

        // Initialize the constraint before solving it
        joints[i]->InitBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive)
        {
            joints[i]->Warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void IConstraintSolver::SolveVelocityConstraints(IIsland* island)
{
    assert(island != NULL);
    assert(island->GetNbJoints() > 0);

    // For each joint of the island
    IJoint** joints = island->GetJoints();
    for (u32 i=0; i<island->GetNbJoints(); i++)
    {
        // Solve the constraint
        joints[i]->SolveVelocityConstraint(mConstraintSolverData);
    }
}





// Solve the position constraints
void IConstraintSolver::SolvePositionConstraints(IIsland* island)
{
    assert(island != NULL);
    assert(island->GetNbJoints() > 0);

    // For each joint of the island
    IJoint** joints = island->GetJoints();
    for (u32 i=0; i < island->GetNbJoints(); i++)
    {
        // Solve the constraint
        joints[i]->SolvePositionConstraint(mConstraintSolverData);
    }
}

}
