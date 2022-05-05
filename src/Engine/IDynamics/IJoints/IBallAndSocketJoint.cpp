#include "IBallAndSocketJoint.h"
#include "../IConstraintSolver.h"

namespace IEngine
{


// Static variables definition
const scalar IBallAndSocketJoint::BETA = scalar(0.2);

// Constructor
IBallAndSocketJoint::IBallAndSocketJoint(const IBallAndSocketJointInfo& jointInfo)
:IJoint(jointInfo), mImpulse(Vector3(0, 0, 0))
{

    isWarmStartingActive = true;

    // Compute the local-space anchor point the constraint error
    mLocalAnchorPointBody1 = mBody1->mTransform.GetInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = mBody2->mTransform.GetInverse() * jointInfo.anchorPointWorldSpace;

    softness = 0.0001f;

}


// Destructor
IBallAndSocketJoint::~IBallAndSocketJoint()
{

}



// Initialize before solving the constraint
void IBallAndSocketJoint::InitBeforeSolve( const IConstraintSolverData &constraintSolverData  )
{

         // Initialize the bodies index in the velocity array
          mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
          mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

         // Get the bodies center of mass and orientations
          const Vector3& x1 = mBody1->mCenterOfMassWorld;
          const Vector3& x2 = mBody2->mCenterOfMassWorld;
          const Quaternion& orientationBody1 = mBody1->mTransform.GetRotation();
          const Quaternion& orientationBody2 = mBody2->mTransform.GetRotation();

          // Get the inertia tensor of bodies
          mI1 = mBody1->GetInertiaTensorInverseWorld();
          mI2 = mBody2->GetInertiaTensorInverseWorld();

          // Compute the vector from body center to the anchor point in world-space
          mR1World = orientationBody1.GetRotMatrix() * mLocalAnchorPointBody1;
          mR2World = orientationBody2.GetRotMatrix() * mLocalAnchorPointBody2;



          // Compute the corresponding skew-symmetric matrices
          Matrix3 skewSymmetricMatrixU1= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
          Matrix3 skewSymmetricMatrixU2= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);

          // Compute the matrix K=JM^-1J^t (3x3 matrix)
          scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
          Matrix3 massMatrix = Matrix3(inverseMassBodies, 0, 0,
                                             0, inverseMassBodies, 0,
                                             0, 0, inverseMassBodies) +
                                             skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.GetTranspose() +
                                             skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.GetTranspose();

          // Compute the inverse mass matrix K^-1
          mInverseMassMatrix.SetToZero();
          if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC )
          {
                  mInverseMassMatrix = massMatrix.GetInverse();
          }

          // Compute the bias "b" of the constraint
          mBiasVector.SetToZero();
          if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS )
          {
              scalar biasFactor = BETA/constraintSolverData.timeStep;
              mBiasVector = biasFactor * (x2 + mR2World -
                                          x1 - mR1World);
          }


          // If warm-starting is not enabled
          if (!isWarmStartingActive)
          {
              // Reset the accumulated impulse
              mImpulse.SetToZero();
          }


}




// Warm start the constraint (apply the previous impulse at the beginning of the step)
void IBallAndSocketJoint::Warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;



    // Compute the impulse P=J^T * lambda for the body 1
    const Vector3 linearImpulseBody1 = -mImpulse;
    const Vector3 linearImpulseBody2 =  mImpulse;

    // Compute the impulse P=J^T * lambda for the body 2
    const Vector3 angularImpulseBody1 =  mImpulse.Cross(mR1World);
    const Vector3 angularImpulseBody2 = -mImpulse.Cross(mR2World);


    // Apply the impulse to the body 1
    v1 += mBody1->mMassInverse * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body to the body 2
    v2 += mBody2->mMassInverse * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;


}




// Solve the velocity constraint
void IBallAndSocketJoint::SolveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
{


  // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Compute J*v
    const Vector3 Jv = v2 + w2.Cross(mR2World) -
                       v1 - w1.Cross(mR1World);



    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = mInverseMassMatrix * (-Jv - mBiasVector);

    mImpulse += (deltaLambda - (deltaLambda.GetUnit() * mImpulse.Length() * softness));

    // Compute the impulse P=J^T * lambda for the body 1
    const Vector3 linearImpulseBody1 = -deltaLambda;
    const Vector3 linearImpulseBody2 =  deltaLambda;

    // Compute the impulse P=J^T * lambda for the body 2
    const Vector3 angularImpulseBody1 =  deltaLambda.Cross(mR1World);
    const Vector3 angularImpulseBody2 = -deltaLambda.Cross(mR2World);


    // Apply the impulse to the body 1
    v1 += mBody1->mMassInverse * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    v2 += mBody2->mMassInverse * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;


}





// Solve the position constraint (for position error correction)
void IBallAndSocketJoint::SolvePositionConstraint(const IConstraintSolverData &constraintSolverData)
{


  // If the error position correction technique is not the non-linear-gauss-seidel, we do
  // do not execute this method
  if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;


  // Get the bodies center of mass and orientations
  Vector3&    x1 = constraintSolverData.Positions[mIndexBody1].x;
  Vector3&    x2 = constraintSolverData.Positions[mIndexBody2].x;
  Quaternion& q1 = constraintSolverData.Positions[mIndexBody1].q;
  Quaternion& q2 = constraintSolverData.Positions[mIndexBody2].q;

  // Get the inverse mass and inverse inertia tensors of the bodies
  scalar inverseMassBody1 = mBody1->mMassInverse;
  scalar inverseMassBody2 = mBody2->mMassInverse;

  // Recompute the inverse inertia tensors
  mI1 = mBody1->GetInertiaTensorInverseWorld();
  mI2 = mBody2->GetInertiaTensorInverseWorld();

  // Compute the vector from body center to the anchor point in world-space
  mR1World = q1.GetRotMatrix() * mLocalAnchorPointBody1;
  mR2World = q2.GetRotMatrix() * mLocalAnchorPointBody2;

  // Compute the corresponding skew-symmetric matrices
  Matrix3 skewSymmetricMatrixU1= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
  Matrix3 skewSymmetricMatrixU2= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);

  // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
  scalar inverseMassBodies = inverseMassBody1 + inverseMassBody2;
  Matrix3 massMatrix = Matrix3(inverseMassBodies, 0, 0,
                                   0, inverseMassBodies, 0,
                                   0, 0, inverseMassBodies) +
                                   skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.GetTranspose() +
                                   skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.GetTranspose();

  mInverseMassMatrix.SetToZero();
  if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
  {
      mInverseMassMatrix = massMatrix.GetInverse();
  }


  // Compute the constraint error (value of the C(x) function)
  Vector3 constraintError = (x2 + mR2World -
                              x1 - mR1World);

//  // Relaxation offset damping to the constraint error
//  scalar dampingRelaxation = 0;//0.0001;
//  if( constraintError.length() > dampingRelaxation)
//  {
//      constraintError *= scalar(1.0 - dampingRelaxation);
//  }
//  else
//  {
//      constraintError = Vector3::ZERO;
//  }



  // Compute the Lagrange multiplier lambda
  // TODO : Do not solve the system by computing the inverse each time and multiplying with the
  //        right-hand side vector but instead use a method to directly solve the linear system.
  const Vector3 lambda = mInverseMassMatrix * ( -constraintError  );




  // Compute the impulse of body
  const Vector3 linearImpulseBody1 = -lambda;
  const Vector3 linearImpulseBody2 =  lambda;

  // Compute the impulse of body
  const Vector3 angularImpulseBody1 =  lambda.Cross(mR1World);
  const Vector3 angularImpulseBody2 = -lambda.Cross(mR2World);


  // Compute the pseudo velocity of body 1
  const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
  const Vector3 w1 = mI1 * angularImpulseBody1;

  // Compute the pseudo velocity of body 2
  const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
  const Vector3 w2 = mI2 * angularImpulseBody2;


  /**/
  // Update the body center of mass and orientation of body 1
  x1 += v1;
  q1 += Quaternion(0, w1) * q1 * scalar(0.5);
  q1.Normalize();
  // Update the body position/orientation of body 2
  x2 += v2;
  q2 += Quaternion(0, w2) * q2 * scalar(0.5);
  q2.Normalize();

  // Update the body center of mass and orientation of body 1
//  x1 = Transform::integrateLinear(  x1 , v1 , 1.0);
//  q1 = Transform::integrateAngular( q1 , w1 , 1.0);
//  // Update the body position/orientation of body 2
//  x2 = Transform::integrateLinear(  x2 , v2 , 1.0);
//  q2 = Transform::integrateAngular( q2 , w2 , 1.0);

}



}
