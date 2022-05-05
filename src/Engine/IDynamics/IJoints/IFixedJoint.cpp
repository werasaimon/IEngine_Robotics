#include "IFixedJoint.h"
#include "../IConstraintSolver.h"


namespace IEngine
{


// Static variables definition
const scalar  IFixedJoint::BETA = scalar(0.02);

// Constructor
IFixedJoint:: IFixedJoint(const IFixedJointInfo& jointInfo)
   :IJoint(jointInfo), mImpulseTranslation(0, 0, 0), mImpulseRotation(0, 0, 0)
{

    isSplitActive = true;
    isWarmStartingActive = true;


    // Compute the local-space anchor point for each body
    const Transform& transform1 = mBody1->GetTransform();
    const Transform& transform2 = mBody2->GetTransform();
    mLocalAnchorPointBody1 = transform1.GetInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.GetInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the inverse of the initial orientation difference between the two bodies
    mInitOrientationDifferenceInv = transform2.GetRotation() *
                                    transform1.GetRotation().GetInverse();
    mInitOrientationDifferenceInv.Normalize();
    mInitOrientationDifferenceInv.GetInverse();

}

// Destructor
IFixedJoint::~IFixedJoint()
{

}

// Initialize before solving the constraint
void  IFixedJoint::InitBeforeSolve(const IConstraintSolverData &constraintSolverData  )
{

    // Initialize the bodies index in the velocity array
    mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
    mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

    // Get the bodies positions and orientations
    const Vector3& x1 = mBody1->mCenterOfMassWorld;
    const Vector3& x2 = mBody2->mCenterOfMassWorld;
    const Quaternion& orientationBody1 = mBody1->GetTransform().GetRotation();
    const Quaternion& orientationBody2 = mBody2->GetTransform().GetRotation();

    // Get the inertia tensor of bodies
    mI1 = mBody1->GetInertiaTensorInverseWorld();
    mI2 = mBody2->GetInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = orientationBody1.GetRotMatrix() * mLocalAnchorPointBody1;
    mR2World = orientationBody2.GetRotMatrix() * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    Matrix3 skewSymmetricMatrixU1=Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3 skewSymmetricMatrixU2=Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3 massMatrix =Matrix3(inverseMassBodies, 0, 0,
                                      0, inverseMassBodies, 0,
                                      0, 0, inverseMassBodies) +
                                      skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.GetTranspose() +
                                      skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.GetTranspose();

    // Compute the inverse mass matrix K^-1 for the 3 translation constraints
    mInverseMassMatrixTranslation.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixTranslation = massMatrix.GetInverse();
    }

    // Compute the bias "b" of the constraint for the 3 translation constraints
    scalar biasFactor = (BETA / constraintSolverData.timeStep);
    mBiasTranslation.SetToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        mBiasTranslation = biasFactor * (x2 + mR2World -
                                         x1 - mR1World);
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.GetInverse();
    }

    // Compute the bias "b" for the 3 rotation constraints
    mBiasRotation.SetToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.GetInverse();
        currentOrientationDifference.Normalize();
        const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        mBiasRotation = biasFactor * scalar(2.0) * qError.GetV();
    }

    // If warm-starting is not enabled
    if (!isWarmStartingActive)
    {
        // Reset the accumulated impulses
        mImpulseTranslation.SetToZero();
        mImpulseRotation.SetToZero();
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void  IFixedJoint::Warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;


    // Get the inverse mass of the bodies
    const scalar inverseMassBody1 = mBody1->mMassInverse;
    const scalar inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
    Vector3 linearImpulseBody1 = -mImpulseTranslation;
    Vector3 angularImpulseBody1 = mImpulseTranslation.Cross(mR1World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
    Vector3 angularImpulseBody2 = -mImpulseTranslation.Cross(mR2World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
    angularImpulseBody2 += mImpulseRotation;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * mImpulseTranslation;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void  IFixedJoint::SolveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;


    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = mBody1->mMassInverse;
    scalar inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 3 translation constraints
    const Vector3 JvTranslation = v2 + w2.Cross(mR2World) -
                                   v1 - w1.Cross(mR1World);

    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambda = mInverseMassMatrixTranslation * (-JvTranslation - mBiasTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for body 1
    const Vector3 linearImpulseBody1 = -deltaLambda;
    Vector3 angularImpulseBody1 = deltaLambda.Cross(mR1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda  for body 2
    const Vector3 angularImpulseBody2 = -deltaLambda.Cross(mR2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambda;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotation * (-JvRotation - mBiasRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body 1
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    w2 += mI2 * deltaLambda2;
}

// Solve the position constraint (for position error correction)
void IFixedJoint::SolvePositionConstraint(const IConstraintSolverData &constraintSolverData)
{

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;


    // Get the bodies positions and orientations
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
    Matrix3 skewSymmetricMatrixU1=Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3 skewSymmetricMatrixU2=Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);

    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3 massMatrix =Matrix3(inverseMassBodies, 0, 0,
                                      0, inverseMassBodies, 0,
                                      0, 0, inverseMassBodies) +
            skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.GetTranspose() +
            skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.GetTranspose();
    mInverseMassMatrixTranslation.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixTranslation = massMatrix.GetInverse();
    }

    // Compute position error for the 3 translation constraints
    const Vector3 errorTranslation = x2 + mR2World -
            x1 - mR1World;

    // Compute the Lagrange multiplier lambda
    const Vector3 lambdaTranslation = mInverseMassMatrixTranslation * (-errorTranslation);

    // Compute the impulse of body 1
    Vector3 linearImpulseBody1 = -lambdaTranslation;
    Vector3 angularImpulseBody1 = lambdaTranslation.Cross(mR1World);

    // Compute the pseudo velocity of body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.Normalize();

    // Compute the impulse of body 2
    Vector3 angularImpulseBody2 = -lambdaTranslation.Cross(mR2World);

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * lambdaTranslation;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.Normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.GetInverse();
    }

    // Compute the position error for the 3 rotation constraints
    Quaternion currentOrientationDifference = q2 * q1.GetInverse();
    currentOrientationDifference.Normalize();
    const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
    const Vector3 errorRotation = scalar(2.0) * qError.GetV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 lambdaRotation = mInverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Compute the pseudo velocity of body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.Normalize();

    // Compute the pseudo velocity of body 2
    w2 = mI2 * lambdaRotation;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.Normalize();

}

}
