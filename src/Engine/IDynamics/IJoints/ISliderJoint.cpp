#include "ISliderJoint.h"
#include "../IConstraintSolver.h"

namespace IEngine
{


// Static variables definition
const scalar ISliderJoint::BETA = scalar(0.2);

// Constructor
ISliderJoint::ISliderJoint(const ISliderJointInfo& jointInfo)
            : IJoint(jointInfo), mImpulseTranslation(0, 0), mImpulseRotation(0, 0, 0),
              mImpulseLowerLimit(0), mImpulseUpperLimit(0), mImpulseMotor(0),
              mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
              mLowerLimit(jointInfo.minTranslationLimit),
              mUpperLimit(jointInfo.maxTranslationLimit), mIsLowerLimitViolated(false),
              mIsUpperLimitViolated(false), mMotorSpeed(jointInfo.motorSpeed),
              mMaxMotorForce(jointInfo.maxMotorForce){

    assert(mUpperLimit >= 0.0);
    assert(mLowerLimit <= 0.0);
    assert(mMaxMotorForce >= 0.0);

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

    // Compute the slider axis in local-space of body 1
    mSliderAxisBody1 = mBody1->GetTransform().GetRotation().GetInverse().GetRotMatrix() *  jointInfo.sliderAxisWorldSpace;
    mSliderAxisBody1.Normalize();
}

// Destructor
ISliderJoint::~ISliderJoint()
{

}

// Initialize before solving the constraint
void ISliderJoint::InitBeforeSolve(const IConstraintSolverData &constraintSolverData )
{

  // Initialize the bodies index in the veloc ity array
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

    // Vector from body center to the anchor point
    mR1 = orientationBody1.GetRotMatrix() * mLocalAnchorPointBody1;
    mR2 = orientationBody2.GetRotMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = orientationBody1.GetRotMatrix() * mSliderAxisBody1;
    mSliderAxisWorld.Normalize();
    mN1 = mSliderAxisWorld.GetOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.Cross(mN1);

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.Dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    bool oldIsLowerLimitViolated = mIsLowerLimitViolated;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    if (mIsLowerLimitViolated != oldIsLowerLimitViolated)
    {
        mImpulseLowerLimit = 0.0;
    }
    bool oldIsUpperLimitViolated = mIsUpperLimitViolated;
    mIsUpperLimitViolated = upperLimitError <= 0;
    if (mIsUpperLimitViolated != oldIsUpperLimitViolated)
    {
        mImpulseUpperLimit = 0.0;
    }

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.Cross(mN1);
    mR2CrossN2 = mR2.Cross(mN2);
    mR2CrossSliderAxis = mR2.Cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).Cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).Cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).Cross(mSliderAxisWorld);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.Dot(I1R1PlusUCrossN1) + mR2CrossN1.Dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.Dot(I1R1PlusUCrossN2) + mR2CrossN1.Dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.Dot(I1R1PlusUCrossN1) + mR2CrossN2.Dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.Dot(I1R1PlusUCrossN2) + mR2CrossN2.Dot(I2R2CrossN2);
    Matrix2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.GetInverse();
    }

    // Compute the bias "b" of the translation constraint
    mBTranslation.SetToZero();
    scalar biasFactor = (BETA/constraintSolverData.timeStep);
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        mBTranslation.x = u.Dot(mN1);
        mBTranslation.y = u.Dot(mN2);
        mBTranslation *= biasFactor;
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.GetInverse();
    }

    // Compute the bias "b" of the rotation constraint
    mBRotation.SetToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        Quaternion currentOrientationDifference = orientationBody2 * orientationBody1.GetInverse();
        currentOrientationDifference.Normalize();
        const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        mBRotation = biasFactor * scalar(2.0) * qError.GetV();
    }

    // If the limits are enabled
    if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))
    {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                  mR1PlusUCrossSliderAxis.Dot(mI1 * mR1PlusUCrossSliderAxis) +
                                  mR2CrossSliderAxis.Dot(mI2 * mR2CrossSliderAxis);
        mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                  scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);

        // Compute the bias "b" of the lower limit constraint
        mBLowerLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBLowerLimit = biasFactor * lowerLimitError;
        }

        // Compute the bias "b" of the upper limit constraint
        mBUpperLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBUpperLimit = biasFactor * upperLimitError;
        }
    }

    // If the motor is enabled
    if (mIsMotorEnabled)
    {
        // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
        mInverseMassMatrixMotor = mBody1->mMassInverse + mBody2->mMassInverse;
        mInverseMassMatrixMotor = (mInverseMassMatrixMotor > 0.0) ?
                    scalar(1.0) / mInverseMassMatrixMotor : scalar(0.0);
    }

    // If warm-starting is not enabled
    if (!isWarmStartingActive)
    {
        // Reset all the accumulated impulses
        mImpulseTranslation.SetToZero();
        mImpulseRotation.SetToZero();
        mImpulseLowerLimit = 0.0;
        mImpulseUpperLimit = 0.0;
        mImpulseMotor = 0.0;
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void ISliderJoint::Warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Get the inverse mass and inverse inertia tensors of the bodies
    const scalar inverseMassBody1 = mBody1->mMassInverse;
    const scalar inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    scalar impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
    Vector3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    Vector3 impulseMotor = mImpulseMotor * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    Vector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x -
                                   mR1PlusUCrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    linearImpulseBody1 += linearImpulseLimits;
    angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    linearImpulseBody1 += impulseMotor;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    Vector3 linearImpulseBody2 = mN1 * mImpulseTranslation.x + mN2 * mImpulseTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x +
                                  mR2CrossN2 * mImpulseTranslation.y;



    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 += mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    linearImpulseBody2 += -linearImpulseLimits;
    angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;



    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    linearImpulseBody2 += -impulseMotor;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void ISliderJoint::SolveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
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

    // Compute J*v for the 2 translation constraints
    const scalar el1 = -mN1.Dot(v1) - w1.Dot(mR1PlusUCrossN1) + mN1.Dot(v2) + w2.Dot(mR2CrossN1);
    const scalar el2 = -mN2.Dot(v1) - w1.Dot(mR1PlusUCrossN2) + mN2.Dot(v2) + w2.Dot(mR2CrossN2);
    const Vector2 JvTranslation(el1, el2);

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation -mBTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x -
                                    mR1PlusUCrossN2 * deltaLambda.y;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * deltaLambda.x + mN2 * deltaLambda.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const Vector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body to body 1
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {
            // Compute J*v for the lower limit constraint
            const scalar JvLowerLimit = mSliderAxisWorld.Dot(v2) + mR2CrossSliderAxis.Dot(w2) -
                                        mSliderAxisWorld.Dot(v1) - mR1PlusUCrossSliderAxis.Dot(w1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit -mBLowerLimit);
            scalar lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = IMax(mImpulseLowerLimit + deltaLambdaLower, scalar(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;



            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;



            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;


        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute J*v for the upper limit constraint
            const scalar JvUpperLimit = mSliderAxisWorld.Dot(v1) + mR1PlusUCrossSliderAxis.Dot(w1)
                                      - mSliderAxisWorld.Dot(v2) - mR2CrossSliderAxis.Dot(w2);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit -mBUpperLimit);
            scalar lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = IMax(mImpulseUpperLimit + deltaLambdaUpper, scalar(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;



            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;



            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2  = -deltaLambdaUpper * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    if (mIsMotorEnabled)
    {

        // Compute J*v for the motor
        const scalar JvMotor = mSliderAxisWorld.Dot(v1) - mSliderAxisWorld.Dot(v2);

        // Compute the Lagrange multiplier lambda for the motor
        const scalar maxMotorImpulse = mMaxMotorForce;
        scalar deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
        scalar lambdaTemp = mImpulseMotor;
        mImpulseMotor = IClamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void ISliderJoint::SolvePositionConstraint(const IConstraintSolverData &constraintSolverData)
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

    // Recompute the inertia tensor of bodies
    mI1 = mBody1->GetInertiaTensorInverseWorld();
    mI2 = mBody2->GetInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = q1.GetRotMatrix() * mLocalAnchorPointBody1;
    mR2 = q2.GetRotMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = q1.GetRotMatrix() * mSliderAxisBody1;
    mSliderAxisWorld.Normalize();
    mN1 = mSliderAxisWorld.GetOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.Cross(mN1);

    //Vector3::BiUnitOrthogonalVector(mSliderAxisWorld , mN1 , mN2 );

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.Dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.Cross(mN1);
    mR2CrossN2 = mR2.Cross(mN2);
    mR2CrossSliderAxis = mR2.Cross(mSliderAxisWorld);
    const Vector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).Cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).Cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).Cross(mSliderAxisWorld);

    // --------------- Translation Constraints --------------- //

    // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    Vector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    Vector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    Vector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    Vector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.Dot(I1R1PlusUCrossN1) + mR2CrossN1.Dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.Dot(I1R1PlusUCrossN2) + mR2CrossN1.Dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.Dot(I1R1PlusUCrossN1) + mR2CrossN2.Dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.Dot(I1R1PlusUCrossN2) + mR2CrossN2.Dot(I2R2CrossN2);
    Matrix2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.GetInverse();
    }

    // Compute the position error for the 2 translation constraints
    const Vector2 translationError(u.Dot(mN1), u.Dot(mN2));

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    Vector2 lambdaTranslation = mInverseMassMatrixTranslationConstraint * (-translationError);

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const Vector3 linearImpulseBody1 = -mN1 * lambdaTranslation.x -
                                        mN2 * lambdaTranslation.y;

    Vector3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x -
                                   mR1PlusUCrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 1
    const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
    Vector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.Normalize();

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const Vector3 linearImpulseBody2 = mN1 * lambdaTranslation.x + mN2 * lambdaTranslation.y;
    Vector3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x +
                                  mR2CrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 2
    const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.Normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.GetInverse();
    }

    // Compute the position error for the 3 rotation constraints
    Quaternion currentOrientationDifference = q2 * q1.GetInverse();
    currentOrientationDifference.Normalize();
    const Quaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
    const Vector3 errorRotation = scalar(2.0) * qError.GetV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector3 lambdaRotation = mInverseMassMatrixRotationConstraint * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Apply the impulse to the body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.Normalize();

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = lambdaRotation;

    // Apply the impulse to the body 2
    w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.Normalize();

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        if (mIsLowerLimitViolated || mIsUpperLimitViolated)
        {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                    mR1PlusUCrossSliderAxis.Dot(mI1 * mR1PlusUCrossSliderAxis) +
                                    mR2CrossSliderAxis.Dot(mI2 * mR2CrossSliderAxis);
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                      scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.Normalize();

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.Normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const Vector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const Vector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.Normalize();

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const Vector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.Normalize();
        }
    }

}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void ISliderJoint::EnableLimit(bool isLimitEnabled)
{
    if (isLimitEnabled != mIsLimitEnabled)
    {
        mIsLimitEnabled = isLimitEnabled;

        // Reset the limits
        ResetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *                       otherwise
 */
void ISliderJoint::EnableMotor(bool isMotorEnabled)
{

    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->SetIsSleeping(false);
    mBody2->SetIsSleeping(false);
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
scalar ISliderJoint::GetTranslation() const
{

    // TODO : Check if we need to compare rigid body position or center of mass here

    const Vector3& x1 = mBody1->mCenterOfMassWorld;
    const Vector3& x2 = mBody2->mCenterOfMassWorld;
    const Quaternion& q1 = mBody1->GetTransform().GetRotation();
    const Quaternion& q2 = mBody2->GetTransform().GetRotation();

    // Compute the two anchor points in world-space coordinates
    const Vector3 anchorBody1 = x1 + q1.GetRotMatrix() * mLocalAnchorPointBody1;
    const Vector3 anchorBody2 = x2 + q2.GetRotMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const Vector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    Vector3 sliderAxisWorld = q1.GetRotMatrix() * mSliderAxisBody1;
    sliderAxisWorld.Normalize();

    // Compute and return the translation value
    return u.Dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void ISliderJoint::SetMinTranslationLimit(scalar lowerLimit)
{

    assert(lowerLimit <= mUpperLimit);

    if (lowerLimit != mLowerLimit)
    {
        mLowerLimit = lowerLimit;

        // Reset the limits
        ResetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void ISliderJoint::SetMaxTranslationLimit(scalar upperLimit)
{

    assert(mLowerLimit <= upperLimit);

    if (upperLimit != mUpperLimit)
    {
        mUpperLimit = upperLimit;

        // Reset the limits
        ResetLimits();
    }
}

// Reset the limits
void ISliderJoint::ResetLimits()
{

    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->SetIsSleeping(false);
    mBody2->SetIsSleeping(false);
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void ISliderJoint::SetMotorSpeed(scalar motorSpeed)
{
    if (motorSpeed != mMotorSpeed)
    {
        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->SetIsSleeping(false);
        mBody2->SetIsSleeping(false);
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void ISliderJoint::SetMaxMotorForce(scalar maxMotorForce)
{
    if (maxMotorForce != mMaxMotorForce)
    {
        assert(mMaxMotorForce >= 0.0);
        mMaxMotorForce = maxMotorForce;

        // Wake up the two bodies of the joint
        mBody1->SetIsSleeping(false);
        mBody2->SetIsSleeping(false);
    }
}




}
