#include "IHingeJoint.h"
#include "../IConstraintSolver.h"


namespace IEngine
{



// Static variables definition
//scalar IHingeJoint::BETA = scalar(0.2);

// Constructor
float IHingeJoint::getPositionMotor() const
{
    return PositionMotor;
}

IHingeJoint::IHingeJoint(const IHingeJointInfo& jointInfo)
           :IJoint(jointInfo),
             PositionMotor(0),
             mImpulseTranslation(0, 0, 0),
             mImpulseRotation(0, 0),
             mImpulseLowerLimit(0),
             mImpulseUpperLimit(0),
             mImpulseMotor(0),
             mIsLimitEnabled(jointInfo.isLimitEnabled),
             mIsMotorEnabled(jointInfo.isMotorEnabled),
             mLowerLimit(jointInfo.minAngleLimit),
             mUpperLimit(jointInfo.maxAngleLimit),
             mIsLowerLimitViolated(false),
             mIsUpperLimitViolated(false),
             mMotorSpeed(jointInfo.motorSpeed),
             mMaxMotorTorque(jointInfo.maxMotorTorque)
{

    assert(mLowerLimit <= 0 && mLowerLimit >= -2.0 * PI);
    assert(mUpperLimit >= 0 && mUpperLimit <=  2.0 * PI);

    isSplitActive = true;
    isWarmStartingActive = true;


    // Compute the local-space anchor point for each body
    Transform transform1 = mBody1->GetTransform();
    Transform transform2 = mBody2->GetTransform();
    mLocalAnchorPointBody1 = transform1.GetInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.GetInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the local-space hinge axis
    mHingeLocalAxisBody1 = transform1.GetRotation().GetInverse().GetRotMatrix() * jointInfo.rotationAxisWorld;
    mHingeLocalAxisBody2 = transform2.GetRotation().GetInverse().GetRotMatrix() * jointInfo.rotationAxisWorld;
    mHingeLocalAxisBody1.Normalize();
    mHingeLocalAxisBody2.Normalize();

    // Compute the inverse of the initial orientation difference between the two bodies
    mInitOrientationDifferenceInv = transform2.GetRotation() *
                                    transform1.GetRotation().GetInverse();
    mInitOrientationDifferenceInv.Normalize();
    mInitOrientationDifferenceInv.GetInverse();

    BETA = scalar(0.2);
}

// Destructor
IHingeJoint::~IHingeJoint()
{

}

// Initialize before solving the constraint
void IHingeJoint::InitBeforeSolve( const IConstraintSolverData& constraintSolverData )
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

    // Compute the current angle around the hinge axis
    scalar hingeAngle = ComputeCurrentHingeAngle(orientationBody1, orientationBody2);

    // Check if the limit constraints are violated or not
    scalar lowerLimitError = hingeAngle - mLowerLimit;
    scalar upperLimitError = mUpperLimit - hingeAngle;
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

    // Compute vectors needed in the Jacobian
            mA1 = orientationBody1.GetRotMatrix() * mHingeLocalAxisBody1;
    Vector3 a2 = orientationBody2.GetRotMatrix() * mHingeLocalAxisBody2;
    mA1.Normalize();
    a2.Normalize();


    const Vector3 b2 = a2.GetOneUnitOrthogonalVector();
    const Vector3 c2 = a2.Cross(b2);

//    Vector3 b2 , c2;
//    Vector3::btPlaneSpace1(a2 , c2 , b2);

    mB2CrossA1 = b2.Cross(mA1);
    mC2CrossA1 = c2.Cross(mA1);

    // Compute the corresponding skew-symmetric matrices
    Matrix3 skewSymmetricMatrixU1= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3 skewSymmetricMatrixU2= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the inverse mass matrix K=JM^-1J^t for the 3 translation constraints (3x3 matrix)
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3 massMatrix = Matrix3(inverseMassBodies, 0, 0,
                                       0, inverseMassBodies, 0,
                                       0, 0, inverseMassBodies) +
                                       skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.GetTranspose() +
                                       skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.GetTranspose();

    mInverseMassMatrixTranslation.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixTranslation = massMatrix.GetInverse();
    }


    // Compute the bias "b" of the translation constraints
    mBTranslation.SetToZero();
    scalar biasFactor = (BETA / constraintSolverData.timeStep);
    if ( mPositionCorrectionTechnique == BAUMGARTE_JOINTS )
    {
        mBTranslation = biasFactor * (x2 + mR2World -
                                      x1 - mR1World);

    }

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 I1B2CrossA1 = mI1 * mB2CrossA1;
    Vector3 I1C2CrossA1 = mI1 * mC2CrossA1;
    Vector3 I2B2CrossA1 = mI2 * mB2CrossA1;
    Vector3 I2C2CrossA1 = mI2 * mC2CrossA1;
    const scalar el11 = mB2CrossA1.Dot(I1B2CrossA1) +  mB2CrossA1.Dot(I2B2CrossA1);
    const scalar el12 = mB2CrossA1.Dot(I1C2CrossA1) +  mB2CrossA1.Dot(I2C2CrossA1);
    const scalar el21 = mC2CrossA1.Dot(I1B2CrossA1) +  mC2CrossA1.Dot(I2B2CrossA1);
    const scalar el22 = mC2CrossA1.Dot(I1C2CrossA1) +  mC2CrossA1.Dot(I2C2CrossA1);
    const Matrix2 matrixKRotation(el11, el12, el21, el22);
    mInverseMassMatrixRotation.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = matrixKRotation.GetInverse();
    }

    // Compute the bias "b" of the rotation constraints
    mBRotation.SetToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS )
    {
        mBRotation = biasFactor * Vector2(mA1.Dot(b2), mA1.Dot(c2));
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

    // If the motor or limits are enabled
    if (mIsMotorEnabled || (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)))
    {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits and motor (1x1 matrix)
        mInverseMassMatrixLimitMotor = mA1.Dot(mI1 * mA1) + mA1.Dot(mI2 * mA1);
        mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0) ?
                                  scalar(1.0) / mInverseMassMatrixLimitMotor : scalar(0.0);

        if (mIsLimitEnabled)
        {
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
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void IHingeJoint::Warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;


    // Get the inverse mass and inverse inertia tensors of the bodies
    const scalar inverseMassBody1 = mBody1->mMassInverse;
    const scalar inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints
    Vector3 rotationImpulse = -mB2CrossA1 * mImpulseRotation.x -
                               mC2CrossA1 * mImpulseRotation.y;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints
    const Vector3 limitsImpulse = (mImpulseUpperLimit - mImpulseLowerLimit) * mA1;


    // Compute the impulse P=J^T * lambda for the motor constraint
    //const Vector3 motorImpulse = -mImpulseMotor * mA1;
    const Vector3 motorImpulse =  -mImpulseMotor * mA1;




    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 1
    Vector3 linearImpulseBody1 = -mImpulseTranslation;
    Vector3 angularImpulseBody1 = mImpulseTranslation.Cross(mR1World);

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 += rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    angularImpulseBody1 += limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    angularImpulseBody1 += motorImpulse;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;




    // Compute the impulse P=J^T * lambda for the 3 translation constraints of body 2
    Vector3 linearImpulseBody2  =  mImpulseTranslation;
    Vector3 angularImpulseBody2 = -mImpulseTranslation.Cross(mR2World);

    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 += -rotationImpulse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    angularImpulseBody2 += -limitsImpulse;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    angularImpulseBody2 += -motorImpulse;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;


}

// Solve the velocity constraint
void IHingeJoint::SolveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
{

    /**/

    // Get the velocities
    Vector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    Vector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    Vector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    Vector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = mBody1->mMassInverse;
    scalar inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v
    const Vector3 JvTranslation = v2 + w2.Cross(mR2World) -
                                  v1 - w1.Cross(mR1World);


    // Compute the Lagrange multiplier lambda
    const Vector3 deltaLambdaTranslation = mInverseMassMatrixTranslation * (-JvTranslation - mBTranslation);
    mImpulseTranslation += deltaLambdaTranslation;


    // Compute the impulse P=J^T * lambda of body 1
    const Vector3 linearImpulseBody1 = -deltaLambdaTranslation;
    Vector3 angularImpulseBody1 = deltaLambdaTranslation.Cross(mR1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;




    // Compute the impulse P=J^T * lambda of body 2
    const Vector3 linearImpulseBody2 = deltaLambdaTranslation;
    Vector3 angularImpulseBody2 = -deltaLambdaTranslation.Cross(mR2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 2 rotation constraints
    const Vector2 JvRotation(-mB2CrossA1.Dot(w1) + mB2CrossA1.Dot(w2),
                             -mC2CrossA1.Dot(w1) + mC2CrossA1.Dot(w2));

    // Compute the Lagrange multiplier lambda for the 2 rotation constraints
    Vector2 deltaLambdaRotation = mInverseMassMatrixRotation * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambdaRotation;



    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 1
    angularImpulseBody1 = -mB2CrossA1 * deltaLambdaRotation.x -
                           mC2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 1
    w1 += mI1 * angularImpulseBody1;



    // Compute the impulse P=J^T * lambda for the 2 rotation constraints of body 2
    angularImpulseBody2 = mB2CrossA1 * deltaLambdaRotation.x +
                          mC2CrossA1 * deltaLambdaRotation.y;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;


    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {

            // Compute J*v for the lower limit constraint
            const scalar JvLowerLimit = (w2 - w1).Dot(mA1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit -mBLowerLimit);
            scalar lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = IMax(mImpulseLowerLimit + deltaLambdaLower, scalar(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;


            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const Vector3 angularImpulseBody1 = -deltaLambdaLower * mA1;
            // Apply the impulse to the body 1
            w1 += mI1 * angularImpulseBody1;


            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const Vector3 angularImpulseBody2 = deltaLambdaLower * mA1;
            // Apply the impulse to the body 2
            w2 += mI2 * angularImpulseBody2;

        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute J*v for the upper limit constraint
            const scalar JvUpperLimit = -(w2 - w1).Dot(mA1);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit -mBUpperLimit);
            scalar lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = IMax(mImpulseUpperLimit + deltaLambdaUpper, scalar(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;


            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const Vector3 angularImpulseBody1 = deltaLambdaUpper * mA1;
            // Apply the impulse to the body 1
            w1 += mI1 * angularImpulseBody1;


            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const Vector3 angularImpulseBody2 = -deltaLambdaUpper * mA1;
            // Apply the impulse to the body 2
            w2 += mI2 * angularImpulseBody2;

        }
    }

    // --------------- Motor --------------- //

    /**/
    // If the motor is enabled
    if (mIsMotorEnabled)
    {
        // Compute J*v for the motor
        const scalar JvMotor = mA1.Dot(w2 - w1);


        // Compute the Lagrange multiplier lambda for the motor
        const scalar maxMotorImpulse = mMaxMotorTorque;
        scalar deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
        scalar lambdaTemp = mImpulseMotor;
        mImpulseMotor = IClamp( mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse );
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;


        // Compute the impulse P=J^T * lambda for the motor of body 1
        const Vector3 angularImpulseBody1 = -deltaLambdaMotor * mA1;
        // Apply the impulse to the body 1
        w1 += mI1 * angularImpulseBody1;


        // Compute the impulse P=J^T * lambda for the motor of body 2
        const Vector3 angularImpulseBody2 = deltaLambdaMotor * mA1;
        // Apply the impulse to the body 2
        w2 += mI2 * angularImpulseBody2;

    }
    /**/

}




// Solve the position constraint (for position error correction)
void IHingeJoint::SolvePositionConstraint(const IConstraintSolverData &constraintSolverData)
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



    // Compute the current angle around the hinge axis
    scalar hingeAngle = ComputeCurrentHingeAngle(q1, q2);

    // Check if the limit constraints are violated or not
    scalar lowerLimitError = hingeAngle - mLowerLimit;
    scalar upperLimitError = mUpperLimit - hingeAngle;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute vectors needed in the Jacobian
            mA1 = q1.GetRotMatrix() * mHingeLocalAxisBody1;
    Vector3 a2  = q2.GetRotMatrix() * mHingeLocalAxisBody2;
    mA1.Normalize();
    a2.Normalize();

    const Vector3 b2 = a2.GetOneUnitOrthogonalVector();
    const Vector3 c2 = a2.Cross(b2);

    //    Vector3 b2 , c2;
    //    Vector3::BiUnitOrthogonalVector(a2 , c2 , b2);

    mB2CrossA1 = b2.Cross(mA1);
    mC2CrossA1 = c2.Cross(mA1);

    // Compute the corresponding skew-symmetric matrices
    Matrix3 skewSymmetricMatrixU1= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR1World);
    Matrix3 skewSymmetricMatrixU2= Matrix3::ComputeSkewSymmetricMatrixForCrossProduct(mR2World);



    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    Matrix3 massMatrix = Matrix3(inverseMassBodies, 0, 0,
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
    Vector3 errorTranslation = x2 + mR2World -
                               x1 - mR1World;


    // Compute the Lagrange multiplier lambda
    const Vector3 lambdaTranslation = mInverseMassMatrixTranslation * (-errorTranslation );




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
    Vector3 linearImpulseBody2  =  lambdaTranslation;
    Vector3 angularImpulseBody2 = -lambdaTranslation.Cross(mR2World);

    // Compute the pseudo velocity of body 2
    const Vector3 v2 = inverseMassBody2 * linearImpulseBody2;
    Vector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += Quaternion(0, w2) * q2 * scalar(0.5);
    q2.Normalize();




    // --------------- Rotation Constraints --------------- //

    // Compute the inverse mass matrix K=JM^-1J^t for the 2 rotation constraints (2x2 matrix)
    Vector3 I1B2CrossA1 = mI1 * mB2CrossA1;
    Vector3 I1C2CrossA1 = mI1 * mC2CrossA1;
    Vector3 I2B2CrossA1 = mI2 * mB2CrossA1;
    Vector3 I2C2CrossA1 = mI2 * mC2CrossA1;
    const scalar el11 = mB2CrossA1.Dot(I1B2CrossA1) + mB2CrossA1.Dot(I2B2CrossA1);
    const scalar el12 = mB2CrossA1.Dot(I1C2CrossA1) + mB2CrossA1.Dot(I2C2CrossA1);
    const scalar el21 = mC2CrossA1.Dot(I1B2CrossA1) + mC2CrossA1.Dot(I2B2CrossA1);
    const scalar el22 = mC2CrossA1.Dot(I1C2CrossA1) + mC2CrossA1.Dot(I2C2CrossA1);
    Matrix2 matrixKRotation(el11, el12, el21, el22);
    mInverseMassMatrixRotation.SetToZero();
    if (mBody1->GetType() == DYNAMIC || mBody2->GetType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = matrixKRotation.GetInverse();
    }

    // Compute the position error for the 3 rotation constraints
    Vector2 errorRotation = Vector2(mA1.Dot(b2), mA1.Dot(c2));



    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    Vector2 lambdaRotation = mInverseMassMatrixRotation * (-errorRotation);



    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -mB2CrossA1 * lambdaRotation.x - mC2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += Quaternion(0, w1) * q1 * scalar(0.5);
    q1.Normalize();




    // Compute the impulse of body 2
    angularImpulseBody2 = mB2CrossA1 * lambdaRotation.x + mC2CrossA1 * lambdaRotation.y;

    // Compute the pseudo velocity of body 2
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
            mInverseMassMatrixLimitMotor = mA1.Dot(mI1 * mA1) + mA1.Dot(mI2 * mA1);
            mInverseMassMatrixLimitMotor = (mInverseMassMatrixLimitMotor > 0.0) ?
                        scalar(1.0) / mInverseMassMatrixLimitMotor : scalar(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar lambdaLowerLimit = mInverseMassMatrixLimitMotor * (-lowerLimitError );

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = -lambdaLowerLimit * mA1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = mI1 * angularImpulseBody1 * scalar(0.5);

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.Normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = lambdaLowerLimit * mA1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = mI2 * angularImpulseBody2 * scalar(0.5);

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.Normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar lambdaUpperLimit = mInverseMassMatrixLimitMotor * (-upperLimitError);

            // Compute the impulse P=J^T * lambda of body 1
            const Vector3 angularImpulseBody1 = lambdaUpperLimit * mA1;

            // Compute the pseudo velocity of body 1
            const Vector3 w1 = mI1 * angularImpulseBody1 * scalar(0.5);

            // Update the body position/orientation of body 1
            q1 += Quaternion(0, w1) * q1 * scalar(0.5);
            q1.Normalize();

            // Compute the impulse P=J^T * lambda of body 2
            const Vector3 angularImpulseBody2 = -lambdaUpperLimit * mA1;

            // Compute the pseudo velocity of body 2
            const Vector3 w2 = mI2 * angularImpulseBody2 * scalar(0.5);

            // Update the body position/orientation of body 2
            q2 += Quaternion(0, w2) * q2 * scalar(0.5);
            q2.Normalize();
        }
    }
}




// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the limits of the joint and
 *                       false otherwise
 */
void IHingeJoint::EnableLimit(bool isLimitEnabled)
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
 * @param isMotorEnabled True if you want to enable the motor of the joint and
 *                       false otherwise
 */
void IHingeJoint::EnableMotor(bool isMotorEnabled)
{
    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->SetIsSleeping(false);
    mBody2->SetIsSleeping(false);
}

// Set the minimum angle limit
/**
 * @param lowerLimit The minimum limit angle of the joint (in radian)
 */
void IHingeJoint::SetMinAngleLimit(scalar lowerLimit)
{

    assert(mLowerLimit <= 0 && mLowerLimit >= -2.0 * PI);

    if (lowerLimit != mLowerLimit)
    {
        mLowerLimit = lowerLimit;

        // Reset the limits
        ResetLimits();
    }
}

// Set the maximum angle limit
/**
 * @param upperLimit The maximum limit angle of the joint (in radian)
 */
void IHingeJoint::SetMaxAngleLimit(scalar upperLimit)
{

    assert(upperLimit >= 0 && upperLimit <= 2.0 * PI);

    if (upperLimit != mUpperLimit)
    {

        mUpperLimit = upperLimit;

        // Reset the limits
        ResetLimits();
    }
}

// Reset the limits
void IHingeJoint::ResetLimits()
{
    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->SetIsSleeping(false);
    mBody2->SetIsSleeping(false);
}

// Set the motor speed
void IHingeJoint::SetMotorSpeed(scalar motorSpeed)
{
    if (motorSpeed != mMotorSpeed)
    {
        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->SetIsSleeping(false);
        mBody2->SetIsSleeping(false);
    }
}

// Set the maximum motor torque
/**
 * @param maxMotorTorque The maximum torque (in Newtons) of the joint motor
 */
void IHingeJoint::SetMaxMotorTorque(scalar maxMotorTorque)
{
    if (maxMotorTorque != mMaxMotorTorque)
    {
        assert(mMaxMotorTorque >= 0.0);
        mMaxMotorTorque = maxMotorTorque;

        // Wake up the two bodies of the joint
        mBody1->SetIsSleeping(false);
        mBody2->SetIsSleeping(false);
    }
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
scalar IHingeJoint::ComputeNormalizedAngle(scalar angle) const
{

    // Convert it into the range [-2*pi; 2*pi]
    angle = IModulo(angle, PI_TIMES_2);

    // Convert it into the range [-pi; pi]
    if (angle < -PI)
    {
        return angle + PI_TIMES_2;
    }
    else if (angle > PI)
    {
        return angle - PI_TIMES_2;
    }
    else
    {
        return angle;
    }
}

// Given an "inputAngle" in the range [-pi, pi], this method returns an
// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
// two angle limits in arguments.
scalar IHingeJoint::ComputeCorrespondingAngleNearLimits(scalar inputAngle, scalar lowerLimitAngle, scalar upperLimitAngle) const
{
    if (upperLimitAngle <= lowerLimitAngle)
    {
        return inputAngle;
    }
    else if (inputAngle > upperLimitAngle)
    {
        scalar diffToUpperLimit = IAbs(ComputeNormalizedAngle(inputAngle - upperLimitAngle));
        scalar diffToLowerLimit = IAbs(ComputeNormalizedAngle(inputAngle - lowerLimitAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
    }
    else if (inputAngle < lowerLimitAngle)
    {
        scalar diffToUpperLimit = IAbs(ComputeNormalizedAngle(upperLimitAngle - inputAngle));
        scalar diffToLowerLimit = IAbs(ComputeNormalizedAngle(lowerLimitAngle - inputAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
    }
    else
    {
        return inputAngle;
    }
}

// Compute the current angle around the hinge axis
scalar IHingeJoint::ComputeCurrentHingeAngle(const Quaternion &orientationBody1,
                                             const Quaternion &orientationBody2)
{

    scalar hingeAngle;

    // Compute the current orientation difference between the two bodies
    Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.GetInverse();
    currentOrientationDiff.Normalize();

    // Compute the relative rotation considering the initial orientation difference
    Quaternion relativeRotation = currentOrientationDiff * mInitOrientationDifferenceInv;
    relativeRotation.Normalize();



    // A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
    // length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
    // |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
    // rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
    // axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
    // has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
    // about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
    scalar cosHalfAngle = relativeRotation.GetW();
    scalar sinHalfAngleAbs = relativeRotation.GetV().Length();

    // Compute the dot product of the relative rotation axis and the hinge axis
    scalar dotProduct = relativeRotation.GetV().Dot(mA1);

    // If the relative rotation axis and the hinge axis are pointing the same direction
    if (dotProduct >= scalar(0.0))
    {
        hingeAngle = scalar(2.0) * IAtan2(sinHalfAngleAbs, cosHalfAngle);
    }
    else
    {
        hingeAngle = scalar(2.0) * IAtan2(sinHalfAngleAbs, -cosHalfAngle);
    }

    // Convert the angle from range [-2*pi; 2*pi] into the range [-pi; pi]
    hingeAngle = ComputeNormalizedAngle(hingeAngle);

    // Compute and return the corresponding angle near one the two limits
    return ComputeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
}

}
