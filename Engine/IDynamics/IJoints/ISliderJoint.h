#ifndef ISLIDERJOINT_H
#define ISLIDERJOINT_H

#include "IJoint.h"

namespace IEngine
{


// Structure SliderJointInfo
/**
 * This structure is used to gather the information needed to create a slider
 * joint. This structure will be used to create the actual slider joint.
 */
struct ISliderJointInfo : public IJointInfo
{

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Slider axis (in world-space coordinates)
        Vector3 sliderAxisWorldSpace;

        /// True if the slider limits are enabled
        bool isLimitEnabled;

        /// True if the slider motor is enabled
        bool isMotorEnabled;

        /// Mininum allowed translation if limits are enabled
        scalar minTranslationLimit;

        /// Maximum allowed translation if limits are enabled
        scalar maxTranslationLimit;

        /// Motor speed
        scalar motorSpeed;

        /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        scalar maxMotorForce;

        /// Constructor without limits and without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         */
        ISliderJointInfo(IRigidBody* rigidBody1,
                         IRigidBody* rigidBody2,
                         const Vector3& initAnchorPointWorldSpace,
                         const Vector3& initSliderAxisWorldSpace)
                       :IJointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(false),
                         isMotorEnabled(false),
                         minTranslationLimit(-1.0),
                         maxTranslationLimit(1.0),
                         motorSpeed(0),
                         maxMotorForce(0)
        {

        }

        /// Constructor with limits and no motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         */
        ISliderJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                          const Vector3& initAnchorPointWorldSpace,
                          const Vector3& initSliderAxisWorldSpace,
                          scalar initMinTranslationLimit,
                          scalar initMaxTranslationLimit)
                          :IJointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true),
                         isMotorEnabled(false),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit),
                         motorSpeed(0),
                         maxMotorForce(0)
        {

        }

        /// Constructor with limits and motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         * @param initMotorSpeed The initial speed of the joint motor (in meters per second)
         * @param initMaxMotorForce The initial maximum motor force of the joint (in Newtons x meters)
         */
        ISliderJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                          const Vector3& initAnchorPointWorldSpace,
                          const Vector3& initSliderAxisWorldSpace,
                          scalar initMinTranslationLimit,
                          scalar initMaxTranslationLimit,
                          scalar initMotorSpeed,
                          scalar initMaxMotorForce)
                         :IJointInfo(rigidBody1, rigidBody2, SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true),
                         isMotorEnabled(true),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit),
                         motorSpeed(initMotorSpeed),
                         maxMotorForce(initMaxMotorForce)
        {

        }
};



// Class SliderJoint
/**
 * This class represents a slider joint. This joint has a one degree of freedom.
 * It only allows relative translation of the bodies along a single direction and no
 * rotation.
 */
class ISliderJoint : public IJoint
{

    private :


        bool isWarmStartingActive = true;


        // -------------------- Constants -------------------- //

        // Beta value for the position correction bias factor
        static const scalar BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 mLocalAnchorPointBody2;

        /// Slider axis (in local-space coordinates of body 1)
        Vector3 mSliderAxisBody1;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3 mI2;

        /// Inverse of the initial orientation difference between the two bodies
        Quaternion mInitOrientationDifferenceInv;

        /// First vector orthogonal to the slider axis local-space of body 1
        Vector3 mN1;

        /// Second vector orthogonal to the slider axis and mN1 in local-space of body 1
        Vector3 mN2;

        /// Vector r1 in world-space coordinates
        Vector3 mR1;

        /// Vector r2 in world-space coordinates
        Vector3 mR2;

        /// Cross product of r2 and n1
        Vector3 mR2CrossN1;

        /// Cross product of r2 and n2
        Vector3 mR2CrossN2;

        /// Cross product of r2 and the slider axis
        Vector3 mR2CrossSliderAxis;

        /// Cross product of vector (r1 + u) and n1
        Vector3 mR1PlusUCrossN1;

        /// Cross product of vector (r1 + u) and n2
        Vector3 mR1PlusUCrossN2;

        /// Cross product of vector (r1 + u) and the slider axis
        Vector3 mR1PlusUCrossSliderAxis;

        /// Bias of the 2 translation constraints
        Vector2 mBTranslation;

        /// Bias of the 3 rotation constraints
        Vector3 mBRotation;

        /// Bias of the lower limit constraint
        scalar mBLowerLimit;

        /// Bias of the upper limit constraint
        scalar mBUpperLimit;

        /// Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
        Matrix2 mInverseMassMatrixTranslationConstraint;

        /// Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
        Matrix3 mInverseMassMatrixRotationConstraint;

        /// Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
        scalar mInverseMassMatrixLimit;

        /// Inverse of mass matrix K=JM^-1J^t for the motor
        scalar mInverseMassMatrixMotor;

        /// Accumulated impulse for the 2 translation constraints
        Vector2 mImpulseTranslation;

        /// Accumulated impulse for the 3 rotation constraints
        Vector3 mImpulseRotation;

        /// Accumulated impulse for the lower limit constraint
        scalar mImpulseLowerLimit;

        /// Accumulated impulse for the upper limit constraint
        scalar mImpulseUpperLimit;

        /// Accumulated impulse for the motor
        scalar mImpulseMotor;

        /// True if the slider limits are enabled
        bool mIsLimitEnabled;

        /// True if the motor of the joint in enabled
        bool mIsMotorEnabled;

        /// Slider axis in world-space coordinates
        Vector3 mSliderAxisWorld;

        /// Lower limit (minimum translation distance)
        scalar mLowerLimit;

        /// Upper limit (maximum translation distance)
        scalar mUpperLimit;

        /// True if the lower limit is violated
        bool mIsLowerLimitViolated;

        /// True if the upper limit is violated
        bool mIsUpperLimitViolated;

        /// Motor speed (in m/s)
        scalar mMotorSpeed;

        /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        scalar mMaxMotorForce;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ISliderJoint(const ISliderJoint& constraint);

        /// Private assignment operator
        ISliderJoint& operator=(const ISliderJoint& constraint);

        /// Reset the limits
        void ResetLimits();

        /// Return the number of bytes used by the joint
        virtual size_t GetSizeInBytes() const;

        /// Initialize before solving the constraint
        virtual void InitBeforeSolve( const IConstraintSolverData& constraintSolverData );

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void Warmstart( const IConstraintSolverData& constraintSolverData );

        /// Solve the velocity constraint
        virtual void SolveVelocityConstraint( const IConstraintSolverData& constraintSolverData );

        /// Solve the position constraint (for position error correction)
        virtual void SolvePositionConstraint( const IConstraintSolverData& constraintSolverData );

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ISliderJoint(const ISliderJointInfo& jointInfo);

        /// Destructor
        virtual ~ISliderJoint();

        /// Return true if the limits or the joint are enabled
        bool IsLimitEnabled() const;

        /// Return true if the motor of the joint is enabled
        bool IsMotorEnabled() const;

        /// Enable/Disable the limits of the joint
        void EnableLimit(bool isLimitEnabled);

        /// Enable/Disable the motor of the joint
        void EnableMotor(bool isMotorEnabled);

        /// Return the current translation value of the joint
        scalar GetTranslation() const;

        /// Return the minimum translation limit
        scalar GetMinTranslationLimit() const;

        /// Set the minimum translation limit
        void SetMinTranslationLimit(scalar lowerLimit);

        /// Return the maximum translation limit
        scalar GetMaxTranslationLimit() const;

        /// Set the maximum translation limit
        void SetMaxTranslationLimit(scalar upperLimit);

        /// Return the motor speed
        scalar GetMotorSpeed() const;

        /// Set the motor speed
        void SetMotorSpeed(scalar motorSpeed);

        /// Return the maximum motor force
        scalar GetMaxMotorForce() const;

        /// Set the maximum motor force
        void SetMaxMotorForce(scalar maxMotorForce);

        /// Return the intensity of the current force applied for the joint motor
        scalar GetMotorForce(scalar timeStep) const;
};

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
SIMD_INLINE bool ISliderJoint::IsLimitEnabled() const
{
    return mIsLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
SIMD_INLINE bool ISliderJoint::IsMotorEnabled() const
{
    return mIsMotorEnabled;
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
SIMD_INLINE scalar ISliderJoint::GetMinTranslationLimit() const
{
    return mLowerLimit;
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
SIMD_INLINE scalar ISliderJoint::GetMaxTranslationLimit() const
{
    return mUpperLimit;
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
SIMD_INLINE scalar ISliderJoint::GetMotorSpeed() const
{
    return mMotorSpeed;
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
SIMD_INLINE scalar ISliderJoint::GetMaxMotorForce() const
{
    return mMaxMotorForce;
}

// Return the intensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
SIMD_INLINE scalar ISliderJoint::GetMotorForce(scalar timeStep) const
{
    return mImpulseMotor / timeStep;
}

// Return the number of bytes used by the joint
SIMD_INLINE size_t ISliderJoint::GetSizeInBytes() const
{
    return sizeof(ISliderJoint);
}



}

#endif // ISLIDERJOINT_H
