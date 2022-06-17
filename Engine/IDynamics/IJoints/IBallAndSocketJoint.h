#ifndef IBALLANDSOCKETJOINT_H
#define IBALLANDSOCKETJOINT_H

#include "IJoint.h"

namespace IEngine
{

// Structure BallAndSocketJointInfo
/**
 * This structure is used to gather the information needed to create a ball-and-socket
 * joint. This structure will be used to create the actual ball-and-socket joint.
 */
struct IBallAndSocketJointInfo : public IJointInfo
{

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Constructor
        /**
         * @param rigidBody1 Pointer to the first body of the joint
         * @param rigidBody2 Pointer to the second body of the joint
         * @param initAnchorPointWorldSpace The anchor point in world-space
         *                                  coordinates
         */
        IBallAndSocketJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                                 const Vector3& initAnchorPointWorldSpace)
                               :IJointInfo(rigidBody1, rigidBody2, BALLSOCKETJOINT),
                               anchorPointWorldSpace(initAnchorPointWorldSpace)
        {

        }
};




// Class BallAndSocketJoint
/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation
 * between two bodies. This joint has three degrees of freedom. It can be used to
 * create a chain of bodies for instance.
 */
class IBallAndSocketJoint : public IJoint
{

    private :


        bool isWarmStartingActive = true;

        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const scalar BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3 mI2;

        /// Bias vector for the constraint
        Vector3 mBiasVector;



        /// Inverse mass matrix K=JM^-1J^-t of the constraint
        Matrix3 mInverseMassMatrix;

        /// Accumulated impulse
        Vector3 mImpulse;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IBallAndSocketJoint(const IBallAndSocketJoint& constraint);

        /// Private assignment operator
        IBallAndSocketJoint& operator=(const IBallAndSocketJoint& constraint);

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
        IBallAndSocketJoint(const IBallAndSocketJointInfo& jointInfo);

        /// Destructor
        virtual ~IBallAndSocketJoint();
};


// Return the number of bytes used by the joint
SIMD_INLINE size_t IBallAndSocketJoint::GetSizeInBytes() const
{
    return sizeof(IBallAndSocketJoint);
}


}

#endif // IBALLANDSOCKETJOINT_H
