#ifndef ICONTACTSOLVER_H
#define ICONTACTSOLVER_H

// Libraries
//#include "../../IGeometry/ICollision/ICollisionShapes/ICollisionShape.h"
#include "IRigidBody.h"
//#include "../../IGeometry/ICollision/ICollisionContact/IContactManager.h"
//#include "../../IGeometry/ICollision/ICollisionContact/IContactManifold.h"
//#include "../../IGeometry/ICollision/ICollisionContact/IContactManifoldSet.h"
//#include "../../IGeometry/ICollision/ICollisionContact/IContactPoint.h"
#include "IStep.h"

#include <map>

namespace IEngine
{

using namespace IMath;

class IIsland;
class IContactSolver
{
    // Structure ContactPointSolver
    /**
    * Contact solver internal data structure that to store all the
    * information relative to a contact point
    */
    struct IContactPointSolver
    {

        /// Accumulated normal impulse
        scalar AccumulatedPenetrationImpulse;

        /// Accumulated impulse in the 1st friction direction
        scalar AccumulatedFriction1Impulse;

        /// Accumulated impulse in the 2nd friction direction
        scalar AccumulatedFriction2Impulse;

        /// Accumulated split impulse for penetration correction
        scalar AccumulatedPenetrationSplitImpulse;

        /// Accumulated rolling resistance impulse
        Vector3 AccumulatedRollingResistanceImpulse;

        /// Accumulated rolling resistance impulse
        Vector3 AccumulatedRollingResistanceSplitImpulse;

        /// Normal vector of the contact
        Vector3 normal;

        /// First friction vector in the tangent plane
        Vector3 frictionVector1;

        /// Second friction vector in the tangent plane
        Vector3 frictionVector2;

        /// Old first friction vector in the tangent plane
        Vector3 oldFrictionVector1;

        /// Old second friction vector in the tangent plane
        Vector3 oldFrictionVector2;

        /// Vector from the body 1 center to the contact point
        Vector3 r1;

        /// Vector from the body 2 center to the contact point
        Vector3 r2;

        /// Cross product of r1 with 1st friction vector
        Vector3 r1CrossT1;

        /// Cross product of r1 with 2nd friction vector
        Vector3 r1CrossT2;

        /// Cross product of r2 with 1st friction vector
        Vector3 r2CrossT1;

        /// Cross product of r2 with 2nd friction vector
        Vector3 r2CrossT2;

        /// Cross product of r1 with the contact normal
        Vector3 r1CrossN;

        /// Cross product of r2 with the contact normal
        Vector3 r2CrossN;

        /// Penetration depth
        scalar penetrationDepth;

        /// Velocity restitution bias
        scalar restitutionBias;

        /// Inverse of the matrix K for the penenetration
        scalar inversePenetrationMass;

        /// Inverse of the matrix K for the 1st friction
        scalar inverseFriction1Mass;

        /// Inverse of the matrix K for the 2nd friction
        scalar inverseFriction2Mass;

        /// True if the contact was existing last time step
        bool isRestingContact;

        /// Pointer to the external contact
        IContactPoint* externalContact;

    };



    struct IContactManifoldSolver
    {

        /// Index of body 1 in the constraint solver
        u32 indexBody1;

        /// Index of body 2 in the constraint solver
        u32 indexBody2;

        /// Inverse of the mass of body 1
        scalar massInverseBody1;

        /// Inverse of the mass of body 2
        scalar massInverseBody2;

        /// Inverse inertia tensor of body 1
        Matrix3 inverseInertiaTensorBody1;

        /// Inverse inertia tensor of body 2
        Matrix3 inverseInertiaTensorBody2;

        /// Contact point constraints
        IContactPointSolver contacts[MAX_CONTACT_POINTS_IN_MANIFOLD];

        /************************************************/

        /// Number of contact points
        u32 nbContacts;

        /// True if the body 1 is of type dynamic
        bool isBody1DynamicType;

        /// True if the body 2 is of type dynamic
        bool isBody2DynamicType;

        /// Mix of the restitution factor for two bodies
        scalar restitutionFactor;

        /// Mix friction coefficient for the two bodies
        scalar frictionCoefficient;

        /// Rolling resistance factor between the two bodies
        scalar rollingResistanceFactor;

        /// Pointer to the external contact manifold
        IContactManifold* externalContactManifold;

        // - Variables used when friction constraints are apply at the center of the manifold-//

        /// Average normal vector of the contact manifold
        Vector3 normal;

        /// Point on body 1 where to apply the friction constraints
        Vector3 frictionPointBody1;

        /// Point on body 2 where to apply the friction constraints
        Vector3 frictionPointBody2;

        /// R1 vector for the friction constraints
        Vector3 r1Friction;

        /// R2 vector for the friction constraints
        Vector3 r2Friction;

        /// Cross product of r1 with 1st friction vector
        Vector3 r1CrossT1;

        /// Cross product of r1 with 2nd friction vector
        Vector3 r1CrossT2;

        /// Cross product of r2 with 1st friction vector
        Vector3 r2CrossT1;

        /// Cross product of r2 with 2nd friction vector
        Vector3 r2CrossT2;

        /// Matrix K for the first friction constraint
        scalar inverseFriction1Mass;

        /// Matrix K for the second friction constraint
        scalar inverseFriction2Mass;

        /// Matrix K for the twist friction constraint
        scalar inverseTwistFrictionMass;

        /// Matrix K for the rolling resistance constraint
        Matrix3 inverseRollingResistance;

        /// First friction direction at contact manifold center
        Vector3 frictionVector1;

        /// Second friction direction at contact manifold center
        Vector3 frictionVector2;

        /// Old 1st friction direction at contact manifold center
        Vector3 oldFrictionVector1;

        /// Old 2nd friction direction at contact manifold center
        Vector3 oldFrictionVector2;

        /// First friction direction impulse at manifold center
        scalar  AccumulatedFriction1Impulse;

        /// Second friction direction impulse at manifold center
        scalar  AccumulatedFriction2Impulse;

        /// Twist friction impulse at contact manifold center
        scalar  AccumulatedFrictionTwistImpulse;

        /// Rolling resistance impulse
        Vector3 AccumulatedRollingResistanceImpulse;

        /// Rolling resistance split impulse
        Vector3 AccumulatedRollingResistanceSplitImpulse;
    };



private:


    // -------------------- Constants --------------------- //
    /// Beta value for the penetration depth position correction without split impulses
    static const scalar BETA;

    /// Beta value for the penetration depth position correction with split impulses
    static const scalar BETA_SPLIT_IMPULSE;

    /// Slop distance (allowed penetration distance between bodies)
    static const scalar SLOP;


    //********************************************************************//


    bool mIsError = false;
    bool atLeastOneRestingContactPoint = false;

    bool mIsWarmStartingActive = true;
    bool mIsSplitImpulseActive = true;
    bool mIsStaticFriction = true;
    bool mIsSolveFrictionAtContactManifoldCenterActive = true;


    //********************************************************************//

    /// Number of contact constraints
    u32 mNbContactManifolds;

    /// Contact constraints
    IContactManifoldSolver* mContactConstraints;

    //********************************************************************//

    scalar mTimeStep;

    const std::map<IRigidBody*, u32>& mMapBodyToConstrainedVelocityIndex;

    IVelocity* mVelocities;
    IVelocity* mSplitVelocities;


    /*********************************************************/


    /// Compute the collision restitution factor from the restitution factor of each body
    scalar ComputeMixedRestitutionFactor(IRigidBody*  body1,
                                         IRigidBody*  body2) const;

    /// Compute the mixed friction coefficient from the friction coefficient of each body
    scalar ComputeMixedFrictionCoefficient(IRigidBody*  body1,
                                           IRigidBody*  body2) const;

    /// Compute th mixed rolling resistance factor between two bodies
    scalar ComputeMixedRollingResistance(IRigidBody*  body1,
                                         IRigidBody*  body2) const;


    /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
    /// plane for a contact manifold. The two vectors have to be
    /// such that : t1 x t2 = contactNormal.
    void ComputeFrictionVectors( const Vector3& deltaVelocity , IContactPointSolver& contactPoint) const;


    /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
    /// plane for a contact manifold. The two vectors have to be
    /// such that : t1 x t2 = contactNormal.
    void ComputeFrictionVectors( const Vector3& deltaVelocity , IContactManifoldSolver* contact ) const;


    /*********************************************************/

    void ApplyImpulseBody1( IContactManifoldSolver* contactManifold , const Vector3& _impulse , const Vector3& R_pivot )
    {
		mVelocities[contactManifold->indexBody1].v += (_impulse * contactManifold->massInverseBody1);
		mVelocities[contactManifold->indexBody1].w += ((R_pivot).Cross(_impulse) * contactManifold->inverseInertiaTensorBody1);
    }

    void ApplyImpulseBody2( IContactManifoldSolver* contactManifold , const Vector3& _impulse , const Vector3& R_pivot )
    {
		mVelocities[contactManifold->indexBody2].v += (_impulse * contactManifold->massInverseBody2);
		mVelocities[contactManifold->indexBody2].w += ((R_pivot).Cross(_impulse) * contactManifold->inverseInertiaTensorBody2);
    }


    void ApplyAngularImpulseBody1( IContactManifoldSolver* contactManifold , const Vector3& _AngularImpulse )
    {
		mVelocities[contactManifold->indexBody1].w += _AngularImpulse * contactManifold->inverseInertiaTensorBody1;
    }

    void ApplyAngularImpulseBody2( IContactManifoldSolver* contactManifold , const Vector3& _AngularImpulse )
    {
		mVelocities[contactManifold->indexBody2].w += _AngularImpulse * contactManifold->inverseInertiaTensorBody2;
    }

public:


    // Constructor
    IContactSolver(const std::map<IRigidBody*, u32>& mapBodyToVelocityIndex)
        :mIsWarmStartingActive(true),
         mIsSplitImpulseActive(true),
         mIsSolveFrictionAtContactManifoldCenterActive(true),
         mContactConstraints(nullptr),
         mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
         mVelocities(nullptr),
         mSplitVelocities(nullptr)
    {

    }

    /// Initilization iSland
    void  InitializeForIsland(scalar dt, IIsland* island);
    void  InitializeContactConstraints();


    /// Warm start the solver.
    void WarmStart();

    /// Solver velocity
    void SolveVelocityConstraint();

    /// Solver split velocity
    void SolveSplitVelocityConstraint();

    /// Store the computed impulses to use them to
    /// warm start the solver at the next iteration
    void StoreImpulsesWarmstart();

    ///Clenup
    void Cleanup();



    //-----------------------------------------//


    /// Set the split velocities arrays
    void SetSplitVelocitiesArrays(IVelocity* splitVelocities)
    {
        assert(splitVelocities != NULL);;
        mSplitVelocities = splitVelocities;

    }

    /// Set the constrained velocities arrays
    void SetConstrainedVelocitiesArrays(IVelocity* constrainedVelocities )
    {
        assert(constrainedVelocities != NULL);
        mVelocities = constrainedVelocities;

    }


    /******************************************************/

    /// Return true if the split impulses position correction technique is used for contacts
    bool IsSplitImpulseActive() const;

    /// Activate or Deactivate the split impulses for contacts
    void SetIsSplitImpulseActive(bool isActive);

    /// Activate or deactivate the solving of friction constraints at the center of
    /// the contact manifold instead of solving them at each contact point
    void SetIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

    /// Return true if the warmstart impulses
    bool IsIsWarmStartingActive() const;

    /// Activate or Deactivate the warmstart impulses
    void SetIsWarmStartingActive(bool isWarmStartingActive);

    //-------------------- Friendship --------------------//

    friend class IDynamicsWorld;

};


/********************************************************************************************************/


// Return true if the split impulses position correction technique is used for contacts
SIMD_INLINE  bool IContactSolver::IsSplitImpulseActive() const
{
    return mIsSplitImpulseActive;
}

// Activate or Deactivate the split impulses for contacts
SIMD_INLINE  void IContactSolver::SetIsSplitImpulseActive(bool isActive)
{
    mIsSplitImpulseActive = isActive;
}


// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
SIMD_INLINE  void IContactSolver::SetIsSolveFrictionAtContactManifoldCenterActive(bool isActive)
{
    mIsSolveFrictionAtContactManifoldCenterActive = isActive;
}

// /// Return true if the warmstart impulses
SIMD_INLINE  bool IContactSolver::IsIsWarmStartingActive() const
{
    return mIsWarmStartingActive;
}

// Activate or Deactivate the warmstart impulses
SIMD_INLINE  void IContactSolver::SetIsWarmStartingActive(bool isWarmStartingActive)
{
    mIsWarmStartingActive = isWarmStartingActive;
}



}


#endif // ICONTACTSOLVER_H
