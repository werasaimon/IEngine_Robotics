#include "IContactSolver.h"
#include "IIsland.h"
#include "IRigidBody.h"

namespace IEngine
{

// Constants initialization
const scalar IContactSolver::BETA = scalar(0.2);
const scalar IContactSolver::BETA_SPLIT_IMPULSE = scalar(0.2);
const scalar IContactSolver::SLOP= scalar(0.01);


void IContactSolver::InitializeForIsland(scalar dt, IIsland *island)
{
    // Set the current time step
     mTimeStep = dt;

     mNbContactManifolds = island->GetNbContactManifolds();
     mContactConstraints = new IContactManifoldSolver[mNbContactManifolds];


     // For each contact manifold of the island
     IContactManifold** contactManifolds = island->GetContactManifold();
     for (u32 i=0; i<mNbContactManifolds; i++)
     {

         IContactManifold* externalManifold = contactManifolds[i];
         IContactManifoldSolver* internalManifold = &mContactConstraints[i];

         assert(externalManifold->GetNbContactPoints() > 0);


         IRigidBody *body1 = static_cast<IRigidBody*>(externalManifold->GetShape1()->GetBody());
         IRigidBody *body2 = static_cast<IRigidBody*>(externalManifold->GetShape2()->GetBody());

         /**
         if( externalManifold->getIsNewContactInfoManiflod() )
         {
             for (u32 i = 0; i < MAX_CONTACT_POINTS_IN_MANIFOLD; ++i)
             {
                 // Initialize the accumulated impulses to zero
                 internalManifold->contacts[i].AccumulatedPenetrationImpulse = scalar(0);
                 internalManifold->contacts[i].AccumulatedFriction1Impulse   = scalar(0);
                 internalManifold->contacts[i].AccumulatedFriction2Impulse   = scalar(0);
                 internalManifold->contacts[i].AccumulatedRollingResistanceImpulse = Vector3(0,0,0);
                 internalManifold->contacts[i].AccumulatedRollingResistanceSplitImpulse = Vector3(0,0,0);
                 internalManifold->contacts[i].AccumulatedPenetrationSplitImpulse = scalar(0);
                 internalManifold->contacts[i].isRestingContact = false;
             }

             // Initialize the accumulated impulses to zero
             internalManifold->AccumulatedFriction1Impulse = 0.0;
             internalManifold->AccumulatedFriction2Impulse = 0.0;
             internalManifold->AccumulatedFrictionTwistImpulse = 0.0;
             internalManifold->AccumulatedRollingResistanceImpulse = Vector3(0,0,0);
             internalManifold->AccumulatedRollingResistanceSplitImpulse = Vector3(0,0,0);
             atLeastOneRestingContactPoint = false;
         }
         /**/


         // Get the position of the two bodies
         const Vector3& x1 = body1->mCenterOfMassWorld;
         const Vector3& x2 = body2->mCenterOfMassWorld;


         // Initialize the internal contact manifold structure using the external
         // contact manifold
         internalManifold->indexBody1 = mMapBodyToConstrainedVelocityIndex.find(body1)->second;
         internalManifold->indexBody2 = mMapBodyToConstrainedVelocityIndex.find(body2)->second;

         internalManifold->inverseInertiaTensorBody1 = (body1->mTransform.GetBasis() * body1->mInertiaTensorLocalInverse * body1->mTransform.GetBasis().GetTranspose());
         internalManifold->inverseInertiaTensorBody2 = (body2->mTransform.GetBasis() * body2->mInertiaTensorLocalInverse * body2->mTransform.GetBasis().GetTranspose());
         internalManifold->massInverseBody1 = body1->mMassInverse;
         internalManifold->massInverseBody2 = body2->mMassInverse;
         internalManifold->nbContacts = externalManifold->GetNbContactPoints();
         internalManifold->restitutionFactor = ComputeMixedRestitutionFactor(body1, body2);
         internalManifold->frictionCoefficient = ComputeMixedFrictionCoefficient(body1, body2);
         internalManifold->rollingResistanceFactor = ComputeMixedRollingResistance(body1, body2);
         internalManifold->externalContactManifold = externalManifold;
         internalManifold->isBody1DynamicType = (body1->mType == DYNAMIC);
         internalManifold->isBody2DynamicType = (body2->mType == DYNAMIC);



         // If we solve the friction constraints at the center of the contact manifold
         if (mIsSolveFrictionAtContactManifoldCenterActive)
         {
             internalManifold->frictionPointBody1 = Vector3(0,0,0);
             internalManifold->frictionPointBody2 = Vector3(0,0,0);
         }



         // For each  contact point of the contact manifold
         for (u32 c=0; c<externalManifold->GetNbContactPoints(); c++)
         {

             // Get a contact point
             IContactPoint* externalContact = externalManifold->GetContactPoint(c);

              // Get a solver contact point
             IContactPointSolver& contactPoint = internalManifold->contacts[c];

             // Get the contact point on the two bodies
             Vector3 p1 = externalContact->GetWorldPointOnBody1();
             Vector3 p2 = externalContact->GetWorldPointOnBody2();


             contactPoint.externalContact = externalContact;
             contactPoint.normal = externalContact->GetNormal();
             contactPoint.r1 = p1 - x1;
             contactPoint.r2 = p2 - x2;
             contactPoint.penetrationDepth = externalContact->GetPenetration();
             contactPoint.isRestingContact = externalContact->GetIsRestingContact();// || !externalManifold->getIsNewContactInfoManiflod();
             //externalContact->setIsRestingContact(true);
             contactPoint.oldFrictionVector1 = externalContact->GetFrictionVector1();
             contactPoint.oldFrictionVector2 = externalContact->GetFrictionVector2();


             // Get the cached accumulated impulses from the previous step
             contactPoint.AccumulatedPenetrationImpulse = 0.0;
             contactPoint.AccumulatedFriction1Impulse = 0.0;
             contactPoint.AccumulatedFriction2Impulse = 0.0;
             contactPoint.AccumulatedRollingResistanceImpulse = Vector3(0,0,0);
             contactPoint.AccumulatedRollingResistanceSplitImpulse  = Vector3(0,0,0);


             // If we solve the friction constraints at the center of the contact manifold
             if (mIsSolveFrictionAtContactManifoldCenterActive)
             {
                 internalManifold->frictionPointBody1 += p1;
                 internalManifold->frictionPointBody2 += p2;
             }
         }


         // If we solve the friction constraints at the center of the contact manifold
         if (mIsSolveFrictionAtContactManifoldCenterActive && internalManifold->nbContacts > 0)
         {
             internalManifold->frictionPointBody1 /=static_cast<scalar>(float(internalManifold->nbContacts));
             internalManifold->frictionPointBody2 /=static_cast<scalar>(float(internalManifold->nbContacts));
             internalManifold->r1Friction = internalManifold->frictionPointBody1 - x1;
             internalManifold->r2Friction = internalManifold->frictionPointBody2 - x2;
             internalManifold->oldFrictionVector1 = externalManifold->GetFrictionVector1();
             internalManifold->oldFrictionVector2 = externalManifold->GetFrictionVector2();

             // If warm starting is active
             if (mIsWarmStartingActive)
             {

                 internalManifold->AccumulatedFriction1Impulse = externalManifold->GetAccumulatedFrictionImpulse1();
                 internalManifold->AccumulatedFriction2Impulse = externalManifold->GetAccumulatedFrictionImpulse2();
                 internalManifold->AccumulatedFrictionTwistImpulse = externalManifold->GetAccumulatedFrictionTwistImpulse();
                 internalManifold->AccumulatedRollingResistanceImpulse = externalManifold->GetAccumulatedRollingResistanceImpulse();

             }
             else
             {

                 // Initialize the accumulated impulses to zero
                 internalManifold->AccumulatedFriction1Impulse = 0.0;
                 internalManifold->AccumulatedFriction2Impulse = 0.0;
                 internalManifold->AccumulatedFrictionTwistImpulse = 0.0;
                 internalManifold->AccumulatedRollingResistanceImpulse = Vector3(0, 0, 0);
                 internalManifold->AccumulatedRollingResistanceSplitImpulse = Vector3(0, 0, 0);
             }

         }
     }

     /// Initilization
     InitializeContactConstraints();
}

void IContactSolver::InitializeContactConstraints()
{
    // For each contact constraint
    for (u32 c=0; c<mNbContactManifolds; c++)
    {
        IContactManifoldSolver* manifold = &mContactConstraints[c];


        // Get the inertia tensors of both bodies
        Matrix3& I1 = manifold->inverseInertiaTensorBody1;
        Matrix3& I2 = manifold->inverseInertiaTensorBody2;


        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive)
        {
            manifold->normal = Vector3(0.0, 0.0, 0.0);
        }


        // Get the velocities of the bodies
        const Vector3& v1 = mVelocities[manifold->indexBody1].v;
        const Vector3& w1 = mVelocities[manifold->indexBody1].w;
        const Vector3& v2 = mVelocities[manifold->indexBody2].v;
        const Vector3& w2 = mVelocities[manifold->indexBody2].w;

        /**
        scalar gamma1 = ISqrt( 1.0 - Dot(mVelocities[manifold->indexBody1].v/60.f,mVelocities[manifold->indexBody1].v/60.f)/ LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C );
        scalar gamma2 = ISqrt( 1.0 - Dot(mVelocities[manifold->indexBody2].v/60.f,mVelocities[manifold->indexBody2].v/60.f)/ LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C );
        /**

        scalar gamma1 = mVelocities[manifold->indexBody1].gamma;
        scalar gamma2 = mVelocities[manifold->indexBody2].gamma;

        // Get the velocities of the bodies
        const Vector3& v1 = mVelocities[manifold->indexBody1].v / gamma1;
        const Vector3& w1 = mVelocities[manifold->indexBody1].w / gamma1;
        const Vector3& v2 = mVelocities[manifold->indexBody2].v / gamma2;
        const Vector3& w2 = mVelocities[manifold->indexBody2].w / gamma2;

        /**/

        // For each contact point constraint
        for (u32 i=0; i<manifold->nbContacts; i++)
        {

            IContactPointSolver& contactPoint = manifold->contacts[i];
            IContactPoint* externalContact = contactPoint.externalContact;

            // Compute the velocity difference
            Vector3 deltaV = v2 + w2.Cross(contactPoint.r2) -
                             v1 - w1.Cross(contactPoint.r1);

            contactPoint.r1CrossN = contactPoint.r1.Cross(contactPoint.normal);
            contactPoint.r2CrossN = contactPoint.r2.Cross(contactPoint.normal);

            // Compute the inverse mass matrix K for the penetration constraint
            scalar massPenetration = manifold->massInverseBody1 +
                    manifold->massInverseBody2 +
                    ((I1 * contactPoint.r1CrossN).Cross(contactPoint.r1)).Dot(contactPoint.normal) +
                    ((I2 * contactPoint.r2CrossN).Cross(contactPoint.r2)).Dot(contactPoint.normal);
            massPenetration > 0.0 ? contactPoint.inversePenetrationMass = scalar(1.0) / massPenetration : scalar(0.0);

            // If we do not solve the friction constraints at the center of the contact manifold
            if (!mIsSolveFrictionAtContactManifoldCenterActive)
            {

                // Compute the friction vectors
                ComputeFrictionVectors(deltaV, contactPoint);

                contactPoint.r1CrossT1 = contactPoint.r1.Cross(contactPoint.frictionVector1);
                contactPoint.r1CrossT2 = contactPoint.r1.Cross(contactPoint.frictionVector2);
                contactPoint.r2CrossT1 = contactPoint.r2.Cross(contactPoint.frictionVector1);
                contactPoint.r2CrossT2 = contactPoint.r2.Cross(contactPoint.frictionVector2);

                // Compute the inverse mass matrix K for the friction
                // constraints at each contact point
                scalar friction1Mass = manifold->massInverseBody1 +
                        manifold->massInverseBody2 +
                        ((I1 * contactPoint.r1CrossT1).Cross(contactPoint.r1)).Dot(contactPoint.frictionVector1) +
                        ((I2 * contactPoint.r2CrossT1).Cross(contactPoint.r2)).Dot(contactPoint.frictionVector1);

                scalar friction2Mass = manifold->massInverseBody1 +
                        manifold->massInverseBody2 +
                        ((I1 * contactPoint.r1CrossT2).Cross(contactPoint.r1)).Dot(contactPoint.frictionVector2) +
                        ((I2 * contactPoint.r2CrossT2).Cross(contactPoint.r2)).Dot(contactPoint.frictionVector2);


                friction1Mass > 0.0 ? contactPoint.inverseFriction1Mass = scalar(1.0) / friction1Mass : scalar(0.0);
                friction2Mass > 0.0 ? contactPoint.inverseFriction2Mass = scalar(1.0) / friction2Mass : scalar(0.0);

                scalar frictionTwistMass = contactPoint.normal.Dot(manifold->inverseInertiaTensorBody1 * contactPoint.normal) +
                                           contactPoint.normal.Dot(manifold->inverseInertiaTensorBody2 * contactPoint.normal);

                manifold->inverseTwistFrictionMass = scalar(1.0) / frictionTwistMass;
            }

            // Compute the restitution velocity bias "b". We compute this here instead
            // of inside the solve() method because we need to use the velocity difference
            // at the beginning of the contact. Note that if it is a resting contact (normal
            // velocity bellow a given threshold), we do not add a restitution velocity bias
            contactPoint.restitutionBias = 0.0;
            scalar deltaVDotN = deltaV.Dot(contactPoint.normal);


            scalar damping =  RESTITUTION_VELOCITY_THRESHOLD * 2.0;
            scalar bounce  =  manifold->restitutionFactor;

            if ( deltaVDotN < -damping )
            {
                contactPoint.restitutionBias =  bounce * deltaVDotN;
            }

            // If the warm starting of the contact solver is active
            if (mIsWarmStartingActive)
            {
                // Get the cached accumulated impulses from the previous step
                contactPoint.AccumulatedPenetrationImpulse            =  externalContact->GetAccumulatedPenetrationImpulse();
                contactPoint.AccumulatedFriction1Impulse              =  externalContact->GetAccumulatedFrictionImpulse1();
                contactPoint.AccumulatedFriction2Impulse              =  externalContact->GetAccumulatedFrictionImpulse2();
                contactPoint.AccumulatedRollingResistanceImpulse      =  externalContact->GetAccumulatedRollingResistanceImpulse();
            }
            else
            {
                // Get the cached accumulated impulses from the previous step
                contactPoint.AccumulatedPenetrationImpulse = 0.0;
                contactPoint.AccumulatedFriction1Impulse = 0.0;
                contactPoint.AccumulatedFriction2Impulse = 0.0;
                contactPoint.AccumulatedRollingResistanceImpulse = Vector3(0,0,0);
            }

            // Initialize the split impulses to zero
            contactPoint.AccumulatedPenetrationSplitImpulse = 0.0;
            contactPoint.AccumulatedRollingResistanceSplitImpulse = Vector3(0,0,0);

            // If we solve the friction constraints at the center of the contact manifold
            if (mIsSolveFrictionAtContactManifoldCenterActive)
            {
                manifold->normal += contactPoint.normal;
            }

        }

        // Compute the inverse K matrix for the rolling resistance constraint
        manifold->inverseRollingResistance.SetToZero();
        if (manifold->rollingResistanceFactor > 0 && (manifold->isBody1DynamicType || manifold->isBody2DynamicType))
        {
            manifold->inverseRollingResistance = manifold->inverseInertiaTensorBody1 + manifold->inverseInertiaTensorBody2;
            manifold->inverseRollingResistance = manifold->inverseRollingResistance.GetInverse();
        }


        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive &&  manifold->nbContacts > 0)
        {

            manifold->normal.Normalize();

            Vector3 deltaVFrictionPoint = v2 + w2.Cross(manifold->r2Friction) -
                                          v1 - w1.Cross(manifold->r1Friction);

            // Compute the friction vectors
            ComputeFrictionVectors(deltaVFrictionPoint, manifold);

            // Compute the inverse mass matrix K for the friction constraints at the center of
            // the contact manifold
            manifold->r1CrossT1 = manifold->r1Friction.Cross(manifold->frictionVector1);
            manifold->r1CrossT2 = manifold->r1Friction.Cross(manifold->frictionVector2);
            manifold->r2CrossT1 = manifold->r2Friction.Cross(manifold->frictionVector1);
            manifold->r2CrossT2 = manifold->r2Friction.Cross(manifold->frictionVector2);

            scalar friction1Mass = manifold->massInverseBody1 + manifold->massInverseBody2 +
                    ((I1 * manifold->r1CrossT1).Cross(manifold->r1Friction)).Dot(manifold->frictionVector1) +
                    ((I2 * manifold->r2CrossT1).Cross(manifold->r2Friction)).Dot(manifold->frictionVector1);

            scalar friction2Mass = manifold->massInverseBody1 + manifold->massInverseBody2 +
                    ((I1 * manifold->r1CrossT2).Cross(manifold->r1Friction)).Dot(manifold->frictionVector2) +
                    ((I2 * manifold->r2CrossT2).Cross(manifold->r2Friction)).Dot(manifold->frictionVector2);

            scalar frictionTwistMass = manifold->normal.Dot(manifold->inverseInertiaTensorBody1 * manifold->normal) +
                                       manifold->normal.Dot(manifold->inverseInertiaTensorBody2 * manifold->normal);

            friction1Mass > 0.0 ? manifold->inverseFriction1Mass = scalar(1.0)/friction1Mass : scalar(0.0);
            friction2Mass > 0.0 ? manifold->inverseFriction2Mass = scalar(1.0) / friction2Mass : scalar(0.0);
            frictionTwistMass > 0.0 ? manifold->inverseTwistFrictionMass = scalar(1.0) / frictionTwistMass : scalar(0.0);
        }
    }
}

void IContactSolver::WarmStart()
{
    // Check that warm starting is active
    if (!mIsWarmStartingActive) return;

    // For each constraint
    for (u32 c=0; c<mNbContactManifolds; c++)
    {

        IContactManifoldSolver* contactManifold = &mContactConstraints[c];
        atLeastOneRestingContactPoint = false;

        const Vector3& v1 = mVelocities[contactManifold ->indexBody1].v;
        const Vector3& w1 = mVelocities[contactManifold ->indexBody1].w;
        const Vector3& v2 = mVelocities[contactManifold ->indexBody2].v;
        const Vector3& w2 = mVelocities[contactManifold ->indexBody2].w;

        for (u32 i = 0; i < contactManifold->nbContacts; ++i)
        {
            IContactPointSolver &contactPoint = contactManifold->contacts[i];
            IContactPoint *cp = contactPoint.externalContact;


            scalar  &_accumulaterImpuls                        =  contactPoint.AccumulatedPenetrationImpulse;
            scalar  &_accumulaterImpulsFriction1               =  contactPoint.AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2               =  contactPoint.AccumulatedFriction2Impulse;
            Vector3 &_accumulaterRollingResistanceImpulse      =  contactPoint.AccumulatedRollingResistanceImpulse;
            Vector3 &_accumulaterRollingResistanceSplitImpulse =  contactPoint.AccumulatedRollingResistanceSplitImpulse;


            /****************************************************************************/
            /// Penetration Distance
            scalar beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
            scalar sepp = -IAbs( cp->GetPenetration() );
            cp->SetPenetration( -(beta/mTimeStep) * IMin( scalar(0), sepp + SLOP ) * 1.f);
            /****************************************************************************/



            if( contactPoint.isRestingContact )
            {
                atLeastOneRestingContactPoint = true;

                //         // Project the old friction impulses (with old friction vectors) into
                //         // the new friction vectors to get the new friction impulses
                //contactManifold->oldFrictionVector1 = contactManifold->frictionVector1;
                //contactManifold->oldFrictionVector2 = contactManifold->frictionVector2;
                Vector3 oldFrictionImpulse = _accumulaterImpulsFriction1 * contactPoint.oldFrictionVector1 +
                                              _accumulaterImpulsFriction2 * contactPoint.oldFrictionVector2;

                _accumulaterImpulsFriction1 = oldFrictionImpulse.Dot(contactPoint.frictionVector1);
                _accumulaterImpulsFriction2 = oldFrictionImpulse.Dot(contactPoint.frictionVector2);


				ApplyImpulseBody1( contactManifold , -contactPoint.normal * _accumulaterImpuls , contactPoint.r1 );
				ApplyImpulseBody2( contactManifold ,  contactPoint.normal * _accumulaterImpuls , contactPoint.r2 );


                //                     //------------------------  accumulation apply impulse linear -----------------------------//
                //                    mVelocities[contactManifold->indexBody1].v += -contactPoint.normal * _accumulaterImpuls * contactManifold->massInverseBody1;
                //                    mVelocities[contactManifold->indexBody2].v +=  contactPoint.normal * _accumulaterImpuls * contactManifold->massInverseBody2;

                //                     //------------------------  accumulation apply impulse angular -----------------------------//
                //                    mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.normal * _accumulaterImpuls) * contactManifold->inverseInertiaTensorBody1;
                //                    mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.normal * _accumulaterImpuls) * contactManifold->inverseInertiaTensorBody2;



                if (!mIsSolveFrictionAtContactManifoldCenterActive)
                {



					ApplyImpulseBody1( contactManifold , -contactPoint.frictionVector1 * _accumulaterImpulsFriction1 , contactPoint.r1 );
					ApplyImpulseBody2( contactManifold ,  contactPoint.frictionVector1 * _accumulaterImpulsFriction1 , contactPoint.r2 );

                    //                        //------------------------ friction1 accumulation apply impulse linear -----------------------------//
                    //                        mVelocities[contactManifold->indexBody1].v += -contactPoint.frictionVector1 * _accumulaterImpulsFriction1 * contactManifold->massInverseBody1;
                    //                        mVelocities[contactManifold->indexBody2].v +=  contactPoint.frictionVector1 * _accumulaterImpulsFriction1 * contactManifold->massInverseBody2;

                    //                        //------------------------ friction1 accumulation apply impulse angular ----------------------------//
                    //                        mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.frictionVector1 * _accumulaterImpulsFriction1) * contactManifold->inverseInertiaTensorBody1;
                    //                        mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.frictionVector1 * _accumulaterImpulsFriction1) * contactManifold->inverseInertiaTensorBody2;



					ApplyImpulseBody1( contactManifold ,-contactPoint.frictionVector2 * _accumulaterImpulsFriction2 , contactPoint.r1 );
					ApplyImpulseBody2( contactManifold , contactPoint.frictionVector2 * _accumulaterImpulsFriction2 , contactPoint.r2 );


                    //                        //------------------------ friction2 accumulation apply impulse linear -----------------------------//
                    //                        mVelocities[contactManifold->indexBody1].v += -contactPoint.frictionVector2 * _accumulaterImpulsFriction2 * contactManifold->massInverseBody1;
                    //                        mVelocities[contactManifold->indexBody2].v +=  contactPoint.frictionVector2 * _accumulaterImpulsFriction2 * contactManifold->massInverseBody2;

                    //                        //------------------------ friction2 accumulation apply impulse angular ----------------------------//
                    //                        mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.frictionVector2 * _accumulaterImpulsFriction2) * contactManifold->inverseInertiaTensorBody1;
                    //                        mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.frictionVector2 * _accumulaterImpulsFriction2) * contactManifold->inverseInertiaTensorBody2;




                    if (contactManifold->rollingResistanceFactor > 0)
                    {


                        ApplyAngularImpulseBody1( contactManifold , -_accumulaterRollingResistanceImpulse );
                        ApplyAngularImpulseBody2( contactManifold ,  _accumulaterRollingResistanceImpulse );

                        //                            //------------------------------ Rolling resistance accumulate apply impulse angular --------------------------//
                        //                            mVelocities[contactManifold->indexBody1].w += -_accumulaterRollingResistanceImpulse * contactManifold->inverseInertiaTensorBody1;
                        //                            mVelocities[contactManifold->indexBody2].w +=  _accumulaterRollingResistanceImpulse * contactManifold->inverseInertiaTensorBody2;


                        //------------------------------ Split Rolling resistance accumulate apply impulse angular --------------------//
                        mSplitVelocities[contactManifold->indexBody1].w += -_accumulaterRollingResistanceSplitImpulse * contactManifold->inverseInertiaTensorBody1;
                        mSplitVelocities[contactManifold->indexBody2].w +=  _accumulaterRollingResistanceSplitImpulse * contactManifold->inverseInertiaTensorBody2;

                    }

                }

            }
            else
            {
                _accumulaterImpuls                        =  0;
                _accumulaterImpulsFriction1               =  0;
                _accumulaterImpulsFriction2               =  0;
                _accumulaterRollingResistanceImpulse      =  Vector3(0,0,0);
                _accumulaterRollingResistanceSplitImpulse =  Vector3(0,0,0);
                contactPoint.isRestingContact = true;

            }

        }



        // If we solve the friction constraints at the center of the contact manifold and there is
        // at least one resting contact point in the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint && contactManifold->nbContacts > 0)
        {

            scalar  &_accumulaterImpulsFriction1               = contactManifold->AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2               = contactManifold->AccumulatedFriction2Impulse;
            scalar  &_accumulatedFrictionTwistImpulse          = contactManifold->AccumulatedFrictionTwistImpulse;
            Vector3 &_accumulaterRollingResistanceImpulse      = contactManifold->AccumulatedRollingResistanceImpulse;
            Vector3 &_accumulaterRollingResistanceSplitImpulse = contactManifold->AccumulatedRollingResistanceSplitImpulse;

            // Project the old friction impulses (with old friction vectors) into the new friction
            // vectors to get the new friction impulses
            //contactManifold->oldFrictionVector1 = contactManifold->frictionVector1;
            //contactManifold->oldFrictionVector2 = contactManifold->frictionVector2;
            Vector3 oldFrictionImpulse =_accumulaterImpulsFriction1 * contactManifold->oldFrictionVector1 +
                                        _accumulaterImpulsFriction2 * contactManifold->oldFrictionVector2;
            _accumulaterImpulsFriction1  = oldFrictionImpulse.Dot(contactManifold->frictionVector1);
            _accumulaterImpulsFriction2  = oldFrictionImpulse.Dot(contactManifold->frictionVector2);




            /**/


			ApplyImpulseBody1( contactManifold ,-contactManifold->frictionVector1 * _accumulaterImpulsFriction1 , contactManifold->r1Friction );
			ApplyImpulseBody2( contactManifold , contactManifold->frictionVector1 * _accumulaterImpulsFriction1 , contactManifold->r2Friction );

            //                // ------ First friction constraint at the center of the contact manifold ------ //

            //                // Compute the impulse P = J^T * lambda
            //                Vector3 linearImpulseBody1  = -contactManifold->frictionVector1 *_accumulaterImpulsFriction1;
            //                Vector3 angularImpulseBody1 = -contactManifold->r1CrossT1 *_accumulaterImpulsFriction1;
            //                Vector3 linearImpulseBody2  =  contactManifold->frictionVector1 *_accumulaterImpulsFriction1;
            //                Vector3 angularImpulseBody2 =  contactManifold->r2CrossT1 *_accumulaterImpulsFriction1;


            //                //------------------------ friction1 accumulation apply impulse linear  -----------------------------//
            //                mVelocities[contactManifold->indexBody1].v += linearImpulseBody1  * contactManifold->massInverseBody1;
            //                mVelocities[contactManifold->indexBody2].v += linearImpulseBody2  * contactManifold->massInverseBody2;

            //                //------------------------ friction1 accumulation apply impulse angular -----------------------------//
            //                mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //                mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;




			ApplyImpulseBody1( contactManifold , -contactManifold->frictionVector2 * _accumulaterImpulsFriction2, contactManifold->r1Friction );
			ApplyImpulseBody2( contactManifold ,  contactManifold->frictionVector2 * _accumulaterImpulsFriction2, contactManifold->r2Friction );


            //                // ------ Second friction constraint at the center of the contact manifold ----- //

            //                // Compute the impulse P = J^T * lambda
            //                linearImpulseBody1  = -contactManifold->frictionVector2 * _accumulaterImpulsFriction2;
            //                angularImpulseBody1 = -contactManifold->r1CrossT2       * _accumulaterImpulsFriction2;
            //                linearImpulseBody2  =  contactManifold->frictionVector2 * _accumulaterImpulsFriction2;
            //                angularImpulseBody2 =  contactManifold->r2CrossT2       * _accumulaterImpulsFriction2;



            //                //------------------------ friction2 accumulation apply impulse linear  -----------------------------//
            //                mVelocities[contactManifold->indexBody1].v += linearImpulseBody1  * contactManifold->massInverseBody1;
            //                mVelocities[contactManifold->indexBody2].v += linearImpulseBody2  * contactManifold->massInverseBody2;

            //                //------------------------ friction2 accumulation apply impulse angular -----------------------------//
            //                mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //                mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;

            /**/



            // ------ Twist friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            Vector3 angularImpulseBody1 = -contactManifold->normal * _accumulatedFrictionTwistImpulse;
            Vector3 angularImpulseBody2 =  contactManifold->normal * _accumulatedFrictionTwistImpulse;


            ApplyAngularImpulseBody1( contactManifold , angularImpulseBody1 );
            ApplyAngularImpulseBody2( contactManifold , angularImpulseBody2 );

            //                //------------------------ twist accumulation apply impulse angular -----------------------------//
            //                mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //                mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;




            // ------ Rolling resistance at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1 = -_accumulaterRollingResistanceImpulse;
            angularImpulseBody2 =  _accumulaterRollingResistanceImpulse;


            ApplyAngularImpulseBody1( contactManifold , angularImpulseBody1 );
            ApplyAngularImpulseBody2( contactManifold , angularImpulseBody2 );

            //                //------------------------ rolling accumulation apply impulse angular  -----------------------------//
            //                mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //                mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;



            // ------ Split Rolling resistance at the center of the contact manifold ------ /

            // Compute the impulse P = J^T * lambda
            Vector3 angularSplitImpulseBody1 = -_accumulaterRollingResistanceSplitImpulse;
            Vector3 angularSplitImpulseBody2 =  _accumulaterRollingResistanceSplitImpulse;

            //------------------------ split rolling accumulation apply impulse angular  -----------------------------//
            mSplitVelocities[contactManifold->indexBody1].w += angularSplitImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            mSplitVelocities[contactManifold->indexBody2].w += angularSplitImpulseBody2 * contactManifold->inverseInertiaTensorBody2;



        }
        else
        {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            contactManifold->AccumulatedFriction1Impulse = 0.0;
            contactManifold->AccumulatedFriction2Impulse = 0.0;
            contactManifold->AccumulatedFrictionTwistImpulse = 0.0;
            contactManifold->AccumulatedRollingResistanceImpulse = Vector3(0,0,0);
            contactManifold->AccumulatedRollingResistanceSplitImpulse = Vector3(0,0,0);
            atLeastOneRestingContactPoint = true;

        }

    }

}

void IContactSolver::SolveVelocityConstraint()
{

    scalar deltaLambda;
    scalar lambdaTemp;

    // For each contact manifold
    for (u32 c=0; c<mNbContactManifolds; c++)
    {

        IContactManifoldSolver* contactManifold = &mContactConstraints[c];


        scalar sumPenetrationImpulse = 0.0;


        // Get the velocities of the bodies
        const Vector3& v1 = mVelocities[contactManifold->indexBody1].v;
        const Vector3& w1 = mVelocities[contactManifold->indexBody1].w;
        const Vector3& v2 = mVelocities[contactManifold->indexBody2].v;
        const Vector3& w2 = mVelocities[contactManifold->indexBody2].w;


        /**
        scalar gamma1 = ISqrt( 1.0 - Dot(mVelocities[contactManifold->indexBody1].v/60.f,mVelocities[contactManifold->indexBody1].v/60.f)/ LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C );
        scalar gamma2 = ISqrt( 1.0 - Dot(mVelocities[contactManifold->indexBody2].v/60.f,mVelocities[contactManifold->indexBody2].v/60.f)/ LIGHT_MAX_VELOCITY_C * LIGHT_MAX_VELOCITY_C );
        /**

        scalar gamma1 = mVelocities[contactManifold->indexBody1].gamma;
        scalar gamma2 = mVelocities[contactManifold->indexBody2].gamma;

        // Get the velocities of the bodies
        const Vector3& v1 = mVelocities[contactManifold->indexBody1].v * gamma1;
        const Vector3& w1 = mVelocities[contactManifold->indexBody1].w * gamma1;
        const Vector3& v2 = mVelocities[contactManifold->indexBody2].v * gamma2;
        const Vector3& w2 = mVelocities[contactManifold->indexBody2].w * gamma2;

        /**/


        for (u32 i = 0; i < contactManifold->nbContacts; ++i)
        {

            IContactPointSolver &contactPoint = contactManifold->contacts[i];
            IContactPoint *cp = contactPoint.externalContact;



            scalar  &_accumulaterImpuls                   = contactPoint.AccumulatedPenetrationImpulse;
            scalar  &_accumulaterImpulsFriction1          = contactPoint.AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2          = contactPoint.AccumulatedFriction2Impulse;
            Vector3 &_accumulaterRollingResistanceImpulse = contactPoint.AccumulatedRollingResistanceImpulse;



            // Compute J*v
            Vector3 deltaV = v2 + w2.Cross(contactPoint.r2) -
                             v1 - w1.Cross(contactPoint.r1);
            scalar deltaVDotN = deltaV.Dot(contactPoint.normal);
            scalar Jv = deltaVDotN;



            // Compute the bias "b" of the constraint
            //scalar beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
            scalar biasPenetrationDepth = 0.0;
            if (contactPoint.penetrationDepth > SLOP)
            {
                // biasPenetrationDepth = -(beta/mTimeStep) * max(0.0f, scalar(contactPoint.penetrationDepth - SLOP));
                biasPenetrationDepth = cp->GetPenetration();
            }

            scalar b = biasPenetrationDepth + contactPoint.restitutionBias;

            if(mIsSplitImpulseActive)
            {
                deltaLambda = -(Jv + contactPoint.restitutionBias) * contactPoint.inversePenetrationMass;
            }
            else
            {
                deltaLambda = -(Jv + b) * contactPoint.inversePenetrationMass;
            }



            lambdaTemp = _accumulaterImpuls;
            _accumulaterImpuls = IMax(_accumulaterImpuls + deltaLambda, scalar(0.0));
            deltaLambda = _accumulaterImpuls - lambdaTemp;


			ApplyImpulseBody1( contactManifold , -contactPoint.normal * deltaLambda , contactPoint.r1 );
			ApplyImpulseBody2( contactManifold ,  contactPoint.normal * deltaLambda , contactPoint.r2 );


            //             //----------------------------- apply impuls linear ----------------------//
            //             mVelocities[contactManifold->indexBody1].v += -contactPoint.normal * deltaLambda * contactManifold->massInverseBody1;
            //             mVelocities[contactManifold->indexBody2].v +=  contactPoint.normal * deltaLambda * contactManifold->massInverseBody2;

            //             //----------------------------- apply impuls angular ----------------------//
            //             mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.normal * deltaLambda) * contactManifold->inverseInertiaTensorBody1;
            //             mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.normal * deltaLambda) * contactManifold->inverseInertiaTensorBody2;




            sumPenetrationImpulse += _accumulaterImpuls;


            // If we do not solve the friction constraints at the center of the contact manifold
            if (!mIsSolveFrictionAtContactManifoldCenterActive)
            {

                // --------- Friction 1 --------- //

                // Compute J*v
                deltaV = v2 + w2.Cross(contactPoint.r2) -
                         v1 - w1.Cross(contactPoint.r1);
                Jv = deltaV.Dot(contactPoint.frictionVector1);

                // Compute the Lagrange multiplier lambda
                deltaLambda = -Jv;
                deltaLambda *= contactPoint.inverseFriction1Mass;
                scalar frictionLimit = contactManifold->frictionCoefficient * _accumulaterImpuls;
                lambdaTemp = _accumulaterImpulsFriction1;
                _accumulaterImpulsFriction1 = IMax(-frictionLimit, IMin(_accumulaterImpulsFriction1 + deltaLambda, frictionLimit));
                deltaLambda = _accumulaterImpulsFriction1 - lambdaTemp;


				ApplyImpulseBody1( contactManifold , -contactPoint.frictionVector1 * deltaLambda , contactPoint.r1 );
				ApplyImpulseBody2( contactManifold ,  contactPoint.frictionVector1 * deltaLambda , contactPoint.r2 );


                //                 //----------------------------- friction1 apply impuls linear ------------------------//
                //                 mVelocities[contactManifold->indexBody1].v += -contactPoint.frictionVector1 * deltaLambda  * contactManifold->massInverseBody1;
                //                 mVelocities[contactManifold->indexBody2].v +=  contactPoint.frictionVector1 * deltaLambda  * contactManifold->massInverseBody2;

                //                 //----------------------------- friction1 apply impuls angular  ----------------------//
                //                 mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.frictionVector1 * deltaLambda ) * contactManifold->inverseInertiaTensorBody1;
                //                 mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.frictionVector1 * deltaLambda ) * contactManifold->inverseInertiaTensorBody2;


                // --------- Friction 2 --------- //

                // Compute J*v
                deltaV = v2 + w2.Cross(contactPoint.r2) -
                         v1 - w1.Cross(contactPoint.r1);
                Jv = deltaV.Dot(contactPoint.frictionVector2);


                // Compute the Lagrange multiplier lambda
                deltaLambda = -Jv;
                deltaLambda *= contactPoint.inverseFriction2Mass;
                frictionLimit = contactManifold->frictionCoefficient * _accumulaterImpuls;
                lambdaTemp = _accumulaterImpulsFriction2;
                _accumulaterImpulsFriction2 = IMax(-frictionLimit, IMin(_accumulaterImpulsFriction2 + deltaLambda, frictionLimit));
                deltaLambda = _accumulaterImpulsFriction2 - lambdaTemp;


				ApplyImpulseBody1( contactManifold , -contactPoint.frictionVector2 * deltaLambda , contactPoint.r1 );
				ApplyImpulseBody2( contactManifold ,  contactPoint.frictionVector2 * deltaLambda , contactPoint.r2 );


                //                 //----------------------------- friction2 apply impuls linear ------------------------//
                //                 mVelocities[contactManifold->indexBody1].v += -contactPoint.frictionVector2 * deltaLambda  * contactManifold->massInverseBody1;
                //                 mVelocities[contactManifold->indexBody2].v +=  contactPoint.frictionVector2 * deltaLambda  * contactManifold->massInverseBody2;

                //                 //----------------------------- friction2 apply impuls angular -----------------------//
                //                 mVelocities[contactManifold->indexBody1].w += contactPoint.r1.cross(-contactPoint.frictionVector2 * deltaLambda ) * contactManifold->inverseInertiaTensorBody1;
                //                 mVelocities[contactManifold->indexBody2].w += contactPoint.r2.cross( contactPoint.frictionVector2 * deltaLambda ) * contactManifold->inverseInertiaTensorBody2;



                // --------- Rolling resistance constraint --------- //
                if ( !mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->rollingResistanceFactor > 0)
                {

                    // Compute J*v
                    const Vector3 JvRolling = w2 - w1;

                    // Compute the Lagrange multiplier lambda
                    Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
                    scalar rollingLimit = contactManifold->rollingResistanceFactor * _accumulaterImpuls;
                    Vector3 lambdaTempRolling = _accumulaterRollingResistanceImpulse;
                    _accumulaterRollingResistanceImpulse = Vector3::Clamp(_accumulaterRollingResistanceImpulse + deltaLambdaRolling, rollingLimit);
                    deltaLambdaRolling = _accumulaterRollingResistanceImpulse - lambdaTempRolling;


                    ApplyAngularImpulseBody1( contactManifold , -deltaLambdaRolling );
                    ApplyAngularImpulseBody2( contactManifold ,  deltaLambdaRolling );

                    //                     //----------------------------- rolling apply impuls angular -----------------------//
                    //                     mVelocities[contactManifold->indexBody1].w += -deltaLambdaRolling * contactManifold->inverseInertiaTensorBody1;
                    //                     mVelocities[contactManifold->indexBody2].w +=  deltaLambdaRolling * contactManifold->inverseInertiaTensorBody2;
                }

            }

        }



        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->nbContacts > 0)
        {


            scalar  &_accumulaterImpulsFriction1          = contactManifold->AccumulatedFriction1Impulse;
            scalar  &_accumulaterImpulsFriction2          = contactManifold->AccumulatedFriction2Impulse;
            scalar  &_accumulatedFrictionTwistImpulse     = contactManifold->AccumulatedFrictionTwistImpulse;
            Vector3 &_accumulaterRollingResistanceImpulse = contactManifold->AccumulatedRollingResistanceImpulse;

            // ------ First friction constraint at the center of the contact manifol ------ //

            // Compute J*v
            Vector3 deltaV = v2 + w2.Cross(contactManifold->r2Friction) -
                             v1 - w1.Cross(contactManifold->r1Friction);
            scalar Jv = deltaV.Dot(contactManifold->frictionVector1);

            // Compute the Lagrange multiplier lambda
            scalar deltaLambda = -Jv * contactManifold->inverseFriction1Mass;
            scalar frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = _accumulaterImpulsFriction1;
            _accumulaterImpulsFriction1 = IMax(-frictionLimit, IMin(_accumulaterImpulsFriction1 + deltaLambda, frictionLimit));
            deltaLambda = _accumulaterImpulsFriction1 - lambdaTemp;




			ApplyImpulseBody1( contactManifold ,-contactManifold->frictionVector1 * deltaLambda , contactManifold->r1Friction );
			ApplyImpulseBody2( contactManifold , contactManifold->frictionVector1 * deltaLambda , contactManifold->r2Friction );



            //             // Compute the impulse P=J^T * lambda
            //             Vector3 linearImpulseBody1  = -contactManifold->frictionVector1 * deltaLambda;
            //             Vector3 angularImpulseBody1 = -contactManifold->r1CrossT1       * deltaLambda;
            //             Vector3 linearImpulseBody2  =  contactManifold->frictionVector1 * deltaLambda;
            //             Vector3 angularImpulseBody2 =  contactManifold->r2CrossT1       * deltaLambda;

            //             //----------------------------- friction1 apply impuls linear  ------------------------//
            //             mVelocities[contactManifold->indexBody1].v += linearImpulseBody1  * contactManifold->massInverseBody1;
            //             mVelocities[contactManifold->indexBody2].v += linearImpulseBody2  * contactManifold->massInverseBody2;

            //             //----------------------------- friction1 apply impuls angular ------------------------//
            //             mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //             mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;

            /**/


            // ------ Second friction constraint at the center of the contact manifol ----- //

            // Compute J*v
            deltaV = v2 + w2.Cross(contactManifold->r2Friction) -
                     v1 - w1.Cross(contactManifold->r1Friction);
            Jv = deltaV.Dot(contactManifold->frictionVector2);

            // Compute the Lagrange multiplier lambda
            deltaLambda = -Jv * contactManifold->inverseFriction2Mass;
            frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = _accumulaterImpulsFriction2;
            _accumulaterImpulsFriction2 = IMax(-frictionLimit, IMin(_accumulaterImpulsFriction2 + deltaLambda, frictionLimit));
            deltaLambda = _accumulaterImpulsFriction2 - lambdaTemp;




			ApplyImpulseBody1( contactManifold ,-contactManifold->frictionVector2 * deltaLambda , contactManifold->r1Friction );
			ApplyImpulseBody2( contactManifold , contactManifold->frictionVector2 * deltaLambda , contactManifold->r2Friction );


            //             // Compute the impulse P=J^T * lambda
            //             linearImpulseBody1  = -contactManifold->frictionVector2 * deltaLambda;
            //             angularImpulseBody1 = -contactManifold->r1CrossT2       * deltaLambda;
            //             linearImpulseBody2  =  contactManifold->frictionVector2 * deltaLambda;
            //             angularImpulseBody2 =  contactManifold->r2CrossT2       * deltaLambda;


            //             //----------------------------- friction2 apply impuls linear  ------------------------//
            //             mVelocities[contactManifold->indexBody1].v += linearImpulseBody1  * contactManifold->massInverseBody1;
            //             mVelocities[contactManifold->indexBody2].v += linearImpulseBody2  * contactManifold->massInverseBody2;

            //             //----------------------------- friction2 apply impuls angular ------------------------//
            //             mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //             mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;





            // ------ Twist friction constraint at the center of the contact manifol ------ //

            // Compute J*v
            deltaV = w2 - w1;
            Jv = deltaV.Dot(contactManifold->normal);

            deltaLambda = -Jv * (contactManifold->inverseTwistFrictionMass);
            frictionLimit = contactManifold->frictionCoefficient * sumPenetrationImpulse;
            lambdaTemp = _accumulatedFrictionTwistImpulse;
            _accumulatedFrictionTwistImpulse = IMax(-frictionLimit, IMin(_accumulatedFrictionTwistImpulse + deltaLambda, frictionLimit));
            deltaLambda = _accumulatedFrictionTwistImpulse - lambdaTemp;

            // Compute the impulse P=J^T * lambda
            Vector3 angularImpulseBody1 = -contactManifold->normal * deltaLambda;
            Vector3 angularImpulseBody2 =  contactManifold->normal * deltaLambda;


            ApplyAngularImpulseBody1( contactManifold , angularImpulseBody1 );
            ApplyAngularImpulseBody2( contactManifold , angularImpulseBody2 );

            //             //----------------------------- twist apply impuls angular -----------------------//
            //             mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
            //             mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;

            /**/
            // --------- Rolling resistance constraint at the center of the contact manifold --------- //
            if (contactManifold->rollingResistanceFactor > 0)
            {

                // Compute J*v
                const Vector3 JvRolling = w2 - w1;

                // Compute the Lagrange multiplier lambda
                Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
                scalar rollingLimit = contactManifold->rollingResistanceFactor * sumPenetrationImpulse;
                Vector3 lambdaTempRolling = _accumulaterRollingResistanceImpulse;
                _accumulaterRollingResistanceImpulse = Vector3::Clamp(_accumulaterRollingResistanceImpulse + deltaLambdaRolling, rollingLimit);
                deltaLambdaRolling = _accumulaterRollingResistanceImpulse - lambdaTempRolling;

                // Compute the impulse P=J^T * lambda
                angularImpulseBody1 = -deltaLambdaRolling;
                angularImpulseBody2 =  deltaLambdaRolling;


                ApplyAngularImpulseBody1( contactManifold , angularImpulseBody1 );
                ApplyAngularImpulseBody2( contactManifold , angularImpulseBody2 );


                //                 //----------------------------- rolling apply impuls angular -----------------------//
                //                 mVelocities[contactManifold->indexBody1].w += angularImpulseBody1 * contactManifold->inverseInertiaTensorBody1;
                //                 mVelocities[contactManifold->indexBody2].w += angularImpulseBody2 * contactManifold->inverseInertiaTensorBody2;

            }
            /**/
        }


    }

}

void IContactSolver::SolveSplitVelocityConstraint()
{
    // For each contact manifold
    for (u32 c=0; c<mNbContactManifolds; c++)
    {
        IContactManifoldSolver* contactManifold = &mContactConstraints[c];


        const Vector3& v1Split = mSplitVelocities[contactManifold->indexBody1].v;
        const Vector3& w1Split = mSplitVelocities[contactManifold->indexBody1].w;
        const Vector3& v2Split = mSplitVelocities[contactManifold->indexBody2].v;
        const Vector3& w2Split = mSplitVelocities[contactManifold->indexBody2].w;



        scalar sumPenetrationSplitImpulse = 0.0;

        for (u32 i = 0; i < contactManifold->nbContacts; ++i)
        {

            IContactPointSolver &contactPoint = contactManifold->contacts[i];
            IContactPoint *cp = contactPoint.externalContact;


            scalar  &_accumulaterPenetrationSplit = contactPoint.AccumulatedPenetrationSplitImpulse;
            Vector3 &_accumulaterRollingResistanceSplitImpulse = contactPoint.AccumulatedRollingResistanceSplitImpulse;


            Vector3 deltaVSplit = v2Split + w2Split.Cross(contactPoint.r2) -
                                   v1Split - w1Split.Cross(contactPoint.r1);

            scalar JvSplit = deltaVSplit.Dot(contactPoint.normal);


            scalar biasPenetrationDepth = -cp->GetPenetration();
            scalar deltaLambdaSplit = -(JvSplit + biasPenetrationDepth) * contactPoint.inversePenetrationMass;


            scalar lambdaTempSplit = _accumulaterPenetrationSplit;
            _accumulaterPenetrationSplit = IMax( _accumulaterPenetrationSplit + deltaLambdaSplit, scalar(0.0));
            scalar deltaLambda = _accumulaterPenetrationSplit - lambdaTempSplit;




            sumPenetrationSplitImpulse += _accumulaterPenetrationSplit;


             //----------------------------- split apply impuls linear ----------------------//
            mSplitVelocities[contactManifold->indexBody1].v += -contactPoint.normal * deltaLambda * contactManifold->massInverseBody1;
            mSplitVelocities[contactManifold->indexBody2].v +=  contactPoint.normal * deltaLambda * contactManifold->massInverseBody2;

             //----------------------------- split apply impuls angular ----------------------//
            mSplitVelocities[contactManifold->indexBody1].w += contactPoint.r1.Cross(-contactPoint.normal * deltaLambda) * contactManifold->inverseInertiaTensorBody1;
            mSplitVelocities[contactManifold->indexBody2].w += contactPoint.r2.Cross( contactPoint.normal * deltaLambda) * contactManifold->inverseInertiaTensorBody2;


            if( !mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->rollingResistanceFactor > 0)
            {

                // Compute J*v
                const Vector3 JvRolling = w2Split  - w1Split;

                // Compute the Lagrange multiplier lambda
                Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvRolling);
                scalar rollingLimit = contactManifold->rollingResistanceFactor * _accumulaterPenetrationSplit;
                Vector3 lambdaTempRolling = _accumulaterRollingResistanceSplitImpulse;
                _accumulaterRollingResistanceSplitImpulse = Vector3::Clamp(_accumulaterRollingResistanceSplitImpulse + deltaLambdaRolling, rollingLimit);
                deltaLambdaRolling = _accumulaterRollingResistanceSplitImpulse - lambdaTempRolling;

                //----------------------------- split rolling apply impuls angular -----------------------//
                mSplitVelocities[contactManifold->indexBody1].w += -deltaLambdaRolling * contactManifold->inverseInertiaTensorBody1;
                mSplitVelocities[contactManifold->indexBody2].w +=  deltaLambdaRolling * contactManifold->inverseInertiaTensorBody2;

            }

        }





        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive && contactManifold->nbContacts > 0)
        {

            Vector3 &_accumulaterRollingResistanceSplitImpulse = contactManifold->AccumulatedRollingResistanceSplitImpulse;

            //--------- Rolling resistance constraint at the center of the contact manifold ---------//
            if (contactManifold->rollingResistanceFactor > 0)
            {

                // Compute J*v
                const Vector3 JvSplitRolling = w2Split - w1Split;

                // Compute the Lagrange multiplier lambda
                Vector3 deltaLambdaRolling = contactManifold->inverseRollingResistance * (-JvSplitRolling);
                scalar rollingLimit = contactManifold->rollingResistanceFactor * sumPenetrationSplitImpulse;
                Vector3 lambdaTempRolling = _accumulaterRollingResistanceSplitImpulse;
                _accumulaterRollingResistanceSplitImpulse = Vector3::Clamp(_accumulaterRollingResistanceSplitImpulse + deltaLambdaRolling, rollingLimit);
                deltaLambdaRolling = _accumulaterRollingResistanceSplitImpulse - lambdaTempRolling;


                //----------------------------- split rolling apply impuls angular -----------------------//
                mSplitVelocities[contactManifold->indexBody1].w += -deltaLambdaRolling * contactManifold->inverseInertiaTensorBody1;
                mSplitVelocities[contactManifold->indexBody2].w +=  deltaLambdaRolling * contactManifold->inverseInertiaTensorBody2;

            }
        }

    }
}

void IContactSolver::StoreImpulsesWarmstart()
{

    // For each contact manifold
    for (u32 c=0; c<mNbContactManifolds; c++)
    {
        IContactManifoldSolver& manifold = mContactConstraints[c];

        for (u32 i=0; i<manifold.nbContacts; i++)
        {

            IContactPointSolver& contactPoint = manifold.contacts[i];

            //********************************************************************************************************
            contactPoint.externalContact->SetAccumulatedPenetrationImpulse(contactPoint.AccumulatedPenetrationImpulse);
            contactPoint.externalContact->SetAccumulatedFrictionImpulse1(contactPoint.AccumulatedFriction1Impulse);
            contactPoint.externalContact->SetAccumulatedFrictionImpulse2(contactPoint.AccumulatedFriction1Impulse);
            contactPoint.externalContact->SetAccumulatedRollingResistanceImpulse(contactPoint.AccumulatedRollingResistanceSplitImpulse);
            contactPoint.externalContact->SetFrictionVector1(contactPoint.frictionVector1);
            contactPoint.externalContact->SetFrictionVector2(contactPoint.frictionVector2);
            //********************************************************************************************************///

        }

        //********************************************************************************************************//
        manifold.externalContactManifold->SetAccumulatedFrictionImpulse1(manifold.AccumulatedFriction1Impulse);
        manifold.externalContactManifold->SetAccumulatedFrictionImpulse2(manifold.AccumulatedFriction2Impulse);
        manifold.externalContactManifold->SetAccumulatedFrictionTwistImpulse(manifold.AccumulatedFrictionTwistImpulse);
        manifold.externalContactManifold->SetAccumulatedRollingResistanceImpulse(manifold.AccumulatedRollingResistanceImpulse);
        manifold.externalContactManifold->SetFrictionVector1(manifold.frictionVector1);
        manifold.externalContactManifold->SetFrictionVector2(manifold.frictionVector2);
        //********************************************************************************************************//

    }


}

void IContactSolver::Cleanup()
{
    if (mContactConstraints != nullptr)
    {
        delete[] mContactConstraints;
        mContactConstraints = nullptr;
    }
}


/**********************************************************************************************/


SIMD_INLINE scalar IContactSolver::ComputeMixedRestitutionFactor( IRigidBody*  body1 ,
                                                                  IRigidBody*  body2) const
{
    scalar restitution1 = body1->GetMaterial().GetBounciness();
    scalar restitution2 = body2->GetMaterial().GetBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

SIMD_INLINE scalar IContactSolver::ComputeMixedFrictionCoefficient( IRigidBody*  body1 ,
                                                                    IRigidBody*  body2) const
{
    // Use the geometric mean to compute the mixed friction coefficient
    return ISqrt(body1->GetMaterial().GetFrictionCoefficient() *
                 body2->GetMaterial().GetFrictionCoefficient());
}

SIMD_INLINE scalar IContactSolver::ComputeMixedRollingResistance( IRigidBody*  body1 ,
                                                                  IRigidBody*  body2) const
{
    return scalar(0.5f) * (body1->GetMaterial().GetRollingResistance() +
                           body2->GetMaterial().GetRollingResistance());
}



SIMD_INLINE void IContactSolver::ComputeFrictionVectors( const Vector3& deltaVelocity, IContactPointSolver& contactPoint) const
{
    assert(contactPoint.normal.Length() > 0.0);


    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity  = deltaVelocity.Dot(contactPoint.normal) * contactPoint.normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    scalar lengthTangenVelocity = tangentVelocity.Length();
    if (lengthTangenVelocity > MACHINE_EPSILON)
    {
        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contactPoint.frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else
    {
        // Get any orthogonal vector to the normal as the first friction vector
        contactPoint.frictionVector1 = contactPoint.normal.GetOneUnitOrthogonalVector();
    }


    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contactPoint.frictionVector2 =contactPoint.normal.Cross(contactPoint.frictionVector1).GetUnit();


}



SIMD_INLINE void IContactSolver::ComputeFrictionVectors( const Vector3& deltaVelocity , IContactManifoldSolver* contact ) const
{
    assert(contact->normal.Length() > 0.0);


    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity = deltaVelocity.Dot(contact->normal) * contact->normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    scalar lengthTangenVelocity = tangentVelocity.Length();
    if (lengthTangenVelocity > MACHINE_EPSILON)
    {
        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contact->frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else
    {
        // Get any orthogonal vector to the normal as the first friction vector
        contact->frictionVector1 = contact->normal.GetOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contact->frictionVector2 = contact->normal.Cross(contact->frictionVector1).GetUnit();

}



}
