#ifndef IRIGIDBODY_H
#define IRIGIDBODY_H


#include "IMaterial/IPhysicsMaterial.h"
#include "../IGeometry/ICollision/ICollisionWorld.h"
#include "../ICommon/IMemory/IList.h"


namespace IEngine
{

using namespace IMath;

//// Class declarations
class   IJoint;

///List element joint
typedef IListElement<IJoint> JointListElement;


enum TypeBody { RIGIDBODY_TYPE , RIGIDREALTIVITY_TYPE };


class IRigidBody : public ICollisionBody
{

protected:


    //-------------------- Attributes --------------------//

    TypeBody mTypeBody;


    /// Rotate is active
    bool  mIsRotate;

    /// Move is active
    bool  mIsMove;


    /// Material properties of the rigid body
    IPhysicsMaterial mMaterial;


    /// First element of the linked list of joints involving this body
    JointListElement*  mJointsList;


    /// Intial mass of the body
    scalar mInitMass;



    /// Center of mass of the body in local-space coordinates.
    /// The center of mass can therefore be different from the body origin
    Vector3 mCenterOfMassLocal;

    /// Center of mass of the body in world-space coordinates
    Vector3 mCenterOfMassWorld;




    /// Linear velocity damping factor
    scalar mLinearDamping;

    /// Angular velocity damping factor
    scalar mAngularDamping;


    /// Linear velocity of the body
    Vector3 mLinearVelocity;

    /// Angular velocity of the body
    Vector3 mAngularVelocity;


    /// Linear velocity of the body
    Vector3 mSplitLinearVelocity;

    /// Angular velocity of the body
    Vector3 mSplitAngularVelocity;


    /// Current external force on the body
    Vector3 mExternalForce;

    /// Current external torque on the body
    Vector3 mExternalTorque;








    /// Inverse of the mass of the body
    scalar     mMassInverse;


    /// Local inertia tensor of the body (in local-space) with respect to the
    /// center of mass of the body
    Matrix3 mInertiaTensorLocal;

    /// Inverse of the inertia tensor of the body
    Matrix3 mInertiaTensorLocalInverse;

    /// Inertia tensor of the body
    Matrix3 mInertiaTensorWorldInverse;


    /// True if the gravity needs to be applied to this rigid body
    bool mIsGravityEnabled;





    /// Private copy-constructor
    IRigidBody(const  IRigidBody& body);

    /// Private assignment operator
    IRigidBody& operator=(const  IRigidBody& body);



public:

    IRigidBody( const Transform& transform, IContactManager *ContactManager, bodyindex id , TypeBody _typeBody = TypeBody::RIGIDBODY_TYPE );


    virtual ~IRigidBody();



    /// Add a collision shape to the body.
    virtual IProxyShape *AddCollisionShape(ICollisionShape* collisionShape , scalar mass ,  const Transform& transform = Transform::Identity() );

    /// Remove a collision shape from the body
    virtual void RemoveCollisionShape(const IProxyShape* proxyShape);


    //============================== Physics ==================================//

    /// Set the type of the body (static, kinematic or dynamic)
    void SetType(BodyType type);


    /// Set the variable to know whether or not the body is sleeping
    void SetIsSleeping(bool isSleeping);


    /// Set the variable to know whether or not the body is stop motion
    void StopMotion();

    /// Update the broad-phase state for this body (because it has moved for instance)
    virtual void UpdateBroadPhaseStatee(scalar _timeStep) const;


    /// remove of the list joints
    virtual void RemoveJointFromJointsList( const IJoint* joint);


    /// Recompute the center of mass, total mass and inertia tensor of the body using all
    /// the collision shapes attached to the body.
    virtual void RecomputeMassInformation();


    /// Update the transform of the body after a change of the center of mass
    void UpdateTransformWithCenterOfMass();


    //============================== Apply =====================================//


    /// apply force
    virtual void ApplyForce(const Vector3& force, const Vector3& point);
    virtual void ApplyTorque(const Vector3& torque);
    virtual void ApplyForceToCenterOfMass(const Vector3& force);




    /// apply impulse
    virtual void ApplyImpulse(const Vector3& impuls , const Vector3& point);
    virtual void ApplyImpulseAngular(const Vector3&  impuls );
    virtual void ApplyImpulseLinear(const Vector3&  impuls );


    //==========================================================================//


    IPhysicsMaterial GetMaterial() const;


    bool IsRotate() const;
    void SetIsRotate(bool newIsRotate);

    void setIsMove(bool newIsMove);
    bool isMove() const;


    /// Inertia to of mass
    scalar GetMass() const;
    scalar GetInverseMass() const;


    /// Inertia tensor to Body
    Matrix3  GetInertiaTensorLocal() const;
    Matrix3  GetInertiaTensorInverseLocal() const;


    Matrix3  GetInertiaTensorWorld() const;
    Matrix3  GetInertiaTensorInverseWorld() const;


    Vector3 GetLinearVelocity() const;
    Vector3 GetAngularVelocity() const;


    scalar GetLinearDamping() const;
    scalar GetAngularDamping() const;


    void SetMaterial(const IPhysicsMaterial &material);
    void SetLinearDamping(const scalar &linearDamping);
    void SetAngularDamping(const scalar &angularDamping);


    Vector3 CenterOfMassWorld() const;
    void SetCenterOfMassWorld(const Vector3 &centerOfMassWorld);


    // -------------------- Friendships -------------------- //
    friend class IDynamicsWorld;
    friend class IIsland;
    friend class IContactSolver;


    friend class IFixedJoint;
    friend class IBallAndSocketJoint;
    friend class IHingeJoint;
    friend class ISliderJoint;



};


//void IRigidBody::updateBroadPhaseStatee(scalar _timeStep) const
//{
////    //DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
////    const Vector3 displacement =  mLinearVelocity * _timeStep;

////    // For all the proxy collision shapes of the body
////    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
////    {
////        // Recompute the world-space AABB of the collision shape
////        IAABB aabb;
////        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform());

////        // Update the broad-phase state for the proxy collision shape
////        mCollisionDetection->updateProxyCollisionShape(shape, aabb, displacement);
////    }
//}


SIMD_INLINE Vector3 IRigidBody::GetLinearVelocity() const
{
    return mLinearVelocity;
}

SIMD_INLINE Vector3 IRigidBody::GetAngularVelocity() const
{
    return mAngularVelocity;
}


SIMD_INLINE scalar IRigidBody::GetLinearDamping() const
{
    return mLinearDamping;
}

SIMD_INLINE scalar IRigidBody::GetAngularDamping() const
{
    return mAngularDamping;
}


}

#endif // IRIGIDBODY_H
