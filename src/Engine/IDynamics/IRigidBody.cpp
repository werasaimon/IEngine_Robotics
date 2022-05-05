#include "IRigidBody.h"



namespace IEngine
{


namespace
{
/// Helper function for daping vector*s
////////////////////////////////////////////////////////////////
/// Helper function for daping vector*s
static void Damping( Vector3& Velocity , const scalar& min_damping , const scalar& damping )
{
    if( Velocity.LengthSquare()  < min_damping )
    {
        if( Velocity.Length() > damping )
        {
            Velocity -= ( Velocity.GetUnit() * damping);
        }
        else
        {
            Velocity  = Vector3(0,0,0);
        }
    }
}
////////////////////////////////////////////////////////////////

}





    




IRigidBody::IRigidBody(const Transform &transform, IContactManager *ContactManager, bodyindex id, TypeBody _typeBody)
    : ICollisionBody( transform , ContactManager , id ),
      mTypeBody(_typeBody) ,
      mIsRotate(true),
      mIsMove(true),
      mJointsList(nullptr),
      mInitMass(scalar(1.0)),
      mCenterOfMassLocal(0, 0, 0),
      mCenterOfMassWorld(transform.GetPosition()),
      mLinearDamping(scalar(0.004)),
      mAngularDamping(scalar(0.004)),
      mLinearVelocity(Vector3(0,0,0)),
      mAngularVelocity(Vector3(0,0,0)),
      mExternalForce(Vector3(0,0,0)),
      mExternalTorque(Vector3(0,0,0)),
      mIsGravityEnabled(true)
{
    /// World transform
    mTransform = (transform);

    /// Compute the inverse mass
    mMassInverse = (mInitMass > scalar(0))? scalar(1.0) / mInitMass : scalar(0);

}


IRigidBody::~IRigidBody()
{
    // Remove all the proxy collision shapes of the body
    ICollisionBody::RemoveAllCollisionShapes();
}


IProxyShape *IRigidBody::AddCollisionShape(ICollisionShape *collisionShape, scalar mass, const Transform &transform)
{
    assert(mass > scalar(0.0));

    // Create a new proxy collision shape to attach the collision shape to the body
    IProxyShape* proxyShape = new IProxyShape(this, collisionShape, transform , mass);


    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == nullptr)
    {
        mProxyCollisionShapes = proxyShape;
    }
    else
    {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }

    // Compute the world-space AABB of the new collision shape
    IAABBox3D aabb;
    collisionShape->ComputeAABB(aabb, mTransform * transform);

    // Notify the collision detection about this new collision shape
    mContactManager->AddProxyCollisionShape(proxyShape, aabb);

    mNbCollisionShapes++;

    // Recompute the center of mass, total mass and inertia tensor of the body with the new
    // collision shape
    RecomputeMassInformation();

    // Return a pointer to the proxy collision shape
    return proxyShape;
}

void IRigidBody::RemoveCollisionShape(const IProxyShape *proxyShape)
{
    // Remove the collision shape
    ICollisionBody::RemoveCollisionShape(proxyShape);

    // Recompute the total mass, center of mass and inertia tensor
    RecomputeMassInformation();
}


void IRigidBody::UpdateTransformWithCenterOfMass()
{
    // Translate the body according to the translation of the center of mass position
    mTransform.SetPosition(mCenterOfMassWorld - mTransform.GetBasis() * mCenterOfMassLocal);
}



void IRigidBody::SetType(BodyType type)
{
    //if (mType == type) return;
    ICollisionBody::SetType(type);

    mType = type;

    // Recompute the total mass, center of mass and inertia tensor
    RecomputeMassInformation();

    // If it is a static body
    if (mType == STATIC)
    {
        // Reset the velocity to zero
        mLinearVelocity=Vector3(0,0,0);
        mAngularVelocity=Vector3(0,0,0);
        mExternalForce=Vector3(0,0,0);
        mExternalTorque=Vector3(0,0,0);
    }

    // If it is a static or a kinematic body
    if (mType == STATIC || mType == KINEMATIC)
    {
        // Reset the inverse mass and inverse inertia tensor to zero
        mMassInverse = scalar(0.0);
        mInertiaTensorLocal.SetToZero();
        mInertiaTensorLocalInverse.SetToZero();

    }
    else
    {
        // If it is a dynamic body
        mMassInverse = scalar(1.0) / mInitMass;
        mInertiaTensorLocalInverse = mInertiaTensorLocal.GetInverse();
    }

    // Awake the body
    SetIsSleeping(false);


    //UpdateMatrices();

    // Remove all the contacts with this body
    ResetContactManifoldsList();

    // Ask the broad-phase to test again the collision shapes of the body for collision
    // detection (as if the body has moved)
    AskForBroadPhaseCollisionCheck();

    // Reset the force and torque on the body
    mExternalForce=Vector3(0,0,0);
    mExternalTorque=Vector3(0,0,0);

}

void IRigidBody::SetIsSleeping(bool isSleeping)
{
    if (isSleeping)
    {
        // Absolutely Stop motion
        mLinearVelocity=Vector3(0,0,0);
        mAngularVelocity=Vector3(0,0,0);
        mExternalForce=Vector3(0,0,0);
        mExternalTorque=Vector3(0,0,0);
    }

    IBody::SetIsSleeping(isSleeping);
}

void IRigidBody::StopMotion()
{
    // Absolutely Stop motion
    mLinearVelocity=Vector3(0,0,0);
    mAngularVelocity=Vector3(0,0,0);
    mExternalForce=Vector3(0,0,0);
    mExternalTorque=Vector3(0,0,0);
}

void IRigidBody::UpdateBroadPhaseStatee(scalar _timeStep) const
{
    //DynamicsWorld& world = static_cast<DynamicsWorld&>(mWorld);
    const Vector3 displacement =  mLinearVelocity * _timeStep;

    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Recompute the world-space AABB of the collision shape
        IAABBox3D aabb;
        shape->GetCollisionShape()->ComputeAABB(aabb, mTransform * shape->GetLocalToBodyTransform());

        // Update the broad-phase state for the proxy collision shape
        mContactManager->UpdateProxyCollisionShape(shape, aabb, displacement);
    }
}



void IRigidBody::RemoveJointFromJointsList(const IJoint *joint)
{

    assert(joint != NULL);
    assert(mJointsList != NULL);

    // Remove the joint from the linked list of the joints of the first body
    if (mJointsList->GetPointer() == joint)
    {   // If the first element is the one to remove
        JointListElement* elementToRemove = mJointsList;
        mJointsList = elementToRemove->GetNext();
        delete elementToRemove;
    }
    else
    {  // If the element to remove is not the first one in the list
        JointListElement* currentElement = mJointsList;
        while (currentElement->GetNext() != nullptr)
        {
            if (currentElement->GetNext()->GetPointer() == joint)
            {
                JointListElement* elementToRemove = currentElement->GetNext();
                currentElement/*->next */ = elementToRemove->GetNext();
                delete elementToRemove;
                break;
            }
            currentElement = currentElement->GetNext();
        }
    }


}

void IRigidBody::RecomputeMassInformation()
{
    mInitMass = scalar(0.0);
    mMassInverse = scalar(0.0);
    mInertiaTensorLocal.SetToZero();
    mInertiaTensorLocalInverse.SetToZero();
    mCenterOfMassLocal.SetToZero();

    // If it is STATIC or KINEMATIC body
    if (mType == STATIC || mType == KINEMATIC)
    {
        mCenterOfMassWorld = mTransform.GetPosition();
        return;
    }

    assert(mType == DYNAMIC);

    // Compute the total mass of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        mInitMass += shape->GetMass();
        mCenterOfMassLocal += shape->GetLocalToBodyTransform().GetPosition() * shape->GetMass();
    }

    if (mInitMass > scalar(0.0))
    {
        mMassInverse = scalar(1.0) / mInitMass;
    }
    else
    {
        mInitMass = scalar(1.0);
        mMassInverse = scalar(1.0);
    }

    // Compute the center of mass
    const Vector3 oldCenterOfMass = mCenterOfMassWorld;
    mCenterOfMassLocal *= mMassInverse;
    mCenterOfMassWorld  =  mTransform * mCenterOfMassLocal;

    // Compute the total mass and inertia tensor using all the collision shapes
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Get the inertia tensor of the collision shape in its local-space
        Matrix3 inertiaTensor = shape->GetCollisionShape()->ComputeLocalInertiaTensor2(shape->GetMass(),mTransform.GetBasis());

        // Convert the collision shape inertia tensor into the local-space of the body
        const Transform& shapeTransform = shape->GetLocalToBodyTransform();
        Matrix3 rotationMatrix = shapeTransform.GetBasis();
        inertiaTensor = rotationMatrix * inertiaTensor * rotationMatrix.GetTranspose();


        // Use the parallel axis theorem to convert the inertia tensor w.r.t the collision shape
        // center into a inertia tensor w.r.t to the body origin.
        Vector3   offset = shapeTransform.GetPosition() - mCenterOfMassLocal;
        scalar     offsetSquare = offset.LengthSquare();
        Matrix3 offsetMatrix;
        offsetMatrix[0].SetAllValues(offsetSquare, scalar(0.0), scalar(0.0));
        offsetMatrix[1].SetAllValues(scalar(0.0), offsetSquare, scalar(0.0));
        offsetMatrix[2].SetAllValues(scalar(0.0), scalar(0.0), offsetSquare);
        offsetMatrix[0] += offset * (-offset.x);
        offsetMatrix[1] += offset * (-offset.y);
        offsetMatrix[2] += offset * (-offset.z);
        offsetMatrix *= shape->GetMass();

        mInertiaTensorLocal += inertiaTensor + offsetMatrix;
    }

    // Compute the local inverse inertia tensor
    mInertiaTensorLocalInverse = mInertiaTensorLocal.GetInverse();

    // Update the linear velocity of the center of mass
   // mLinearVelocity += mAngularVelocity.Cross(mCenterOfMassWorld - oldCenterOfMass);

}


//============================== Apply =====================================//

void IRigidBody::ApplyForce(const Vector3 &force, const Vector3 &point)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the force
    mExternalForce += mMassInverse * force;
    // Add the torque
    mExternalTorque += GetInertiaTensorInverseWorld() * ((point - mCenterOfMassWorld).Cross(force));

}

void IRigidBody::ApplyTorque(const Vector3 &torque)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the torque
    mExternalTorque += GetInertiaTensorInverseWorld() * torque;
}

void IRigidBody::ApplyForceToCenterOfMass(const Vector3 &force)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Add the force
    mExternalForce += mMassInverse * force;
}

void IRigidBody::ApplyImpulse(const Vector3 &impuls, const Vector3 &point)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    mLinearVelocity  += mMassInverse * impuls;
    mAngularVelocity += GetInertiaTensorInverseWorld() * (point - mCenterOfMassWorld).Cross(impuls);
}

void IRigidBody::ApplyImpulseAngular(const Vector3 &impuls)
{
    // If it is not a dynamic body, we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    // Awake the body if it was sleeping
    mAngularVelocity += GetInertiaTensorInverseWorld() * impuls;

}

void IRigidBody::ApplyImpulseLinear(const Vector3 &impuls)
{
    // If it is not a dynamic body or sleeping , we do nothing
    if (mType != DYNAMIC || mIsSleeping ) return;

    mLinearVelocity += impuls * mMassInverse;
}

//=======================================================================//


IPhysicsMaterial IRigidBody::GetMaterial() const
{
    return mMaterial;
}


bool IRigidBody::IsRotate() const
{
    return mIsRotate;
}

void IRigidBody::SetIsRotate(bool newIsRotate)
{
    mIsRotate = newIsRotate;
}

bool IRigidBody::isMove() const
{
    return mIsMove;
}

void IRigidBody::setIsMove(bool newIsMove)
{
    mIsMove = newIsMove;
}


/// Massa of the body
scalar IRigidBody::GetMass() const
{
    return mInitMass;// * gamma
}

/// Inverse massa of the body
scalar IRigidBody::GetInverseMass() const
{
    return mMassInverse;// * gamma
}



// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inertia tensor matrix of the body (in local-space coordinates)
 */
/*SIMD_INLINE  */Matrix3 IRigidBody::GetInertiaTensorLocal() const
{
    return mInertiaTensorLocal;// * gamma
}


// Return the local inertia tensor of the body (in local-space coordinates)
/**
 * @return The 3x3 inverse inertia tensor matrix of the body (in local-space coordinates)
 */
/*SIMD_INLINE  */Matrix3 IRigidBody::GetInertiaTensorInverseLocal() const
{
    return mInertiaTensorLocalInverse;// * gamma
}



// Return the inertia tensor in world coordinates.
/// The inertia tensor I_w in world coordinates is computed
/// with the local inertia tensor I_b in body coordinates
/// by I_w = R * I_b * R^T
/// where R is the rotation matrix (and R^T its transpose) of
/// the current orientation quaternion of the body
/**
 * @return The 3x3 inertia tensor matrix of the body in world-space coordinates
 */
/*SIMD_INLINE */Matrix3 IRigidBody::GetInertiaTensorWorld() const
{
   // Compute and return the inertia tensor in world coordinates
   return mTransform.GetBasis() * GetInertiaTensorLocal() * mTransform.GetBasis().GetTranspose();
}




/*SIMD_INLINE */Matrix3 IRigidBody::GetInertiaTensorInverseWorld() const
{
    // TODO : DO NOT RECOMPUTE THE MATRIX MULTIPLICATION EVERY TIME. WE NEED TO STORE THE
    //        INVERSE WORLD TENSOR IN THE CLASS AND UPLDATE IT WHEN THE ORIENTATION OF THE BODY CHANGES

    // Compute and return the inertia tensor in world coordinates
    return (mTransform.GetBasis() * GetInertiaTensorInverseLocal() * mTransform.GetBasis().GetTranspose());
}


void IRigidBody::SetMaterial(const IPhysicsMaterial &material)
{
    mMaterial = material;
}

void IRigidBody::SetLinearDamping(const scalar &linearDamping)
{
    mLinearDamping = linearDamping;
}

void IRigidBody::SetAngularDamping(const scalar &angularDamping)
{
    mAngularDamping = angularDamping;
}

Vector3 IRigidBody::CenterOfMassWorld() const
{
    return mCenterOfMassWorld;
}

void IRigidBody::SetCenterOfMassWorld(const Vector3 &centerOfMassWorld)
{
    mCenterOfMassWorld = centerOfMassWorld;
}


}
