#include "ICollisionBody.h"

#include "../IProxyShape.h"
#include "../IRaycastInfo.h"
#include "../../IAABBox3D.h"
#include "../../ICollision/ICollisionContact/IContactManager.h"


namespace IEngine
{

// Constructor
/**
 * @param transform The transform of the body
 * @param world The physics world where the body is created
 * @param id ID of the body
 */
ICollisionBody::ICollisionBody(const Transform &transform, IContactManager* collide_meneger , bodyindex id)
    : IBody(id),
      mType(DYNAMIC),
      mTransform(transform),
      mProxyCollisionShapes(nullptr),
      mNbCollisionShapes(0) ,
      mContactManager(collide_meneger) ,
      mContactManifoldsList(nullptr)
{

}

// Destructor
ICollisionBody::~ICollisionBody()
{
    assert(mContactManifoldsList == NULL);
    // Remove all the proxy collision shapes of the body
    RemoveAllCollisionShapes();
}



// Add a collision shape to the body. Note that you can share a collision
// shape between several bodies using the same collision shape instance to
// when you add the shape to the different bodies. Do not forget to delete
// the collision shape you have created at the end of your program.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *        local-space of the collision shape into the local-space of the body
 * @return A pointer to the proxy shape that has been created to link the body to
 *         the new collision shape you have added.
 */
IProxyShape* ICollisionBody::AddCollisionShape(ICollisionShape* collisionShape, scalar massa , const Transform &transform)
{
    // Create a new proxy collision shape to attach the collision shape to the body
    IProxyShape* proxyShape = new IProxyShape( this, collisionShape, transform , massa);

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
    mNbCollisionShapes++;


    /**********************************/
    // Compute the world-space AABB of the new collision shape
    IAABBox3D aabb;
    collisionShape->ComputeAABB(aabb, mTransform * transform);

    // Notify the collision detection about this new collision shape
    mContactManager->AddProxyCollisionShape(proxyShape, aabb);
    /*********************************/

    // Return a pointer to the collision shape
    return proxyShape;
}



// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void ICollisionBody::RemoveCollisionShape(const IProxyShape* proxyShape)
{
  IProxyShape* current = mProxyCollisionShapes;

  // If the the first proxy shape is the one to remove
  if (current == proxyShape)
  {
      mProxyCollisionShapes = current->mNext;

      if (mIsActive)
      {
          mContactManager->RemoveProxyCollisionShape(current);
      }

      //current->~IProxyShape();
      delete current;
      current = nullptr;

      mNbCollisionShapes--;
      return;
  }

    // Look for the proxy shape that contains the collision shape in parameter
    while(current->mNext != nullptr)
    {
      // If we have found the collision shape to remove
      if (current->mNext == proxyShape)
      {
          // Remove the proxy collision shape
          IProxyShape* elementToRemove = current->mNext;
          current->mNext = elementToRemove->mNext;

          if (mIsActive)
          {
             mContactManager->RemoveProxyCollisionShape(elementToRemove);
          }

          //elementToRemove->~IProxyShape();
          delete elementToRemove;
          elementToRemove = nullptr;

          mNbCollisionShapes--;
          return;
      }

      // Get the next element in the list
      current = current->mNext;
    }
}


// Remove all the collision shapes
void ICollisionBody::RemoveAllCollisionShapes()
{
        IProxyShape* current = mProxyCollisionShapes;

        // Look for the proxy shape that contains the collision shape in parameter
        while(current != nullptr)
        {
            // Remove the proxy collision shape
            IProxyShape* nextElement = current->mNext;

            if (mIsActive)
            {
               mContactManager->RemoveProxyCollisionShape(current);
            }

            //current->~IProxyShape();
            delete current->mCollisionShape;
            delete current;
            current = nullptr;
            // Get the next element in the list
            current = nextElement;

        }

        mProxyCollisionShapes = nullptr;
}


// Reset the contact manifold lists
void ICollisionBody::ResetContactManifoldsList()
{
    // Delete the linked list of contact manifolds of that body
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != nullptr)
    {
        // Next element
        ContactManifoldListElement* nextElement = currentElement->GetNext();

        // Delete the current element
        delete currentElement;
        currentElement = nullptr;
        // Next the push current element
        currentElement = nextElement;
    }
    mContactManifoldsList = nullptr;
}



// Update the broad-phase state for this body (because it has moved for instance)
void ICollisionBody::UpdateBroadPhaseState() const
{
    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Update the proxy
        UpdateProxyShapeInBroadPhase( shape , Vector3(0,0,0) );
    }
}



// Update the broad-phase state of a proxy collision shape of the body
void ICollisionBody::UpdateProxyShapeInBroadPhase(IProxyShape* proxyShape, const Vector3 &displacement, bool forceReinsert) const
{
    // Recompute the world-space AABB of the collision shape
    IAABBox3D aabb;
    proxyShape->GetCollisionShape()->ComputeAABB(aabb, mTransform * proxyShape->GetLocalToBodyTransform() );

    // Update the broad-phase state for the proxy collision shape
    mContactManager->UpdateProxyCollisionShape(proxyShape, aabb , displacement , forceReinsert);

}



// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void ICollisionBody::AskForBroadPhaseCollisionCheck() const
{
    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
       mContactManager->AskForBroadPhaseCollisionCheck(shape);
    }
}



// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void ICollisionBody::SetIsActive(bool isActive)
{

    // If the state does not change
    if (mIsActive == isActive) return;

    IBody::setIsActive(isActive);

    // If we have to activate the body
    if (isActive)
    {

        // For each proxy shape of the body
        for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
        {

            // Compute the world-space AABB of the new collision shape
            IAABBox3D aabb;
            shape->GetCollisionShape()->ComputeAABB(aabb, mTransform * shape->mLocalToBodyTransform );

            // Add the proxy shape to the collision detection
            mContactManager->AddProxyCollisionShape(shape, aabb);
        }
    }
    else
    {  // If we have to deactivate the body

        // For each proxy shape of the body
        for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
        {
            // Remove the proxy shape from the collision detection
            mContactManager->RemoveProxyCollisionShape(shape);
        }

        // Reset the contact manifold list of the body
        ResetContactManifoldsList();
    }
}



//// Reset the mIsAlreadyInIsland variable of the body and contact manifolds.
///// This method also returns the number of contact manifolds of the body.
int ICollisionBody::ResetIsAlreadyInIslandAndCountManifolds()
{

    mIsAlreadyInIsland = false;

    // Reset the mIsAlreadyInIsland variable of the contact manifolds for
    // this body
    int nbManifolds = 0;
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != nullptr)
    {
        //currentElement->getPointer()->mIsAlreadyInIsland = false;
        currentElement = currentElement->GetNext();
        nbManifolds++;
    }
    return nbManifolds;
}


// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool ICollisionBody::TestPointInside(const Vector3& worldPoint) const
{
    // For each collision shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Test if the point is inside the collision shape
        if (shape->TestPointInside(worldPoint)) return true;
    }

    return false;
}



// Raycast method with feedback information
/// The method returns the closest hit among all the collision shapes of the body
/**
* @param ray The ray used to raycast agains the body
* @param[out] raycastInfo Structure that contains the result of the raycasting
*                         (valid only if the method returned true)
* @return True if the ray hit the body and false otherwise
*/
bool ICollisionBody::Raycast(const IRay& ray, IRaycastInfo& raycastInfo)
{
    // If the body is not active, it cannot be hit by rays
    if (!mIsActive) return false;

    bool isHit = false;
    IRay rayTemp(ray);


    // For each collision shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != nullptr; shape = shape->mNext)
    {
        // Test if the ray hits the collision shape
        if (shape->Raycast(rayTemp, raycastInfo))
        {
            rayTemp.maxFraction = raycastInfo.hitFraction;
            isHit = true;
        }
    }

    return isHit;
}




// Compute and return the AABB of the body by merging all proxy shapes AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
IAABBox3D ICollisionBody::GetAABB() const
{
    IAABBox3D bodyAABB;

    if (mProxyCollisionShapes == nullptr) return bodyAABB;

    mProxyCollisionShapes->GetCollisionShape()->ComputeAABB(bodyAABB, mTransform * mProxyCollisionShapes->GetLocalToBodyTransform());

    // For each proxy shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes->mNext; shape != nullptr; shape = shape->mNext)
    {
        // Compute the world-space AABB of the collision shape
        IAABBox3D aabb;
        shape->GetCollisionShape()->ComputeAABB(aabb, mTransform * shape->GetLocalToBodyTransform());

        // Merge the proxy shape AABB with the current body AABB
        bodyAABB.MergeWithAABB(aabb);
    }

    return bodyAABB;
}


}
