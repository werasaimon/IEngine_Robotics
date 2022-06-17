#ifndef ICONTACTMANIFOLDSET_H
#define ICONTACTMANIFOLDSET_H

#include "IContactManifold.h"


namespace IEngine
{

// Class ContactManifoldSet
/**
 * This class represents a set of one or several contact manifolds. Typically a
 * convex/convex collision will have a set with a single manifold and a convex-concave
 * collision can have more than one manifolds. Note that a contact manifold can
 * contains several contact points.
 */
class IContactManifoldSet
{

    private:

        // -------------------- Attributes -------------------- //

        /// Maximum number of contact manifolds in the set
        i32 mNbMaxManifolds;

        /// Current number of contact manifolds in the set
        i32 mNbManifolds;

        /// Pointer to the first proxy shape of the contact
        IProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        IProxyShape* mShape2;

        /// Contact manifolds of the set
        IContactManifold* mManifolds[MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET];

        // -------------------- Methods -------------------- //

        /// Create a new contact manifold and add it to the set
        void CreateManifold(short normalDirectionId);

        /// Remove a contact manifold from the set
        void RemoveManifold(i32 index);

        // Return the index of the contact manifold with a similar average normal.
        i32 SelectManifoldWithSimilarNormal(short int normalDirectionId) const;

        // Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
        // Each face of the cube is divided into 4x4 buckets. This method maps the
        // normal vector into of the of the bucket and returns a unique Id for the bucket
        short int ComputeCubemapNormalId(const Vector3& normal) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IContactManifoldSet(IProxyShape* shape1, IProxyShape* shape2, i32 nbMaxManifolds);

        /// Destructor
        ~IContactManifoldSet();



        /// Return the first proxy shape
        IProxyShape* GetShape1() const;

        /// Return the second proxy shape
        IProxyShape* GetShape2() const;

        /// Add a contact point to the manifold set
        void AddContactPoint(IContactPoint* contact);

        /// Update the contact manifolds
        void Update();

        /// Delete the contact manifold.
        void Update_delete_not_uset_contact();

        /// Clear the contact manifold set
        void Clear();

        /// Return the number of manifolds in the set
        i32 GetNbContactManifolds() const;

        /// Return a given contact manifold
        IContactManifold* GetContactManifold(i32 index) const;

        /// Return the total number of contact points in the set of manifolds
        int GetTotalNbContactPoints() const;

        // -------------------- Friendships -------------------- //

        friend class ICollisionManager;
};



// Return the first proxy shape
SIMD_INLINE IProxyShape* IContactManifoldSet::GetShape1() const
{
    return mShape1;
}

// Return the second proxy shape
SIMD_INLINE IProxyShape* IContactManifoldSet::GetShape2() const
{
    return mShape2;
}

// Return the number of manifolds in the set
SIMD_INLINE i32 IContactManifoldSet::GetNbContactManifolds() const
{
    return mNbManifolds;
}


// Return a given contact manifold
SIMD_INLINE IContactManifold* IContactManifoldSet::GetContactManifold(i32 index) const
{
    assert(index >= 0 && index < mNbManifolds);
    return mManifolds[index];
}


}

#endif // ICONTACTMANIFOLDSET_H
