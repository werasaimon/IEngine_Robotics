#ifndef ICONTACTPOINT_H
#define ICONTACTPOINT_H

#include "../../../imaths.hpp"


namespace IEngine
{


// Structure ContactPointInfo
/**
 * This structure contains informations about a collision contact
 * computed during the narrow-phase collision detection. Those
 * informations are used to compute the contact set for a contact
 * between two bodies.
 */
struct IContactPointInfo
{

    private:

        //-------------------- Methods --------------------//

    public:


        //-------------------- Attributes -----------------//

        /// Normalized normal vector of the collision contact in world space
        Vector3  normal;

        /// Penetration depth of the contact
        scalar   penetration;

        /// Contact point of body 1 in local space of body 1
        Vector3  localPoint1;

        /// Contact point of body 2 in local space of body 2
        Vector3  localPoint2;

        //-------------------- Methods --------------------//

        IContactPointInfo(void){}

        /// Constructor
        IContactPointInfo(const Vector3& normal, scalar penetration,
                           const Vector3& localPoint1,
                           const Vector3& localPoint2)
            : normal(normal),
              penetration(penetration),
              localPoint1(localPoint1),
              localPoint2(localPoint2)
        {

        }
};


// Class ContactPoint
/**
 * This class represents a collision contact point between two
 * bodies in the physics engine.
 */
class IContactPoint
{

    private :

        // -------------------- Attributes -------------------- //

        /// Normalized normal vector of the contact (from body1 toward body2) in world space
        Vector3 mNormal;

        /// Contact point on body 1 in local space of body 1
        Vector3 mLocalPointOnBody1;

        /// Contact point on body 2 in local space of body 2
        Vector3 mLocalPointOnBody2;

        /// Contact point on body 1 in world space
        Vector3 mWorldPointOnBody1;

        /// Contact point on body 2 in world space
        Vector3 mWorldPointOnBody2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Penetration depth
        scalar  mPenetration;

        /// Two orthogonal vectors that span the tangential friction plane
        Vector3 mFrictionVectors[2];

        /// Cached penetration impulse
        scalar  mAccumulatedPenetrationImpulse;

        /// Cached first friction impulse
        scalar  mAccumulatedFrictionImpulse1;

        /// Cached second friction impulse
        scalar  mAccumulatedFrictionImpulse2;

        /// Cached rolling resistance impulse
        Vector3 mAccumulatedRollingResistanceImpulse;



        //-------------------- Methods --------------------//

//        // Private copy-constructor
//        IContactPoint(const IContactPoint& contact);

//        // Private assignment operator
//        IContactPoint& operator=(const IContactPoint& contact);

    public :



        // -------------------- Methods -------------------- //

         /// Constructor
         IContactPoint(){}

        /// Constructor
         IContactPoint(const IContactPointInfo &contactInfo);

        /// Destructor
        ~IContactPoint();


         void SetContactInfo( const IContactPointInfo &contactInfo )
         {
              mNormal            = contactInfo.normal;
              mPenetration       = contactInfo.penetration;
              mLocalPointOnBody1 = contactInfo.localPoint1;
              mLocalPointOnBody2 = contactInfo.localPoint2;
              mWorldPointOnBody1 = contactInfo.localPoint1;
              mWorldPointOnBody2 = contactInfo.localPoint2;
         }


         /// Set the penetration depth of the contact
         void SetPenetration(scalar penetration);

        /// Return the penetration depth of the contact
        scalar GetPenetration() const;


        /// Return the normal vector of the contact
        Vector3 GetNormal() const;

        /// Return the contact local point on body 1
        Vector3 GetLocalPointOnBody1() const;

        /// Return the contact local point on body 2
        Vector3 GetLocalPointOnBody2() const;

        /// Return the contact world point on body 1
        Vector3 GetWorldPointOnBody1() const;

        /// Return the contact world point on body 2
        Vector3 GetWorldPointOnBody2() const;


        /// Return the cached penetration impulse
        scalar GetAccumulatedPenetrationImpulse() const;

        /// Return the cached first friction impulse
        scalar GetAccumulatedFrictionImpulse1() const;

        /// Return the cached second friction impulse
        scalar GetAccumulatedFrictionImpulse2() const;

        /// Return the cached rolling resistance impulse
        Vector3 GetAccumulatedRollingResistanceImpulse() const;


        /// Set the cached penetration impulse
        void SetAccumulatedPenetrationImpulse(scalar impulse);

        /// Set the first cached friction impulse
        void SetAccumulatedFrictionImpulse1(scalar impulse);

        /// Set the second cached friction impulse
        void SetAccumulatedFrictionImpulse2(scalar impulse);

        /// Set the cached rolling resistance impulse
        void SetAccumulatedRollingResistanceImpulse(const Vector3& impulse);



        /// Set the contact local point on body 1
        void SetLocalPointOnBody1(const Vector3& localPoint);

        /// Set the contact local point on body 2
        void SetLocalPointOnBody2(const Vector3& localPoint);

        /// Set the contact world point on body 1
        void SetWorldPointOnBody1(const Vector3& worldPoint);

        /// Set the contact world point on body 2
        void SetWorldPointOnBody2(const Vector3& worldPoint);





        /// Return true if the contact is a resting contact
        bool GetIsRestingContact() const;

        /// Set the mIsRestingContact variable
        void SetIsRestingContact(bool isRestingContact);

        /// Get the first friction vector
        Vector3 GetFrictionVector1() const;

        /// Set the first friction vector
        void SetFrictionVector1(const Vector3& frictionVector1);

        /// Get the second friction vector
        Vector3 GetFrictionVector2() const;

        /// Set the second friction vector
        void SetFrictionVector2(const Vector3& frictionVector2);


        /// Return the number of bytes used by the contact point
        size_t GetSizeInBytes() const;


        //-------------------- Friendships --------------------//


};



// Return the normal vector of the contact
SIMD_INLINE Vector3 IContactPoint::GetNormal() const
{
    return mNormal;
}

// Set the penetration depth of the contact
SIMD_INLINE void IContactPoint::SetPenetration(scalar penetration)
{
    this->mPenetration = penetration;
}


// Return the contact point on body 1
SIMD_INLINE Vector3 IContactPoint::GetLocalPointOnBody1() const
{
    return mLocalPointOnBody1;
}

// Return the contact point on body 2
SIMD_INLINE Vector3 IContactPoint::GetLocalPointOnBody2() const
{
    return mLocalPointOnBody2;
}

// Return the contact world point on body 1
SIMD_INLINE Vector3 IContactPoint::GetWorldPointOnBody1() const
{
    return mWorldPointOnBody1;
}

// Return the contact world point on body 2
SIMD_INLINE Vector3 IContactPoint::GetWorldPointOnBody2() const
{
    return mWorldPointOnBody2;
}




// Return the cached penetration impulse
SIMD_INLINE scalar IContactPoint::GetAccumulatedPenetrationImpulse() const
{
    return mAccumulatedPenetrationImpulse;
}

// Return the cached first friction impulse
SIMD_INLINE scalar IContactPoint::GetAccumulatedFrictionImpulse1() const
{
    return mAccumulatedFrictionImpulse1;
}

// Return the cached second friction impulse
SIMD_INLINE scalar IContactPoint::GetAccumulatedFrictionImpulse2() const
{
    return mAccumulatedFrictionImpulse2;
}

// Return the cached rolling resistance impulse
SIMD_INLINE Vector3 IContactPoint::GetAccumulatedRollingResistanceImpulse() const
{
    return mAccumulatedRollingResistanceImpulse;
}



// Set the cached penetration impulse
SIMD_INLINE void IContactPoint::SetAccumulatedPenetrationImpulse(scalar impulse)
{
    mAccumulatedPenetrationImpulse = impulse;
}

// Set the first cached friction impulse
SIMD_INLINE void IContactPoint::SetAccumulatedFrictionImpulse1(scalar impulse)
{
    mAccumulatedFrictionImpulse1 = impulse;
}

// Set the second cached friction impulse
SIMD_INLINE void IContactPoint::SetAccumulatedFrictionImpulse2(scalar impulse)
{
    mAccumulatedFrictionImpulse2 = impulse;
}

// Set the cached rolling resistance impulse
SIMD_INLINE void IContactPoint::SetAccumulatedRollingResistanceImpulse(const Vector3& impulse)
{
    mAccumulatedRollingResistanceImpulse = impulse;
}


// Set the contact local point on body 1
SIMD_INLINE void IContactPoint::SetLocalPointOnBody1(const Vector3 &localPoint)
{
   mLocalPointOnBody1 = localPoint;
}

// Set the contact local point on body 2
SIMD_INLINE void IContactPoint::SetLocalPointOnBody2(const Vector3 &localPoint)
{
   mLocalPointOnBody2 = localPoint;
}


// Set the contact world point on body 1
SIMD_INLINE void IContactPoint::SetWorldPointOnBody1(const Vector3& worldPoint)
{
    mWorldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
SIMD_INLINE void IContactPoint::SetWorldPointOnBody2(const Vector3& worldPoint)
{
    mWorldPointOnBody2 = worldPoint;
}

// Return true if the contact is a resting contact
SIMD_INLINE bool IContactPoint::GetIsRestingContact() const
{
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
SIMD_INLINE void IContactPoint::SetIsRestingContact(bool isRestingContact)
{
    mIsRestingContact = isRestingContact;
}

// Get the first friction vector
SIMD_INLINE Vector3 IContactPoint::GetFrictionVector1() const
{
    return mFrictionVectors[0];
}

// Set the first friction vector
SIMD_INLINE void IContactPoint::SetFrictionVector1(const Vector3& frictionVector1)
{
    mFrictionVectors[0] = frictionVector1;
}

// Get the second friction vector
SIMD_INLINE Vector3 IContactPoint::GetFrictionVector2() const
{
    return mFrictionVectors[1];
}

// Set the second friction vector
SIMD_INLINE void IContactPoint::SetFrictionVector2(const Vector3& frictionVector2)
{
    mFrictionVectors[1] = frictionVector2;
}

// Return the penetration depth of the contact
SIMD_INLINE scalar IContactPoint::GetPenetration() const
{
    return mPenetration;
}

// Return the number of bytes used by the contact point
SIMD_INLINE size_t IContactPoint::GetSizeInBytes() const
{
    return sizeof(IContactPoint);
}

}

#endif // ICONTACTPOINT_H
