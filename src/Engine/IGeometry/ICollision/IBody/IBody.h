#ifndef IBODY_H
#define IBODY_H


#include "../../../ICommon/ISettings.h"
#include "../../../imaths.hpp"

namespace IEngine
{

using namespace IMath;

typedef long unsigned int luint;
typedef luint bodyindex;
typedef std::pair<bodyindex, bodyindex> bodyindexpair;



// TODO : Make this class abstract
// Class Body
/**
 * This class to represent a body of the physics engine. You should not
 * instantiante this class but instantiate the CollisionBody or RigidBody
 * classes instead.
 */
class IBody
{

    protected :

        // -------------------- Attributes -------------------- //

        /// ID of the body
        bodyindex mID;

        /// True if the body has already been added in an island (for sleeping technique)
        bool mIsAlreadyInIsland;

        /// True if the body is allowed to go to sleep for better efficiency
        bool mIsAllowedToSleep;

        /// True if the body is active.
        /// An inactive body does not participate in collision detection,
        /// is not simulated and will not be hit in a ray casting query.
        /// A body is active by default. If you set this
        /// value to "false", all the proxy shapes of this body will be
        /// removed from the broad-phase. If you set this value to "true",
        /// all the proxy shapes will be added to the broad-phase. A joint
        /// connected to an inactive body will also be inactive.
        bool mIsActive;

        /// True if the body is sleeping (for sleeping technique)
        bool mIsSleeping;

        /// Elapsed time since the body velocity was bellow the sleep velocity
        scalar mSleepTime;

        /// Pointer that can be used to attach user data to the body
        void* mUserData;



        //-------------------- Constructor -------------------- //

        /// Private copy-constructor
        IBody(const IBody& body);

        /// Private assignment operator
        IBody& operator=(const IBody& body);

    public:

        //----------------------- Methods ----------------------//

        /// Constructor
        IBody(bodyindex id);

        /// Destructor
        virtual ~IBody();

        /// Return the ID of the body
        bodyindex GetID() const;

        /// Return whether or not the body is allowed to sleep
        bool IsAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void SetIsAllowedToSleep(bool isAllowedToSleep);

        /// Set the variable to know whether or not the body is sleeping
        virtual void SetIsSleeping(bool isSleeping);

        /// Return whether or not the body is sleeping
        bool IsSleeping() const;

        /// Return true if the body is active
        bool IsActive() const;

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return a pointer to the user data attached to this body
        void* GetUserData() const;

        /// Attach user data to this body
        void SetUserData(void* userData);

        /// Smaller than operator
        bool operator<(const IBody& body2) const;

        /// Larger than operator
        bool operator>(const IBody& body2) const;

        /// Equal operator
        bool operator==(const IBody& body2) const;

        /// Not equal operator
        bool operator!=(const IBody& body2) const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the id of the body
/**
 * @return The ID of the body
 */
SIMD_INLINE bodyindex IBody::GetID() const
{
    return mID;
}

// Return whether or not the IBody is allowed to sleep
/**
 * @return True if the IBody is allowed to sleep and false otherwise
 */
SIMD_INLINE bool IBody::IsAllowedToSleep() const
{
    return mIsAllowedToSleep;
}

// Set whether or not the IBody is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the IBody is allowed to sleep
 */
SIMD_INLINE void IBody::SetIsAllowedToSleep(bool isAllowedToSleep)
{
    mIsAllowedToSleep = isAllowedToSleep;
    if (!mIsAllowedToSleep) SetIsSleeping(false);
}

// Return whether or not the IBody is sleeping
/**
 * @return True if the IBody is currently sleeping and false otherwise
 */
SIMD_INLINE bool IBody::IsSleeping() const
{
    return mIsSleeping;
}

// Return true if the IBody is active
/**
 * @return True if the IBody currently active and false otherwise
 */
SIMD_INLINE bool IBody::IsActive() const
{
    return mIsActive;
}

// Set whether or not the IBody is active
/**
 * @param isActive True if you want to activate the IBody
 */
SIMD_INLINE void IBody::setIsActive(bool isActive)
{
    mIsActive = isActive;
}

// Set the variable to know whether or not the IBody is sleeping
SIMD_INLINE void IBody::SetIsSleeping(bool isSleeping)
{
    if (isSleeping)
    {
        mSleepTime = scalar(0.0);
    }
    else
    {
        if (mIsSleeping)
        {
            mSleepTime = scalar(0.0);
        }
    }

    mIsSleeping = isSleeping;
}

// Return a pointer to the user data attached to this IBody
/**
 * @return A pointer to the user data you have attached to the IBody
 */
SIMD_INLINE void* IBody::GetUserData() const
{
    return mUserData;
}

// Attach user data to this IBody
/**
 * @param userData A pointer to the user data you want to attach to the IBody
 */
SIMD_INLINE void IBody::SetUserData(void* userData)
{
    mUserData = userData;
}

// Smaller than operator
SIMD_INLINE bool IBody::operator<(const IBody& body2) const
{
    return (mID < body2.mID);
}

// Larger than operator
SIMD_INLINE bool IBody::operator>(const IBody& body2) const
{
    return (mID > body2.mID);
}

// Equal operator
SIMD_INLINE bool IBody::operator==(const IBody& body2) const
{
    return (mID == body2.mID);
}

// Not equal operator
SIMD_INLINE bool IBody::operator!=(const IBody& body2) const
{
    return (mID != body2.mID);
}


}

#endif // IBODY_H
