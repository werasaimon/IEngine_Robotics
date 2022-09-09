#ifndef ISENSORLIDAR_H
#define ISENSORLIDAR_H

#include "Engine/engine.hpp"

using namespace IEngine;
using namespace IMath;



class IRaycastCallbackInfo : public IRaycastCallback
{
    private:


       // -------------------- Attributes -------------------- //

       ///! is bind caculate LASER
       bool  mIsBind;

       ///! Fraction distance of the hit point between point1 and point2 of the ray
       ///! The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
       scalar mDistance;

       ///! Maximun distance to object
       scalar mMaximumDistance;

    public:

        //-------------------- Methods --------------------//

        IRaycastCallbackInfo() :
         mIsBind(true),
         mDistance(1),
         mMaximumDistance(1)
        {

        }

        ~IRaycastCallbackInfo()
        {

        }

        void Bind(const scalar& _maximunm_distance)
        {
            mIsBind = true;
            mDistance = _maximunm_distance;
            mMaximumDistance = _maximunm_distance;
        }

        /// This method will be called for each ProxyShape that is hit by the
        /// ray. You cannot make any assumptions about the order of the
        /// calls. You should use the return value to control the continuation
        /// of the ray. The returned value is the next maxFraction value to use.
        /// If you return a fraction of 0.0, it means that the raycast should
        /// terminate. If you return a fraction of 1.0, it indicates that the
        /// ray is not clipped and the ray cast should continue as if no hit
        /// occurred. If you return the fraction in the parameter (hitFraction
        /// value in the IRaycastInfo object), the current ray will be clipped
        /// to this fraction in the next queries. If you return -1.0, it will
        /// ignore this ProxyShape and continue the ray cast.
        /**
         * @param IRaycastInfo Information about the raycast hit
         * @return Value that controls the continuation of the ray after a hit
         */
        scalar notifyRaycastHit(const IRaycastInfo& IRaycastInfo)
        {
            /// Minimal distance [sorting]
            if(mDistance > IRaycastInfo.hitFraction || mIsBind)
            {
                mDistance = (IRaycastInfo.hitFraction <= mMaximumDistance) ? IRaycastInfo.hitFraction : mMaximumDistance;
                mIsBind = false;
            }

            return IRaycastInfo.hitFraction;
        }

        ///
        /// \brief distance
        /// \return Minimal distance Lidar
        ///
        float getDistance() const
        {
            return mDistance;
        }

};

class ISensorLIDAR
{

    private:

        /// ! Dynamic world physics - pointer
        IDynamicsWorld *m_pDynamicsWorld;

        /// ! Direction world ray laser
        Vector3   mDirection;

        /// ! Transform direction and position laser
        Transform mTransform;

        /// > Callback raycasing information
        mutable IRaycastCallbackInfo RaycastingCallback;

    public:

        ISensorLIDAR(IDynamicsWorld *_DynamicsWorld ,
                      const Vector3& _direction = Vector3::Z,
                      const Transform& _transform = Transform::Identity());

        ~ISensorLIDAR();

        ///
        /// \brief WorldDirection Direction Laser
        /// \return Direction Laser
        ///
        Vector3 WorldDirection() const;


        ///
        /// \brief LAngle -
        /// \param _NormalFace - Predepikular to direction ray-laser
        /// \return  Laser head rotation angle
        ///
        scalar LAngleRotation(const Vector3& _NormalFace);


        ///
        /// \brief CalculateDistance calculate distance LIDAR Lidar (/ˈlaɪdɑːr/, also LIDAR, or LiDAR; sometimes LADAR)
        /// is a method for determining ranges (variable distance) by targeting an object
        /// or a surface with a laser and measuring the time for the reflected light to return to the receiver.
        /// It can also be used to make digital 3-D representations of areas on the Earth's surface and
        /// ocean bottom of the intertidal and near coastal zone by varying the wavelength of light.
        /// It has terrestrial, airborne, and mobile applications.
        /// ------------------------------------------------------
        /// \param _maximunm_distance Maximum distance object detection
        /// \param raycastWithCategoryMaskBits mask group
        /// \return distance to object !!!
        ///
        scalar CalculateDistance(const scalar& _maximunm_distance = 10 , unsigned short raycastWithCategoryMaskBits = 0xFFFF) const
        {
            RaycastingCallback.Bind(_maximunm_distance);
            const Vector3 P = mTransform.GetPosition();
            const Vector3 D = mTransform.GetBasis() * mDirection;
            const IRay _ray(P,D,_maximunm_distance);
            m_pDynamicsWorld->raycast(_ray , &RaycastingCallback, raycastWithCategoryMaskBits);
            return RaycastingCallback.getDistance();
        }


        const Transform &transform() const
        {
            return mTransform;
        }

        void setTransform(const Transform &newTransform)
        {
            mTransform = newTransform;
        }
};

#endif // ISENSORLIDAR_H
