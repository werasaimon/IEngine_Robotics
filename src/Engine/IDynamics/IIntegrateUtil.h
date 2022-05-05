#ifndef IINTEGRATEUTIL_H
#define IINTEGRATEUTIL_H

#include "../imaths.hpp"

namespace IEngine
{


    class IIntegrateUtil
    {
        public:

            IIntegrateUtil();

            static  Transform IntegrateTransform( const Transform& curTrans , const Vector3& linvel, const Vector3& angvel, scalar timeStep);

            static void Damping( Vector3& Velocity , const scalar& min_damping , const scalar& damping );
    };
}





#endif // IINTEGRATEUTIL_H
