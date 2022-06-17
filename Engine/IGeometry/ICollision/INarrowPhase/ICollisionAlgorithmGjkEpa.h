#ifndef ICOLLISIONALGORITHMGJKEPA_H
#define ICOLLISIONALGORITHMGJKEPA_H

#include "ICollisionAlgorithm.h"
#include "../../../IAlgorithm/GJK-EPA/IGjkEpa.h"


namespace IEngine
{

using namespace IAlgorithm;

class ICollisionAlgorithmGjkEpa: public ICollisionAlgorithm
{

        /// Private copy-constructor
        ICollisionAlgorithmGjkEpa(const ICollisionAlgorithmGjkEpa& algorithm);

        /// Private assignment operator
        ICollisionAlgorithmGjkEpa& operator=(const ICollisionAlgorithmGjkEpa& algorithm);


    public:
                 ICollisionAlgorithmGjkEpa();
        virtual ~ICollisionAlgorithmGjkEpa();


        /// Compute a contact info if the two bounding volume collide
        virtual bool TestCollision(const ICollisionShapeInfo &shape1Info,
                                   const ICollisionShapeInfo &shape2Info);


};
}

#endif // ICOLLISIONALGORITHMGJKEPA_H
