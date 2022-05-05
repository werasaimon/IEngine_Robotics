#ifndef ICOLLISIONALGORITHM_H
#define ICOLLISIONALGORITHM_H


#include "../../../imaths.hpp"
#include "ICollisionShapeInfo.h"


namespace IEngine
{

using namespace IMath;


class IOutputCollisionInfo
{

public:

    //-------- Attribute --------//
    scalar   Penetration;
    Vector3 Normal;
    Vector3 Point1;
    Vector3 Point2;

    IOutputCollisionInfo()
    {

    }

    IOutputCollisionInfo( const Vector3& _normal ,  const scalar&  _penetration ,
                          const Vector3& _point1 ,
                          const Vector3& _point2 )
      :Normal(_normal),
       Penetration(_penetration),
       Point1(_point1),
       Point2(_point2)
    {

    }
};




// Class NarrowPhaseAlgorithm
/**
 * This abstract class is the base class for a  narrow-phase collision
 * detection algorithm. The goal of the narrow phase algorithm is to
 * compute information about the contact between two proxy shapes.
 */
class ICollisionAlgorithm
{

protected :

    // -------------------- Attributes -------------------- //

    IOutputCollisionInfo mOutputCollisionInfo;


    // -------------------- Methods -------------------- //

    /// Private copy-constructor
    ICollisionAlgorithm(const ICollisionAlgorithm& algorithm);

    /// Private assignment operator
    ICollisionAlgorithm& operator=(const ICollisionAlgorithm& algorithm);

public :

    // -------------------- Methods -------------------- //

    /// Constructor
    ICollisionAlgorithm();

    /// Destructor
    virtual ~ICollisionAlgorithm();


    /// Compute a contact info if the two bounding volume collide
    virtual bool TestCollision(const ICollisionShapeInfo& shape1Info,
                               const ICollisionShapeInfo& shape2Info) = 0;

    IOutputCollisionInfo GetOutputCollisionInfo() const;

    //---------------------- friendship --------------------------//
    friend class IContactGenerator;


};

}

#endif // ICOLLISIONALGORITHM_H
