#include "ICollisionAlgorithmGjkEpa.h"


 #include "../../../IAlgorithm/GJK-EPA/IGjkEpa.cpp"

namespace IEngine
{


ICollisionAlgorithmGjkEpa::ICollisionAlgorithmGjkEpa()
{
    // TODO Auto-generated constructor stub

}

ICollisionAlgorithmGjkEpa::~ICollisionAlgorithmGjkEpa()
{
    // TODO Auto-generated destructor stub
}

bool ICollisionAlgorithmGjkEpa::TestCollision(const ICollisionShapeInfo &shape1Info,
                                              const ICollisionShapeInfo &shape2Info)
{

        IGjkCollisionDescription desp;
        desp.mFirstDir = mOutputCollisionInfo.Normal;

        return   IGjkEpa::ComputeGjkEpaPenetrationDepth(shape1Info ,
                                                        shape2Info ,
                                                        desp ,
                                                        mOutputCollisionInfo.Normal,
                                                        mOutputCollisionInfo.Point1,
                                                        mOutputCollisionInfo.Point2);

}

}
