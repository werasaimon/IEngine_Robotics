#include "ISensorEncoder.h"

ISensorEncoder::ISensorEncoder(IHingeJoint *_HingeJoint)
    : m_HingeJoint(_HingeJoint)
{

}

scalar ISensorEncoder::getPos() const
{
    return pos;
}

scalar ISensorEncoder::getVel() const
{
    return vel;
}
