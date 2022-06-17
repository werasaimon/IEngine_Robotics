#include "EncoderSensor.h"

EncoderSensor::EncoderSensor(IHingeJoint *_HingeJoint)
    : m_HingeJoint(_HingeJoint)
{

}

scalar EncoderSensor::getPos() const
{
    return pos;
}

scalar EncoderSensor::getVel() const
{
    return vel;
}
