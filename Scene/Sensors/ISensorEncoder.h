#ifndef ISENSORENCODER_H
#define ISENSORENCODER_H

#include "Engine/engine.hpp"
#include "Engine/imaths.hpp"

using namespace IEngine;
using namespace IMath;


class ISensorEncoder
{

public:

    ISensorEncoder(IHingeJoint *_HingeJoint);

    void Update(float _dt)
    {
        auto transform = m_HingeJoint->GetBody2()->GetTransform();
        auto velocity = transform.GetBasis() * m_HingeJoint->GetBody2()->GetAngularVelocity();
        auto axis = transform.GetBasis() * Vector3::Z;
        vel = (velocity * _dt).Dot(axis);
        pos += vel;
        pos = round(pos*10)/10;
    }


    scalar getPos() const;
    scalar getVel() const;

private:

    IHingeJoint *m_HingeJoint;
    scalar pos;
    scalar vel;

};

#endif // ISENSORENCODER_H
