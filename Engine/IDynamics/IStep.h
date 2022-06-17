#ifndef ISTEP2_H
#define ISTEP2_H


#include "../imaths.hpp"

namespace IEngine
{
    struct IMovement
    {
        Vector3    x;
        Quaternion q;
    };


    struct IVelocity
    {
        Vector3 v;
        Vector3 w;
    };
}

#endif // ISTEP2_H
