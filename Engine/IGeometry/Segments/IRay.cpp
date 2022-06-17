#include "IRay.h"


namespace IEngine
{

IRay &IRay::operator=(const IRay &ray)
{
    if (&ray != this)
    {
        Origin = ray.Origin;
        Direction = ray.Direction;
        maxFraction = ray.maxFraction;
    }
    return *this;
}




}
