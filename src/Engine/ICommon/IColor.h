#ifndef ICOLOR_H
#define ICOLOR_H

#include "../imaths.hpp"

namespace IEngine
{

using namespace  IMath;

// Structure Color
// This structure represents a RGBA color.
struct IColor : public IVector4f
{

public:

    // -------------------- Methods -------------------- //

    // Constructor
    IColor() : IVector4f(1,1,1,1) {}

    // Constructor
    IColor(float r, float g, float b, float a) : IVector4f(r,g,b,a) {}

    // Constructor
    IColor(const IColor& color) : IVector4f(color.r,color.g,color.b,color.a) {}

    // Destructor
    ~IColor() {}

    // Return the black color
    static IColor black() { return IColor(0.0f, 0.0f, 0.0f, 1.0f);}

    // Return the white color
    static IColor white() { return IColor(1.0f, 1.0f, 1.0f, 1.0f);}

    // = operator
    IColor& operator=(const IColor& color)
    {
        if (&color != this)
        {
            r = color.r;
            g = color.g;
            b = color.b;
            a = color.a;
        }
        return *this;
    }
};


}

#endif // ICOLOR_H
