#ifndef LIGHT_H
#define LIGHT_H

// Libraries
#include <cassert>
#include "../imaths.hpp"
#include "IQCamera.h"


namespace IEngine
{

class ILight : public AffineTransform
{
public:
    enum struct LightType
    {
        Direct = 0,
        Point = 1,
        Spot = 2
    };

    ILight(const LightType &type = LightType::Direct);

    const Vector3 DiffuseColor() const;
    void SetDiffuseColor(const Vector3 &diffuseColor);

    const Vector3 AmbienceColor() const;
    void SetAmbienceColor(const Vector3 &ambienceColor);

    const Vector3 SpecularColor() const;
    void SetSpecularColor(const Vector3 &specularColor);

    Vector4 ReflectionColor() const;
    void SetReflectionColor(const Vector4 &ReflectionColor);

    const Vector4 Position() const;
    void SetPosition(const Vector4 &position);

    const Vector4 Direction() const;
    void SetDirection(const Vector4 &direction);

    float Cutoff() const;
    void SetCutoff(const float& cutoff);

    float Power() const;
    void SetPower(const float& power);

    ILight::LightType Type() const;
    void SetType(const LightType &type);

    const Matrix4 LightMatrix() const;

    float VolumeShadow() const;
    void SetVolumeShadow(float Near);

    IQCamera CameraShadows() const;

//protected:
    void applyLightMatrix();



private:
    Vector3 mAmbienceColor;
    Vector3 mDiffuseColor;
    Vector3 mSpecularColor;
    Vector4 mReflectionColor; // цвет блика
    Vector4 mPosition;
    Vector4 mDirection;

    float   mCutoff;             // сектор пучка света
    float   mPower;
    float   mVolumeShadow;

    IQCamera mCameraShadows;

    LightType mType;
};
}

#endif
