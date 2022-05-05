// Libraries
#include "ILight.h"


namespace IEngine
{

ILight::ILight(const LightType &type):
    mAmbienceColor(1.0f, 1.0f, 1.0f),
    mDiffuseColor(1.0f, 1.0f, 1.0f),
    mSpecularColor(1.0f, 1.0f, 1.0f),
    mReflectionColor(1.0f, 1.0f, 1.0f, 1.0f),
    mPosition(0.0f, 0.0f, 0.0f, 1.0f),
    mDirection(0.0f, 0.0f, -1.0f, 0.0f),
    mCutoff(static_cast<float>(3.14 * 2.f)),
    mPower(1.0f),
    mVolumeShadow(120.f),
    mType(type)
{
    applyLightMatrix();
}


const Vector3 ILight::DiffuseColor() const
{
    return mDiffuseColor;
}

void ILight::SetDiffuseColor(const Vector3 &diffuseColor)
{
    mDiffuseColor = diffuseColor;
}

const Vector3 ILight::AmbienceColor() const
{
    return mAmbienceColor;
}

void ILight::SetAmbienceColor(const Vector3 &ambienceColor)
{
    mAmbienceColor = ambienceColor;
}

const Vector3 ILight::SpecularColor() const
{
    return mSpecularColor;
}

void ILight::SetSpecularColor(const Vector3 &specularColor)
{
    mSpecularColor = specularColor;
}

const Vector4 ILight::Position() const
{
    return Vector4(mTransformMatrix.GetTranslation(),1.0);
}

void ILight::SetPosition(const Vector4 &position)
{
    mTransformMatrix.SetTranslation(position);
    applyLightMatrix();
}

const Vector4 ILight::Direction() const
{
    // transpose rotate direction
    return mDirection * Matrix4(mTransformMatrix.GetRotMatrix());
}

void ILight::SetDirection(const Vector4 &direction)
{
    mDirection = direction.Normalized();
    applyLightMatrix();
}

float ILight::Cutoff() const
{
    return mCutoff;
}

void ILight::SetCutoff(const float& cutoff)
{
    mCutoff = cutoff;
}

ILight::LightType ILight::Type() const
{
    return mType;
}

void ILight::SetType(const LightType &type)
{
    mType = type;
}

const Matrix4 ILight::LightMatrix() const
{
    return mCameraShadows.ViewMatrix();
}

void ILight::applyLightMatrix()
{
    if(mType == LightType::Direct)
    {
       mCameraShadows.SetOrthographic(true);
       mCameraShadows.SetAngle(30.f);
       mCameraShadows.SetAspect(1.0);
       mCameraShadows.SetNear(-mVolumeShadow);
       mCameraShadows.SetFar(mVolumeShadow);
    }
    else
    {
       mCameraShadows.SetOrthographic(false);
       mCameraShadows.SetAngle(90.f);
       mCameraShadows.SetAspect(1.0);
       mCameraShadows.SetNear(1.0);
       mCameraShadows.SetFar(mVolumeShadow);
    }

    Vector3 pos = mTransformMatrix.GetTranslation();
    Vector3 dir = Direction().GetXYZ();
    mCameraShadows.LookAt( pos, pos + (dir).Normalized(), Vector3(dir.x,dir.z,-dir.y) );
}

IQCamera ILight::CameraShadows() const
{
    return mCameraShadows;
}

float ILight::VolumeShadow() const
{
    return mVolumeShadow;
}

void ILight::SetVolumeShadow(float Near)
{
    mVolumeShadow = Near;
}

Vector4 ILight::ReflectionColor() const
{
    return mReflectionColor;
}

void ILight::SetReflectionColor(const Vector4 &ReflectionColor)
{
    mReflectionColor = ReflectionColor;
}

float ILight::Power() const
{
    return mPower;
}

void ILight::SetPower(const float& power)
{
    mPower = power;
}

}
