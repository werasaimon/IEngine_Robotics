#include "IComponentLight.h"


IComponentLight::IComponentLight(const ILight::LightType &type)
: IComponentAbstract(TYPE_COMPONENT::LIGHT) ,
  mLight(type)
{

}

void IComponentLight::InitAttributTransform()
{
   mLight.applyLightMatrix();
   SetTransformMatrix(mLight.GetTransformMatrix());
}

void IComponentLight::Updatee()
{
    mLight.SetTransformMatrix(GetTransformMatrixHierarchy());
    mLight.applyLightMatrix();
}

ILight *IComponentLight::light()// const
{
    return &mLight;
}

QOpenGLFramebufferObject *IComponentLight::ShadowFrameBuffer() const
{
    return mShadowFrameBuffer;
}

void IComponentLight::CreateShadowFrameBuffer(float _fb_width, float _fb_height )
{
    mShadowFrameBuffer = new QOpenGLFramebufferObject( _fb_width , _fb_height , QOpenGLFramebufferObject::Depth);
}

void IComponentLight::SetVolumeShadow(float _volume_shadow)
{
    mLight.SetVolumeShadow(_volume_shadow);
}


bool IComponentLight::isUseSadow() const
{
    return mIsUseSadow;
}

void IComponentLight::setIsUseSadow(bool isUseSadow)
{
    mIsUseSadow = isUseSadow;
}
