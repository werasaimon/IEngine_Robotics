#ifndef ICOMPONENTLIGHT_H
#define ICOMPONENTLIGHT_H

#include "IComponentAbstract.h"
#include <QOpenGLFramebufferObject>


class IComponentLight : public IComponentAbstract
{

private:

    ILight mLight;

    bool  mIsUseSadow;
    QOpenGLFramebufferObject *mShadowFrameBuffer;

public:

    IComponentLight(const ILight::LightType &type = ILight::LightType::Direct);

    void InitAttributTransform();

    void Updatee() override;

    ILight *light();

    ///
    /// \brief ShadowFrameBuffer
    /// \return Frame buffer for shadow camera
    QOpenGLFramebufferObject *ShadowFrameBuffer() const;

    ///
    /// \brief CreateShadowFrameBuffer
    /// \param _fb_width shadow map
    /// \param _fb_height shadow map
    void CreateShadowFrameBuffer(float _fb_width , float _fb_height );

    ///
    /// \brief SetVolumeShadow
    /// \param _volume_shadow volume shadow
    void SetVolumeShadow(float _volume_shadow );

    bool isUseSadow() const;
    void setIsUseSadow(bool isUseSadow);
};

#endif // ICOMPONENTLIGHT_H
