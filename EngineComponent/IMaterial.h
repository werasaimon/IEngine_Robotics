#ifndef IMATERIAL_H
#define IMATERIAL_H


#include <QImage>
#include <QString>
#include <QOpenGLTexture>
#include "Engine/engine.hpp"


using namespace IEngine;

class IMaterial
{
public:
    IMaterial();

    QString Name();
    void setName(const QString &n);

    Vector3 DiffuseColor() const;
    void setDiffuseColor(const Vector3 &dc);

    Vector3 AmbienceColor() const;
    void setAmbienceColor(const Vector3 &ac);

    Vector3 SpecularColor() const;
    void setSpecularColor(const Vector3 &sc);

    float Shines() const;
    void setShines(const float &s);

    QImage DiffuseMap() const;
    void setDiffuseMap(const QString &filename);
    void setDiffuseMap(const QImage &image);
    bool isUseDiffuseMap() const;

    QImage NormalMap() const;
    void setNormalMap(const QString &filename);
    void setNormalMap(const QImage &image);
    bool isUseNormalMap() const;

    QOpenGLTexture *DiffuseMapTexture() const;
    QOpenGLTexture *NormalMapTexture() const;

private:
    QString m_Name;
    Vector3 m_DiffuseColor;
    Vector3 m_AmbienceColor;
    Vector3 m_SpecularColor;
    float m_Shines;

    QImage m_DiffuseMap;
    QImage m_NormalMap;
    bool m_IsUseDiffuseMap;
    bool m_IsUseNormalMap;


    QOpenGLTexture* m_DiffuseMapTexture;
    QOpenGLTexture* m_NormalMapTexture;
};


#endif // IMATERIAL_H
