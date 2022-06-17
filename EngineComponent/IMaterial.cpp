#include "IMaterial.h"

#include <QFile>
#include <QDebug>




IMaterial::IMaterial()
    :  m_DiffuseColor(1.0f, 1.0f, 1.0f),
       m_AmbienceColor(1.0f, 1.0f, 1.0f),
       m_SpecularColor(1.0f, 1.0f, 1.0f),
       m_Shines(90.0f)
{
    m_IsUseDiffuseMap = false;
    m_IsUseNormalMap = false;
}

QString IMaterial::Name()
{
    return m_Name;
}

void IMaterial::setName(const QString &n)
{
    m_Name = n;
}

Vector3 IMaterial::DiffuseColor() const
{
    return m_DiffuseColor;
}

void IMaterial::setDiffuseColor(const Vector3 &dc)
{
    m_DiffuseColor = dc;
}

Vector3 IMaterial::AmbienceColor() const
{
    return m_AmbienceColor;
}

void IMaterial::setAmbienceColor(const Vector3 &ac)
{
    m_AmbienceColor = ac;
}

Vector3 IMaterial::SpecularColor() const
{
    return m_SpecularColor;
}

void IMaterial::setSpecularColor(const Vector3 &sc)
{
    m_SpecularColor = sc;
}

float IMaterial::Shines() const
{
    return m_Shines;
}

void IMaterial::setShines(const float &s)
{
    m_Shines = s;
}

QImage IMaterial::DiffuseMap() const
{
    return m_DiffuseMap;
}

void IMaterial::setDiffuseMap(const QString &filename)
{
    if(!QFile(filename).exists()) {qDebug() << "File not found:" << filename; return; }

    m_DiffuseMap = QImage(filename);
    m_IsUseDiffuseMap = true;

    //if(m_DiffuseMapTexture) delete m_DiffuseMapTexture;

    m_DiffuseMapTexture = new QOpenGLTexture(m_DiffuseMap.mirrored());
    m_DiffuseMapTexture->setMinificationFilter(QOpenGLTexture::Nearest);
    m_DiffuseMapTexture->setMagnificationFilter(QOpenGLTexture::Linear);
    m_DiffuseMapTexture->setWrapMode(QOpenGLTexture::Repeat);
}

void IMaterial::setDiffuseMap(const QImage &image)
{
    m_DiffuseMap = image;
    m_IsUseDiffuseMap = true;
}

bool IMaterial::isUseDiffuseMap() const
{
    return m_IsUseDiffuseMap;
}

QImage IMaterial::NormalMap() const
{
    return m_NormalMap;
}

void IMaterial::setNormalMap(const QString &filename)
{
    if(!QFile(filename).exists()) {qDebug() << "File not found:" << filename; return; }

    m_NormalMap = QImage(filename);
    m_IsUseNormalMap = true;

    //if(m_NormalMapTexture) delete m_NormalMapTexture;

    m_NormalMapTexture = new QOpenGLTexture(m_NormalMap.mirrored());
    m_NormalMapTexture->setMinificationFilter(QOpenGLTexture::Nearest);
    m_NormalMapTexture->setMagnificationFilter(QOpenGLTexture::Linear);
    m_NormalMapTexture->setWrapMode(QOpenGLTexture::Repeat);
}

void IMaterial::setNormalMap(const QImage &image)
{
    m_NormalMap = image;
    m_IsUseNormalMap = true;
}

bool IMaterial::isUseNormalMap() const
{
    return m_IsUseNormalMap;
}

QOpenGLTexture *IMaterial::DiffuseMapTexture() const
{
    return m_DiffuseMapTexture;
}

QOpenGLTexture *IMaterial::NormalMapTexture() const
{
    return m_NormalMapTexture;
}


