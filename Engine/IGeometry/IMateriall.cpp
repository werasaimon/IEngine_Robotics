#include "IMateriall.h"

namespace IEngine
{

IMateriall::IMateriall()
{
}

std::string IMateriall::Name()
{
    return m_Name;
}

void IMateriall::SetName(const std::string &n)
{
    m_Name = n;
}

Vector3 IMateriall::DiffuseColor() const
{
    return m_DiffuseColor;
}

void IMateriall::SetDiffuseColor(const Vector3 &dc)
{
    m_DiffuseColor = dc;
}

Vector3 IMateriall::AmbienceColor() const
{
    return m_AmbienceColor;
}

void IMateriall::SetAmbienceColor(const Vector3 &ac)
{
    m_AmbienceColor = ac;
}

Vector3 IMateriall::SpecularColor() const
{
    return m_SpecularColor;
}

void IMateriall::SetSpecularColor(const Vector3 &sc)
{
    m_SpecularColor = sc;
}

float IMateriall::Shines() const
{
    return m_Shines;
}

void IMateriall::SetShines(const float &s)
{
    m_Shines = s;
}

}
