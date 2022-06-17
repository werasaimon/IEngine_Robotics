#ifndef IMATERIAL_H
#define IMATERIALL_H


// Libraries
#include <cassert>
#include "../imaths.hpp"

namespace IEngine
{

using namespace IMath;
using namespace IEngine;

class IMateriall
{
public:
    IMateriall();

    std::string Name();
    void SetName(const std::string &n);

    Vector3 DiffuseColor() const;
    void SetDiffuseColor(const Vector3 &dc);

    Vector3 AmbienceColor() const;
    void SetAmbienceColor(const Vector3 &ac);

    Vector3 SpecularColor() const;
    void SetSpecularColor(const Vector3 &sc);

    float Shines() const;
    void SetShines(const float &s);


private:
    std::string m_Name;
    Vector3     m_DiffuseColor;
    Vector3     m_AmbienceColor;
    Vector3     m_SpecularColor;
    float       m_Shines;

};

}



#endif // IMATERIALL_H
