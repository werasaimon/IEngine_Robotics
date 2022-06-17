#ifndef ICOMPONENTABSTRACT_H
#define ICOMPONENTABSTRACT_H

#include <string>
#include "Engine/engine.hpp"

#include <QOpenGLTexture>
#include "IMaterial.h"

#include "../Scene/Shader/Shader.h"

using namespace IMath;
using namespace IEngine;


enum struct TYPE_COMPONENT { DEFAULT , MESH_MODEL , CAMERA , LIGHT };




class IComponentAbstract : public IHierarchyTransform
{

protected:

    TYPE_COMPONENT mType;

    std::string  mName;
    IAABBox3D    mAxisAlignedBoundingBox;


public:


    IComponentAbstract(TYPE_COMPONENT _Type = TYPE_COMPONENT::DEFAULT);
    virtual ~IComponentAbstract();


    virtual void Setup() {}
    virtual void Updatee() {}

    virtual bool IntersectionRaycast(const IRay &ray , IRaycast *raycast = nullptr) const;

    std::string GetName() const;
    void SetName(const std::string &name);


    IAABBox3D GetAxisAlignedBoundingBox() const;
    void SetAxisAlignedBoundingBox(const IAABBox3D &axisAlignedBoundingBox);


    //--------------------------------------//

    //    void InitMaterial(IMaterial *_Material);

    //    QOpenGLTexture *DiffuseMap() const;
    //    void setDiffuseMap(QOpenGLTexture *DiffuseMap);

    //    QOpenGLTexture *NormalMap() const;
    //    void setNormalMap(QOpenGLTexture *NormalMap);

    //    IMaterial *material() const;
    TYPE_COMPONENT type() const;
};

#endif // ICOMPONENTABSTRACT_H
