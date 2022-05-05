#ifndef ICOMPONENTMESH_H
#define ICOMPONENTMESH_H

#include "IComponentAbstract.h"

class IComponentMesh : public IComponentAbstract
{

private:

   IMaterial mMaterial;
   IMesh *mModelmesh;

public:

    IComponentMesh(IMesh* _modelmesh = nullptr);
   ~IComponentMesh();

    void Setup() override final;
    void Updatee() override final;

   // virtual bool IntersectionRaycast(const IRay &ray, IRaycast *raycast=nullptr) const;


    IMesh *Modelmesh() const;
    void SetModelmesh(IMesh *modelmesh);


    IMaterial Material() const;
    void SetMaterial(const IMaterial &material);
};

#endif // ICOMPONENTMESH_H
