#include "IComponentMesh.h"
#include "../Scene/OpenGL/OpenGLRender.h"




IComponentMesh::IComponentMesh(IMesh* _modelmesh)
    : IComponentAbstract(TYPE_COMPONENT::MESH_MODEL),
      mModelmesh(_modelmesh)
{
    if(mModelmesh)
    {
        mAxisAlignedBoundingBox = mModelmesh->GetAABBLocal();

        mModelmesh->CalculateNormals();
        mModelmesh->CalculateTangets();

        mModelmesh->EnableVertexColors(Vector3(0.01));
    }
}

IComponentMesh::~IComponentMesh()
{
    if(mModelmesh)
    {
        delete mModelmesh;
        mModelmesh=nullptr;
    }
}

void IComponentMesh::Setup()
{

}

void IComponentMesh::Updatee()
{

}

IMesh *IComponentMesh::Modelmesh() const
{
    return mModelmesh;
}

void IComponentMesh::SetModelmesh(IMesh *modelmesh)
{
    mModelmesh = modelmesh;
}


IMaterial IComponentMesh::Material() const
{
    return mMaterial;
}

void IComponentMesh::SetMaterial(const IMaterial &material)
{
    mMaterial = material;
}

