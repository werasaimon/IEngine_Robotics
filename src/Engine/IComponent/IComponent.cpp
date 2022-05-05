#include "IComponent.h"


namespace IEngine
{

Matrix4 IComponent::GetTransformMatrixHierarchy() const
{
    assert(mCollid != NULL);
    if(_parent != nullptr )
    {
        return static_cast<IComponent*>(_parent)->GetTransformMatrixHierarchy() * mCollid->GetTransform().GetTransformMatrix();
    }
    return mCollid->GetTransform().GetTransformMatrix();;
}

Transform IComponent::GetTransformHierarchy() const
{
    assert(mCollid != NULL);
    if(_parent != nullptr )
    {
        return static_cast<IComponent*>(_parent)->GetTransformHierarchy() * mCollid->GetTransform();
    }
    return mCollid->GetTransform();
}

void IComponent::SetTransform(const IMatrix4x4<float> &m)
{
    mCollid->SetTransform( m );
}

IProxyShape *IComponent::Add(ICollisionShape *collid_shape, IMesh *mesh, scalar massa)
{
    assert(mCollid != NULL);
    if(mMeshes.find(mesh) == mMeshes.end())
    {
        Transform local_transform = GetTransformMatrixHierarchy().GetInverse() * mesh->GetTransformMatrix();
        auto proxy_shape =  mCollid->AddCollisionShape( collid_shape , massa , local_transform );
        mMeshes.insert( std::make_pair(mesh,proxy_shape) );
        return proxy_shape;
    }
    return NULL;
}

}


