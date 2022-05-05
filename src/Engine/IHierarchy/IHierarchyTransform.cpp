#include "IHierarchyTransform.h"



namespace IEngine
{

IHierarchyTransform::IHierarchyTransform(unsigned int id)
 :IHierarchyNode(id)
{

}

Matrix4 IEngine::IHierarchyTransform::GetTransformMatrixHierarchy() const
{
    if(_parent != 0 )
    {
        return static_cast<IHierarchyTransform*>(_parent)->GetTransformMatrixHierarchy() * mTransformMatrix;
    }
    return mTransformMatrix;
}

Transform IHierarchyTransform::GetTransformHierarchy() const
{
    if(_parent != nullptr )
    {
        return static_cast<IHierarchyTransform*>(_parent)->GetTransformHierarchy() * GetTransfom();
    }
    return mTransformMatrix;
}

Transform IHierarchyTransform::GetTransfom() const
{
    return Transform(mTransformMatrix);
}

void IHierarchyTransform::SetTransformMatrix(const Matrix4 &m)
{
    mTransformMatrix = m;
}

void IHierarchyTransform::SetTransform(const Transform &m)
{
    mTransformMatrix = m.GetTransformMatrix();
}

}
