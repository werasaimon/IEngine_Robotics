#include "IComponentAbstract.h"



TYPE_COMPONENT IComponentAbstract::type() const
{
    return mType;
}

IComponentAbstract::IComponentAbstract(TYPE_COMPONENT _Type)
    : mType(_Type),
      mAxisAlignedBoundingBox(-Vector3::IDENTITY,Vector3::IDENTITY)
{

}

IComponentAbstract::~IComponentAbstract()
{

}

bool IComponentAbstract::IntersectionRaycast(const IRay &ray , IRaycast *raycast) const
{
   return mAxisAlignedBoundingBox.TestRayIntersect(ray,GetTransformMatrixHierarchy(),raycast);
}

std::string IComponentAbstract::GetName() const
{
    return mName;
}

void IComponentAbstract::SetName(const std::string &name)
{
    mName = name;
}

IAABBox3D IComponentAbstract::GetAxisAlignedBoundingBox() const
{
    return mAxisAlignedBoundingBox;
}

void IComponentAbstract::SetAxisAlignedBoundingBox(const IAABBox3D &axisAlignedBoundingBox)
{
    mAxisAlignedBoundingBox = axisAlignedBoundingBox;
}


