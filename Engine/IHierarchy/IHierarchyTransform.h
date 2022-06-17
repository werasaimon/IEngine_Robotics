#ifndef IHIERARCHYTRANSFORM_H
#define IHIERARCHYTRANSFORM_H

#include "../imaths.hpp"
#include "IHierarchy/IHierarchyNode.h"


namespace IEngine
{


class IHierarchyTransform : public AffineTransform , public IHierarchyNode
{
  public:

    IHierarchyTransform(unsigned int id = 0);


    Matrix4 GetTransformMatrixHierarchy() const;
    Transform GetTransformHierarchy() const;

    Transform GetTransfom() const;

    void SetTransformMatrix( const Matrix4& m );
    void SetTransform( const Transform& m );
};

}

#endif // IHIERARCHYTRANSFORM_H
