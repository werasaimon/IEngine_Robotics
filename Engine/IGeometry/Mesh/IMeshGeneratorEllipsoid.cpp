
#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"

#include "../igeometry_types.hpp"

namespace IEngine
{

namespace MeshGenerator
{


void GenerateMesh(const EllipsoidDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsU = std::max(3, desc.segments.x);
    const auto segsV = std::max(2, desc.segments.y);

    const auto invSegsU = float(1) / static_cast<float>(segsU);
    const auto invSegsV = float(1) / static_cast<float>(segsV);

    /* Generate vertices */
    IMath::Sphericalf point(1, 0, 0);
    Vector2 texCoord;

    for (std::uint32_t v = 0; v <= segsV; ++v)
    {
        /* Compute theta of spherical coordinate */
        texCoord.y = static_cast<float>(v) * invSegsV;
        point.theta = texCoord.y * pi;

        for (std::uint32_t u = 0; u <= segsU; ++u)
        {
            /* Compute phi of spherical coordinate */
            texCoord.x = static_cast<float>(u) * invSegsU;
            point.phi = texCoord.x * pi_2;

            /* Convert spherical coordinate into cartesian coordinate and set normal by coordinate */
            auto coord = Vector3(point.GetConvertVector3());
            std::swap(coord.y, coord.z);

            /* Add new vertex */
            mesh.AppendVertex( VertexInfo( coord * desc.radius, coord.Normalized(), coord.Normalized(), texCoord));
        }
    }

    /* Generate indices */
    for (std::uint32_t v = 0; v < segsV; ++v)
    {
        for (std::uint32_t u = 0; u < segsU; ++u)
        {
            /* Compute indices for current face */
            auto i0 = v*(segsU + 1) + u;
            auto i1 = v*(segsU + 1) + (u + 1);

            auto i2 = (v + 1)*(segsU + 1) + (u + 1);
            auto i3 = (v + 1)*(segsU + 1) + u;

            /* Add new indices */
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxBaseOffset);
        }
    }
}


IMesh GenerateMeshT(const EllipsoidDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

} // /namespace MeshGenerator

} // /namespace Gm
