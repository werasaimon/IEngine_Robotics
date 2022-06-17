
#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{


void GenerateMesh(const TorusDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsU = std::max(3, desc.segments.x);
    const auto segsV = std::max(3, desc.segments.y);

    const auto invSegsU = float(1) / static_cast<float>(segsU);
    const auto invSegsV = float(1) / static_cast<float>(segsV);

    /* Generate vertices */
    Vector3 coord, normal;
    Vector2 texCoord;

    for (std::uint32_t v = 0; v <= segsV; ++v)
    {
        /* Compute theta of spherical coordinate */
        texCoord.y = static_cast<float>(v) * invSegsV;
        auto theta = texCoord.y * pi_2;

        auto s0 = std::sin(theta);
        auto c0 = std::cos(theta);

        coord.y = c0 * desc.tubeRadius.y;

        for (std::uint32_t u = 0; u <= segsU; ++u)
        {
            /* Compute phi of spherical coordinate */
            texCoord.x = static_cast<float>(u) * invSegsU;
            auto phi = texCoord.x * pi_2;

            auto s1 = std::sin(phi);
            auto c1 = std::cos(phi);

            /* Compute coordinate and normal */
            coord.x = s1 * desc.ringRadius.x + s1 * s0 * desc.tubeRadius.x;
            coord.z = c1 * desc.ringRadius.y + c1 * s0 * desc.tubeRadius.z;

            normal.x = s1 * s0 / desc.tubeRadius.x;
            normal.y =      c0 / desc.tubeRadius.y;
            normal.z = c1 * s0 / desc.tubeRadius.z;
            normal.Normalize();

            /* Add new vertex */
            texCoord.x = -texCoord.x;
            mesh.AppendVertex( VertexInfo( coord, normal, normal, texCoord) );
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
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxBaseOffset);
        }
    }
}

IMesh GenerateMeshT(const TorusDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

//=============================================================================//


} // /namespace MeshGenerator

} // /namespace Gm
