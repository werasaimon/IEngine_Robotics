


#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{
void GenerateMesh(const SpiralDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto turns = std::max(float(0), desc.turns);

    const auto segsU = std::max(3, desc.mantleSegments.x);
    const auto segsV = std::max(3, desc.mantleSegments.y);

    const auto invSegsU = float(1) / static_cast<float>(segsU);
    const auto invSegsV = float(1) / static_cast<float>(segsV);

    const auto totalSegsU = static_cast<std::uint32_t>(turns * static_cast<float>(segsU));

    auto GetCoverCoordAndNormal = [&](float theta, float phi, Vector3& coord, Vector3& normal, bool center)
    {
        auto s1 = std::sin(phi);
        auto c1 = std::cos(phi);

        coord.x = s1 * desc.ringRadius.x;
        coord.y = ((phi / pi_2) - turns * float(0.5)) * desc.displacement;
        coord.z = c1 * desc.ringRadius.y;

        if (!center)
        {
            auto s0 = std::sin(theta);
            auto c0 = std::cos(theta);

            coord.x += s1 * s0 * desc.tubeRadius.x;
            coord.y +=      c0 * desc.tubeRadius.y;
            coord.z += c1 * s0 * desc.tubeRadius.z;
        }

        normal.x = c1;
        normal.y = 0;
        normal.z = s1;

        return coord;
    };

    /* Generate mantle vertices */
    Vector3 coord, normal;
    Vector2 texCoord;

    for (std::uint32_t v = 0; v <= segsV; ++v)
    {
        /* Compute theta of spherical coordinate */
        texCoord.y = static_cast<float>(v) * invSegsV;
        auto theta = texCoord.y * pi_2;

        auto s0 = std::sin(theta);
        auto c0 = std::cos(theta);

        for (std::uint32_t u = 0; u <= totalSegsU; ++u)
        {
            /* Compute phi of spherical coordinate */
            texCoord.x = static_cast<float>(u) * invSegsU;
            auto phi = texCoord.x * pi_2;

            auto s1 = std::sin(phi);
            auto c1 = std::cos(phi);

            /* Compute coordinate and normal */
            coord.x = s1 * desc.ringRadius.x + s1 * s0 * desc.tubeRadius.x;
            coord.y = c0 * desc.tubeRadius.y + (texCoord.x - turns * float(0.5)) * desc.displacement;
            coord.z = c1 * desc.ringRadius.y + c1 * s0 * desc.tubeRadius.z;

            normal.x = s1 * s0 / desc.tubeRadius.x;
            normal.y =      c0 / desc.tubeRadius.y;
            normal.z = c1 * s0 / desc.tubeRadius.z;
            normal.Normalize();

            /* Add new vertex */
            texCoord.x = -texCoord.x;
            mesh.AppendVertex( VertexInfo(coord, normal, normal, texCoord));
        }
    }

    /* Generate bottom and top cover vertices */
    const std::uint32_t segsCov[2]  = { desc.bottomCoverSegments, desc.topCoverSegments };
    const float coverPhi[2]      = { 0, static_cast<float>(turns * pi_2) };
    const float coverSide[2]     = { -1, 1 };

    VertexIndex coverIndexOffset[2] = { 0 };

    for (std::size_t i = 0; i < 2; ++i)
    {
        if (segsCov[i] == 0)
            continue;

        auto theta = float(0);
        const auto invCov = float(1) / static_cast<float>(segsCov[i]);

        /* Add centered vertex */
        GetCoverCoordAndNormal(0, coverPhi[i], coord, normal, true);
        coverIndexOffset[i] = mesh.AppendVertex(
           VertexInfo( coord,
                       normal * coverSide[i],
                       normal * coverSide[i],
                       Vector2(float(0.5), float(0.5)))
        );

        auto centerCoord = coord;

        for (std::uint32_t v = 0; v <= segsV; ++v)
        {
            /* Compute texture coordinates */
            texCoord.x = std::sin(theta);
            texCoord.y = std::cos(theta);

            /* Add vertex around the top and bottom */
            for (std::uint32_t j = 1; j <= segsCov[i]; ++j)
            {
                auto interp = static_cast<float>(j) * invCov;
                auto texCoordFinal = Vector2(float(0.5)) + texCoord * float(0.5) * interp;

                if (i == 1)
                    texCoordFinal.y = float(1) - texCoordFinal.y;

                GetCoverCoordAndNormal(theta, coverPhi[i], coord, normal, false);
                mesh.AppendVertex(
                   VertexInfo( Lerp(centerCoord, coord, interp),
                               normal * coverSide[i],
                               normal * coverSide[i],
                               texCoordFinal)
                );
            }

            /* Increase angle for the next iteration */
            theta += invSegsV * pi_2;
        }
    }

    /* Generate indices for the mantle */
    for (std::uint32_t v = 0; v < segsV; ++v)
    {
        for (std::uint32_t u = 0; u < totalSegsU; ++u)
        {
            /* Compute indices for current face */
            auto i0 = v*(totalSegsU + 1) + u;
            auto i1 = v*(totalSegsU + 1) + (u + 1);

            auto i2 = (v + 1)*(totalSegsU + 1) + (u + 1);
            auto i3 = (v + 1)*(totalSegsU + 1) + u;

            /* Add new indices */
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxBaseOffset);
        }
    }

    /* Generate indices for the bottom and top */
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (segsCov[i] == 0)
            continue;

        auto idxOffset = coverIndexOffset[i] + 1;

        for (std::uint32_t v = 0; v < segsV; ++v)
        {
            if (i == 0)
                mesh.AppendTriangle(idxOffset + segsCov[i], idxOffset, coverIndexOffset[i]);
            else
                mesh.AppendTriangle(coverIndexOffset[i], idxOffset, idxOffset + segsCov[i]);

            for (std::uint32_t j = 1; j < segsCov[i]; ++j)
            {
                auto i1 = j - 1 + segsCov[i];
                auto i0 = j - 1;
                auto i3 = j;
                auto i2 = j + segsCov[i];

                if (i == 0)
                    AddTriangulatedQuad(mesh, desc.alternateGrid, v, j, i0, i1, i2, i3, idxOffset);
                else
                    AddTriangulatedQuad(mesh, desc.alternateGrid, v, j, i1, i0, i3, i2, idxOffset);
            }

            idxOffset += segsCov[i];
        }
    }
}

IMesh GenerateMeshT(const SpiralDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}




} // /namespace MeshGenerator

} // /namespace Gm

