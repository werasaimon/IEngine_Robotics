


#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{


void GenerateMesh(const PipeDescriptor& desc, IMesh& mesh)
{
    const auto segsHorz = std::max(3, desc.mantleSegments.x);
    const auto segsVert = std::max(1, desc.mantleSegments.y);

    const auto invHorz = float(1) / static_cast<float>(segsHorz);
    const auto invVert = float(1) / static_cast<float>(segsVert);

    const auto angleSteps = invHorz * pi_2;

    const auto halfHeight = desc.height*float(0.5);

    /* Generate outer- and inner mantle vertices */
    Vector3 coord, normal, coordAlt;
    Vector2 texCoord;

    auto angle = float(0);

    const Vector2 radii[2] = { desc.outerRadius, desc.innerRadius };
    const float faceSide[2] = { 1, -1 };

    VertexIndex mantleIndexOffset[2] = { 0 };

    for (std::size_t i = 0; i < 2; ++i)
    {
        mantleIndexOffset[i] = mesh.VertexCount();
        angle = float(0);

        for (std::uint32_t u = 0; u <= segsHorz; ++u)
        {
            /* Compute X- and Z coordinates */
            texCoord.x = std::sin(angle);
            texCoord.y = std::cos(angle);

            coord.x = texCoord.x * radii[i].x;
            coord.z = texCoord.y * radii[i].y;

            /* Compute normal vector */
            normal.x = texCoord.x;
            normal.y = 0;
            normal.z = texCoord.y;
            normal.Normalize();

            /* Add top and bottom vertex */
            texCoord.x = static_cast<float>(segsHorz - u) * invHorz;

            for (std::uint32_t v = 0; v <= segsVert; ++v)
            {
                texCoord.y = static_cast<float>(v) * invVert;
                coord.y = IMath::Lerp(halfHeight, -halfHeight, texCoord.y);
                mesh.AppendVertex( VertexInfo( coord, normal * faceSide[i], normal * faceSide[i], texCoord) );
            }

            /* Increase angle for the next iteration */
            angle += angleSteps;
        }
    }

    /* Generate bottom and top cover vertices */
    const std::uint32_t segsCov[2]  = { desc.bottomCoverSegments, desc.topCoverSegments };
    const float coverSide[2]     = { -1, 1 };

    VertexIndex coverIndexOffset[2] = { 0 };

    for (std::size_t i = 0; i < 2; ++i)
    {
        if (segsCov[i] == 0)
            continue;

        angle = float(0);
        const auto invCov = float(1) / static_cast<float>(segsCov[i]);
        const auto invRadius = Vector2(1) / (desc.outerRadius * float(2.0));

        coord.y = halfHeight * coverSide[i];
        coordAlt.y = halfHeight * coverSide[i];
        coverIndexOffset[i] = mesh.VertexCount();

        for (std::uint32_t u = 0; u <= segsHorz; ++u)
        {
            /* Compute X- and Z coordinates */
            texCoord.x = std::sin(angle);
            texCoord.y = std::cos(angle);

            coord.x = texCoord.x * desc.outerRadius.x;
            coord.z = texCoord.y * desc.outerRadius.y;

            coordAlt.x = texCoord.x * desc.innerRadius.x;
            coordAlt.z = texCoord.y * desc.innerRadius.y;

            /* Add vertex around the top and bottom */
            for (std::uint32_t v = 0; v <= segsCov[i]; ++v)
            {
                auto interp = static_cast<float>(v) * invCov;
                auto texCoordA = Vector2(float(0.5)) + Vector2(coordAlt.x, coordAlt.z) * invRadius;
                auto texCoordB = Vector2(float(0.5)) + Vector2(coord.x, coord.z) * invRadius;

                if (i == 1)
                {
                    texCoordA.y = float(1) - texCoordA.y;
                    texCoordB.y = float(1) - texCoordB.y;
                }

                mesh.AppendVertex(
                    VertexInfo( Lerp(coordAlt, coord, interp),
                                Vector3(0, coverSide[i], 0),
                                Vector3(0, coverSide[i], 0),
                                Lerp(texCoordA, texCoordB, interp))
                );
            }

            /* Increase angle for the next iteration */
            angle += angleSteps;
        }
    }

    /* Generate indices for the outer mantle */
    VertexIndex idxOffset = 0;

    for (std::size_t i = 0; i < 2; ++i)
    {
        idxOffset = mantleIndexOffset[i];

        for (std::uint32_t u = 0; u < segsHorz; ++u)
        {
            for (std::uint32_t v = 0; v < segsVert; ++v)
            {
                auto i0 = v + 1 + segsVert;
                auto i1 = v;
                auto i2 = v + 1;
                auto i3 = v + 2 + segsVert;

                if (i == 0)
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
                else
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxOffset);
            }

            idxOffset += (1 + segsVert);
        }
    }

    /* Generate indices for the bottom and top */
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (segsCov[i] == 0)
            continue;

        idxOffset = coverIndexOffset[i];

        for (std::uint32_t u = 0; u < segsHorz; ++u)
        {
            for (std::uint32_t v = 0; v < segsCov[i]; ++v)
            {
                auto i0 = v;
                auto i1 = v + 1 + segsCov[i];
                auto i2 = v + 2 + segsCov[i];
                auto i3 = v + 1;

                if (i == 0)
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
                else
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxOffset);
            }

            idxOffset += segsCov[i] + 1;
        }
    }
}

IMesh GenerateMeshT(const PipeDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

//============================================================================//


} // /namespace MeshGenerator

} // /namespace Gm
