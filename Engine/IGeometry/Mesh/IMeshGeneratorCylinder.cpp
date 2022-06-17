

#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{

void GenerateMesh(const CylinderDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsHorz = std::max(3, desc.mantleSegments.x);
    const auto segsVert = std::max(1, desc.mantleSegments.y);

    const auto invHorz = float(1) / static_cast<float>(segsHorz);
    const auto invVert = float(1) / static_cast<float>(segsVert);

    const auto angleSteps = invHorz * pi_2;

    const auto halfHeight = desc.height*float(0.5);

    /* Generate mantle vertices */
    Vector3 coord, normal;
    Vector2 texCoord;

    auto angle = float(0);

    for (std::uint32_t u = 0; u <= segsHorz; ++u)
    {
        /* Compute X- and Z coordinates */
        texCoord.x = std::sin(angle);
        texCoord.y = std::cos(angle);

        coord.x = texCoord.x * desc.radius.x;
        coord.z = texCoord.y * desc.radius.y;

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
            mesh.AppendVertex(VertexInfo(coord, normal, normal, texCoord));
        }

        /* Increase angle for the next iteration */
        angle += angleSteps;
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

        /* Add centered vertex */
        coord.y = halfHeight * coverSide[i];

        coverIndexOffset[i] = mesh.AppendVertex(
            VertexInfo(Vector3( 0, coord.y, 0 ),
                       Vector3( 0, coverSide[i], 0 ),
                       Vector3( 0, coverSide[i], 0 ),
                       Vector2( float(0.5), float(0.5) ) )
        );

        for (std::uint32_t u = 0; u <= segsHorz; ++u)
        {
            /* Compute X- and Z coordinates */
            texCoord.x = std::sin(angle);
            texCoord.y = std::cos(angle);

            coord.x = texCoord.x * desc.radius.x;
            coord.z = texCoord.y * desc.radius.y;

            /* Add vertex around the top and bottom */
            for (std::uint32_t v = 1; v <= segsCov[i]; ++v)
            {
                auto interp = static_cast<float>(v) * invCov;
                auto texCoordFinal = Vector2(float(0.5)) + texCoord * float(0.5) * interp;

                if (i == 1)
                    texCoordFinal.y = float(1) - texCoordFinal.y;

                mesh.AppendVertex(
                   VertexInfo( Vector3(0, coord.y, 0).Lerp(interp, coord),
                               Vector3(0, coverSide[i], 0),
                               Vector3(0, coverSide[i], 0),
                               texCoordFinal)
                );
            }

            /* Increase angle for the next iteration */
            angle += angleSteps;
        }
    }

    /* Generate indices for the mantle */
    auto idxOffset = idxBaseOffset;

    for (std::uint32_t u = 0; u < segsHorz; ++u)
    {
        for (std::uint32_t v = 0; v < segsVert; ++v)
        {
            auto i0 = v + 1 + segsVert;
            auto i1 = v;
            auto i2 = v + 1;
            auto i3 = v + 2 + segsVert;

            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
        }

        idxOffset += (1 + segsVert);
    }

    /* Generate indices for the bottom and top */
    for (std::size_t i = 0; i < 2; ++i)
    {
        if (segsCov[i] == 0)
            continue;

        idxOffset = coverIndexOffset[i] + 1;

        for (std::uint32_t u = 0; u < segsHorz; ++u)
        {
            if (i == 0)
                mesh.AppendTriangle(idxOffset + segsCov[i], idxOffset, coverIndexOffset[i]);
            else
                mesh.AppendTriangle(coverIndexOffset[i], idxOffset, idxOffset + segsCov[i]);

            for (std::uint32_t v = 1; v < segsCov[i]; ++v)
            {
                auto i0 = v - 1;
                auto i1 = v - 1 + segsCov[i];
                auto i2 = v + segsCov[i];
                auto i3 = v;

                if (i == 0)
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
                else
                    AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxOffset);
            }

            idxOffset += segsCov[i];
        }
    }
}

IMesh GenerateMeshT(const CylinderDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

//===========================================================================================//

} // /namespace MeshGenerator

} // /namespace Gm
