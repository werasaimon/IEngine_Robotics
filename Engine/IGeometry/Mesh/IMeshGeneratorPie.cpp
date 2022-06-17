


#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{


void GenerateMesh(const PieDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsHorz = std::max(3, desc.mantleSegments.x);
    const auto segsVert = std::max(1, desc.mantleSegments.y);
    const auto segsCov = desc.coverSegments;
    const auto segsCovMantle = std::max(1u, segsCov);

    const auto invHorz = float(1) / static_cast<float>(segsHorz);
    const auto invVert = float(1) / static_cast<float>(segsVert);
    const auto invCovMantle = float(1) / static_cast<float>(segsCovMantle);

    float m_pi_2 = pi_2;
    const auto pieAngle = IMath::IClamp(desc.angle,float(0.f),m_pi_2);
    const auto pieAngleOffset = desc.angleOffset + pieAngle;

    const auto angleSteps = invHorz * (pi_2 - pieAngle);

    const auto halfHeight = desc.height*float(0.5);

    /* Generate outer mantle vertices */
    Vector3 coord, normal;
    Vector2 texCoord;

    auto angle = pieAngleOffset;

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
            mesh.AppendVertex( VertexInfo(coord, normal, normal , texCoord) );
        }

        /* Increase angle for the next iteration */
        angle += angleSteps;
    }

    /* Generate inner mantle vertices */
    const float mantleSideAngles[2] = { pieAngleOffset, pieAngleOffset - pieAngle };
    const float mantleSideNormalOffset[2] = { static_cast<float>(pi + pi_0_5) ,
                                              static_cast<float>(pi_0_5) };
    const float mantleSideTC[4] = { 1, 0, -1, 1 };

    VertexIndex mantleIndexOffset[2] = { 0 };

    for (std::size_t i = 0; i < 2; ++i)
    {
        mantleIndexOffset[i] = mesh.VertexCount();

        /* Compute normal vector */
        const auto angleNormal = mantleSideAngles[i] + mantleSideNormalOffset[i];

        normal.x = std::sin(angleNormal);
        normal.y = float(0);
        normal.z = std::cos(angleNormal);

        for (std::uint32_t u = 0; u <= segsCovMantle; ++u)
        {
            /* Compute X- and Z coordinates */
            const auto r = static_cast<float>(u) * invCovMantle;

            texCoord.x = mantleSideTC[i] + r * mantleSideTC[i + 2];

            coord.x = std::sin(mantleSideAngles[i]) * desc.radius.x * r;
            coord.z = std::cos(mantleSideAngles[i]) * desc.radius.y * r;

            for (std::uint32_t v = 0; v <= segsVert; ++v)
            {
                /* Vertical coordinates */
                texCoord.y = static_cast<float>(v) * invVert;
                coord.y = IMath::Lerp(halfHeight, -halfHeight, texCoord.y);

                /* Add new vertex */
                mesh.AppendVertex(VertexInfo(coord, normal, normal, texCoord));
            }
        }
    }

    /* Generate bottom and top cover vertices */
    VertexIndex coverIndexOffset[2] = { 0 };

    if (segsCov > 0)
    {
        const auto invCov = float(1) / static_cast<float>(segsCov);

        const float coverSide[2] = { -1, 1 };

        for (std::size_t i = 0; i < 2; ++i)
        {
            angle = pieAngleOffset;

            /* Add centered vertex */
            coord.y = halfHeight * coverSide[i];

            coverIndexOffset[i] = mesh.AppendVertex(
               VertexInfo( Vector3( 0, coord.y, 0 ),
                           Vector3( 0, coverSide[i], 0 ),
                           Vector3( 0, coverSide[i], 0 ),
                           Vector2( float(0.5), float(0.5) ))
            );

            for (std::uint32_t u = 0; u <= segsHorz; ++u)
            {
                /* Compute X- and Z coordinates */
                texCoord.x = std::sin(angle);
                texCoord.y = std::cos(angle);

                coord.x = texCoord.x * desc.radius.x;
                coord.z = texCoord.y * desc.radius.y;

                /* Add vertex around the top and bottom */
                for (std::uint32_t v = 1; v <= segsCov; ++v)
                {
                    auto interp = static_cast<float>(v) * invCov;
                    auto texCoordFinal = Vector2(float(0.5)) + texCoord * float(0.5) * interp;

                    if (i == 1)
                        texCoordFinal.y = float(1) - texCoordFinal.y;

                    mesh.AppendVertex(
                       VertexInfo( Vector3(0, coord.y, 0).Lerp(interp,coord),
                                   Vector3(0, coverSide[i], 0),
                                   Vector3(0, coverSide[i], 0),
                                   texCoordFinal)
                    );
                }

                /* Increase angle for the next iteration */
                angle += angleSteps;
            }
        }
    }

    /* Generate indices for the outer mantle */
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

    /* Generate indices for the inner mantle */
    for (std::size_t i = 0; i < 2; ++i)
    {
        idxOffset = mantleIndexOffset[i];

        for (std::uint32_t u = 0; u < segsCovMantle; ++u)
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
    if (segsCov > 0)
    {
        for (std::size_t i = 0; i < 2; ++i)
        {
            idxOffset = coverIndexOffset[i] + 1;

            for (std::uint32_t u = 0; u < segsHorz; ++u)
            {
                if (i == 0)
                    mesh.AppendTriangle(idxOffset + segsCov, idxOffset, coverIndexOffset[i]);
                else
                    mesh.AppendTriangle(coverIndexOffset[i], idxOffset, idxOffset + segsCov);

                for (std::uint32_t v = 1; v < segsCov; ++v)
                {
                    auto i0 = v - 1;
                    auto i1 = v - 1 + segsCov;
                    auto i2 = v + segsCov;
                    auto i3 = v;

                    if (i == 0)
                        AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
                    else
                        AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxOffset);
                }

                idxOffset += segsCov;
            }
        }
    }
}

IMesh GenerateMeshT(const PieDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

//==================================================================================//


} // /namespace MeshGenerator

} // /namespace Gm
