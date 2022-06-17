
#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{


//============================================================================//
void GenerateMesh(const ConeDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsHorz = std::max(3, desc.mantleSegments.x);
    const auto segsVert = std::max(1, desc.mantleSegments.y);
    const auto segsCov = desc.coverSegments;

    const auto invHorz = float(1) / static_cast<float>(segsHorz);
    const auto invVert = float(1) / static_cast<float>(segsVert);

    const auto angleSteps = invHorz * pi_2;

    const auto halfHeight = desc.height*float(0.5);

    /* Generate mantle vertices */
    Vector3 coord, normal;
    Vector2 texCoord;

    const Vector3 tip(0, halfHeight, 0);
    coord.y = -halfHeight;

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

        /* Add bottom vertex */
        texCoord.x = static_cast<float>(segsHorz - u) * invHorz;

        for (std::uint32_t v = 1; v <= segsVert; ++v)
        {
            texCoord.y = static_cast<float>(v) * invVert;

            mesh.AppendVertex( VertexInfo(
                IMath::Lerp(tip, coord, texCoord.y),
                Vector3(0, 1, 0).Lerp(std::sqrt(texCoord.y) , normal ),
                Vector3(0, 1, 0).Lerp(std::sqrt(texCoord.y) , normal ),
                texCoord)
            );
        }

        /* Add top vertex */
        if (u < segsHorz)
        {
            texCoord.y = 0.0f;
            mesh.AppendVertex(VertexInfo(tip, Vector3::Y , Vector3::Z, texCoord));
        }

        /* Increase angle for the next iteration */
        angle += angleSteps;
    }

    /* Generate cover vertices */
    angle = float(0);
    VertexIndex coverIndexOffset = 0;

    if (segsCov > 0)
    {
        const auto invCov = float(1) / static_cast<float>(segsCov);

        /* Add centered bottom vertex */
        coverIndexOffset = mesh.AppendVertex( VertexInfo( Vector3(0, -halfHeight, 0 ),
                                                          Vector3( 0, -1, 0 ),
                                                          Vector3( 0, 0, -1 ),
                                                          Vector2(0.5,0.5) ) );

        for (std::uint32_t u = 0; u <= segsHorz; ++u)
        {
            /* Compute X- and Z coordinates */
            texCoord.x = std::sin(angle);
            texCoord.y = std::cos(angle);

            coord.x = texCoord.x * desc.radius.x;
            coord.z = texCoord.y * desc.radius.y;

            /* Add vertex around the bottom */
            for (std::uint32_t v = 1; v <= segsCov; ++v)
            {
                auto interp = static_cast<float>(v) * invCov;
                auto texCoordFinal = Vector2(float(0.5)) + texCoord * float(0.5) * interp;

                mesh.AppendVertex( VertexInfo(Vector3(0, -halfHeight, 0).Lerp( interp, coord ),
                                   Vector3(0, -1, 0),
                                   Vector3(0, 0, -1),
                                   texCoordFinal));
            }

            /* Increase angle for the next iteration */
            angle += angleSteps;
        }
    }

    /* Generate indices for the mantle */
    auto idxOffset = idxBaseOffset;

    for (std::uint32_t u = 0; u < segsHorz; ++u)
    {
        mesh.AppendTriangle(idxOffset + segsVert, idxOffset, idxOffset + 1 + segsVert);

        for (std::uint32_t v = 1; v < segsVert; ++v)
        {
            auto i0 = v + segsVert;
            auto i1 = v - 1;
            auto i2 = v;
            auto i3 = v + 1 + segsVert;

            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
        }

        idxOffset += (1 + segsVert);
    }

    if (segsCov > 0)
    {
        /* Generate indices for the bottom */
        idxOffset = coverIndexOffset + 1;

        for (std::uint32_t u = 0; u < segsHorz; ++u)
        {
            mesh.AppendTriangle(idxOffset + segsCov, idxOffset, coverIndexOffset);

            for (std::uint32_t v = 1; v < segsCov; ++v)
            {
                auto i1 = v - 1 + segsCov;
                auto i0 = v - 1;
                auto i3 = v;
                auto i2 = v + segsCov;

                AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
            }

            idxOffset += segsCov;
        }
    }
}

IMesh GenerateMeshT(const ConeDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}
//============================================================================//


} // /namespace MeshGenerator

} // /namespace Gm
