

#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"


namespace IEngine
{

namespace MeshGenerator
{

void GenerateMesh(const CapsuleDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

    const auto segsHorz = std::max(3, desc.mantleSegments.x);
    const auto segsVert = std::max(1, desc.mantleSegments.y);
    const auto segsV    = std::max(std::uint32_t(2), desc.ellipsoidSegments);

    const auto invHorz  = float(1) / static_cast<float>(segsHorz);
    const auto invVert  = float(1) / static_cast<float>(segsVert);
    const auto invSegsV = float(1) / static_cast<float>(segsV);

    const auto angleSteps = invHorz * pi_2;

    const auto halfHeight = desc.height*float(0.5);

    /* Generate mantle vertices */
    IMath::Sphericalf point(1, 0, 0);
    Vector3 coord, normal;
    Vector2 texCoord;

    auto angle = float(0);

    for (std::uint32_t u = 0; u <= segsHorz; ++u)
    {
        /* Compute X- and Z coordinates */
        texCoord.x = std::sin(angle);
        texCoord.y = std::cos(angle);

        coord.x = texCoord.x * desc.radius.x;
        coord.z = texCoord.y * desc.radius.z;

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
            mesh.AppendVertex( VertexInfo( coord, normal, normal, texCoord) );
        }

        /* Increase angle for the next iteration */
        angle += angleSteps;
    }

    /* Generate bottom and top cover vertices */
    const float coverSide[2] = { 1, -1 };
    std::size_t idxBaseOffsetEllipsoid[2] = { 0 };

    for (std::size_t i = 0; i < 2; ++i)
    {
        idxBaseOffsetEllipsoid[i] = mesh.VertexCount();

        for (std::uint32_t v = 0; v <= segsV; ++v)
        {
            /* Compute theta of spherical coordinate */
            texCoord.y = static_cast<float>(v) * invSegsV;
            point.theta = texCoord.y * pi_0_5;

            for (std::uint32_t u = 0; u <= segsHorz; ++u)
            {
                /* Compute phi of spherical coordinate */
                texCoord.x = static_cast<float>(u) * invHorz;
                point.phi = texCoord.x * pi_2 * coverSide[i] + pi_0_5;

                /* Convert spherical coordinate into cartesian coordinate and set normal by coordinate */
                coord = Vector3(point.GetConvertVector3());
                std::swap(coord.y, coord.z);
                coord.y *= coverSide[i];

                /* Get normal and move half-sphere */
                normal = coord.Normalized();

                /* Transform coordiante with radius and height */
                coord *= desc.radius;
                coord.y += halfHeight * coverSide[i];

                //TODO: texCoord wrong for bottom half-sphere!!!
                /* Add new vertex */
                mesh.AppendVertex(VertexInfo(coord, normal,normal, texCoord));
            }
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

    /* Generate indices for the top and bottom */
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (std::uint32_t v = 0; v < segsV; ++v)
        {
            for (std::uint32_t u = 0; u < segsHorz; ++u)
            {
                /* Compute indices for current face */
                auto i0 = v*(segsHorz + 1) + u;
                auto i1 = v*(segsHorz + 1) + (u + 1);

                auto i2 = (v + 1)*(segsHorz + 1) + (u + 1);
                auto i3 = (v + 1)*(segsHorz + 1) + u;

                /* Add new indices */
                AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxBaseOffsetEllipsoid[i]);
            }
        }
    }
}

IMesh GenerateMeshT(const CapsuleDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}
//==================================================================//


} // /namespace MeshGenerator

} // /namespace Gm
