
#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"


namespace IEngine
{

namespace MeshGenerator
{



void GenerateMesh(const CurveDescriptor& desc, IMesh& mesh)
{
    const auto idxBaseOffset = mesh.VertexCount();

            const auto segsU = std::max(3, desc.segments.x);
            const auto segsV = std::max(3, desc.segments.y);


            //std::cout << segsU << std::endl;
            //const auto invSegsU = float(1) / static_cast<float>(segsU);
            //const auto invSegsV = float(1) / static_cast<float>(segsV);

            /* Sample curve progression function */
            std::vector<Vector3> curveSamples(segsU);

            for (std::uint32_t i = 0; i < segsU; ++i)
            {
                auto t = static_cast<float>(i) / (segsU - 1);
                curveSamples[i] = desc.curveFunction(t);
            }

            /* Generate vertices */
            Vector3 coord, normal, tangent, bitangent;
            Vector2 texCoord;

            for (std::uint32_t u = 0; u < segsU; ++u)
            {
                /* Compute texture X coordinate */
                texCoord.x = static_cast<float>(u) / (segsU - 1);

                for (std::uint32_t v = 0; v < segsV; ++v)
                {
                    /* Compute tangent vector from center of this ring to the next center */
                    tangent = curveSamples[(u + 1) % (segsU - 1)] - curveSamples[u];
                    tangent.Normalize();

                    /* Compute vector which is perpendicular to the tangent */
                    //if (!Gs::Equals(Gs::Dot(Gs::Vector3(0, 1, 0), tangent), float(1)))
                        bitangent = Vector3(0, 1, 0);
                    /*else
                        bitangent = Gs::Vector3(1, 0, 0);*/

                    bitangent = Cross(bitangent, tangent);
                    normal = Cross(tangent, bitangent);
                    normal.Normalize();

                    /* Compute coordinate and normal */
                    texCoord.y = static_cast<float>(v) / (segsV - 1);

                    normal = normal.RotateVectorAroundAxis(tangent, texCoord.y*pi_2);

                    auto displacement = desc.radius;
                    if (desc.vertexModifier)
                        displacement *= desc.vertexModifier(texCoord.x, texCoord.y);

                    coord = curveSamples[u] + normal * displacement;

                    mesh.AppendVertex(VertexInfo(coord, normal, normal, texCoord));
                }
            }

    /* Generate indices */
    VertexIndex i0, i1, i2, i3;

    for (std::uint32_t u = 0; u < segsU; ++u)
    {
        for (std::uint32_t v = 0; v < segsV; ++v)
        {
            i0 = u*segsV + v;

            if (v + 1 < segsV)
                i1 = u*segsV + v + 1;
            else
                i1 = u*segsV;

            if (u + 1 < segsU)
            {
                i2 = (u + 1)*segsV + v;
                if (v + 1 < segsV)
                    i3 = (u + 1)*segsV + v + 1;
                else
                    i3 = (u + 1)*segsV;
            }
            else
            {
                i2 = v;
                if (v + 1 < segsV)
                    i3 = v + 1;
                else
                    i3 = 0;
            }

            /* Add the computed quad */
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i3, i2, idxBaseOffset);
        }
    }
}

IMesh GenerateMeshT(const CurveDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}

//===============================================================================//

} // /namespace MeshGenerator




} // /namespace Gm
