
#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"


namespace IEngine
{

namespace MeshGenerator
{


//================================================================//
void GenerateMesh(const BezierPatchDescriptor& desc, IMesh& mesh)
{
    auto idxOffset = mesh.VertexCount();

    const auto segsHorz = std::max(1, desc.segments.x);
    const auto segsVert = std::max(1, desc.segments.y);

    const auto invHorz  = float(1) / static_cast<float>(segsHorz);
    const auto invVert  = float(1) / static_cast<float>(segsVert);

    auto AddQuad = [&](std::uint32_t u, std::uint32_t v, VertexIndex i0, VertexIndex i1, VertexIndex i2, VertexIndex i3)
    {
        if (desc.backFacing)
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i1, i0, i3, i2, idxOffset);
        else
            AddTriangulatedQuad(mesh, desc.alternateGrid, u, v, i0, i1, i2, i3, idxOffset);
    };

    /* Generate vertices */
    static const float delta = float(0.01);

    Vector3 coord, normal;
    Vector2 texCoord;



    for (std::uint32_t i = 0; i <= segsVert; ++i)
    {
        for (std::uint32_t j = 0; j <= segsHorz; ++j)
        {
            /* Compute coordinate and texture-coordinate */
            texCoord.x = static_cast<float>(j) * invHorz;
            texCoord.y = static_cast<float>(i) * invVert;

            coord = desc.bezierPatch(texCoord.x, texCoord.y);

            /* Sample bezier patch to approximate normal */
            auto uOffset = (desc.bezierPatch(texCoord.x + delta, texCoord.y) - coord);
            auto vOffset = (desc.bezierPatch(texCoord.x, texCoord.y + delta) - coord);
            normal = Cross(uOffset, vOffset).Normalized();

            /* Add vertex */
            if (!desc.backFacing)
            {
                texCoord.y = float(1) - texCoord.y;
                normal = -normal;
            }

            mesh.AppendVertex(VertexInfo(coord, normal, normal, texCoord));
        }
    }



    /* Generate indices */
    const auto strideHorz = segsHorz + 1;

    for (std::uint32_t v = 0; v < segsVert; ++v)
    {
        for (std::uint32_t u = 0; u < segsHorz; ++u)
        {
            AddQuad(
                u, v,
                (  v   *strideHorz + u   ),
                ( (v+1)*strideHorz + u   ),
                ( (v+1)*strideHorz + u+1 ),
                (  v   *strideHorz + u+1 )
            );
        }
    }

}

IMesh GenerateMeshT(const BezierPatchDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}
//================================================================//

} // /namespace MeshGenerator





} // /namespace Gm
