


#include "IMeshGeneratorDetails.h"
#include "IMeshGenerator.h"



namespace IEngine
{

namespace MeshGenerator
{


void GenerateMesh(const TorusKnotDescriptor& desc, IMesh& mesh)
{
    CurveDescriptor curveDesc;

    const auto loops = static_cast<float>(desc.loops);
    const auto turns = static_cast<float>(desc.turns);

    /* Pass torus-knot curve function to curve mesh generator */
    curveDesc.curveFunction = [&](float t)
    {
        t *= pi_2;

        auto p = loops;
        auto q = turns;
        auto r = std::cos(q*t) + desc.innerRadius;

        return Vector3(
            std::cos(p*t) * r,
            std::sin(q*t),
            std::sin(p*t) * r
        ) * desc.ringRadius;
    };

    curveDesc.radius            = desc.tubeRadius;
    curveDesc.segments          = desc.segments;
    curveDesc.alternateGrid     = desc.alternateGrid;
    curveDesc.vertexModifier    = desc.vertexModifier;

    GenerateMesh(curveDesc, mesh);
}

IMesh GenerateMeshT(const TorusKnotDescriptor& desc)
{
    IMesh mesh(1,0,0,0);
    GenerateMesh(desc, mesh);
    return mesh;
}




} // /namespace MeshGenerator

} // /namespace Gm
