#include "IMesh.h"
#include "IMeshGenerator.h"


namespace IEngine
{

using namespace MeshGenerator;


IMeshGenerate::IMeshGenerate(MeshGenerator::CuboidDescriptor __descriptor_cuboid__)
{
    MeshGenerator::GenerateMesh( __descriptor_cuboid__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::EllipsoidDescriptor __descriptor_Ellipsoid__)
{
    MeshGenerator::GenerateMesh( __descriptor_Ellipsoid__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::ConeDescriptor __descriptor_Cone__)
{
    MeshGenerator::GenerateMesh( __descriptor_Cone__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::CylinderDescriptor __descriptor_Cylinder__)
{
     MeshGenerator::GenerateMesh( __descriptor_Cylinder__, *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::PieDescriptor __descriptor_Pie__)
{
    MeshGenerator::GenerateMesh(__descriptor_Pie__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::PipeDescriptor __descriptor_Pipe__)
{
    MeshGenerator::GenerateMesh(__descriptor_Pipe__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::CapsuleDescriptor __descriptor_Capsule__)
{
     MeshGenerator::GenerateMesh( __descriptor_Capsule__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::TorusDescriptor __descriptor_Torus__)
{
    MeshGenerator::GenerateMesh(__descriptor_Torus__, *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::TorusKnotDescriptor __descriptor_TorusKnot__)
{
    MeshGenerator::GenerateMesh( __descriptor_TorusKnot__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::SpiralDescriptor __descriptor_Spiral__)
{
    MeshGenerator::GenerateMesh( __descriptor_Spiral__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::CurveDescriptor __descriptor_Curve__)
{
    MeshGenerator::GenerateMesh( __descriptor_Curve__ , *this );
}

IMeshGenerate::IMeshGenerate(MeshGenerator::BezierPatchDescriptor __descriptor_Bezier__)
{
    MeshGenerator::GenerateMesh( __descriptor_Bezier__ , *this );
}





}
