#ifndef IMESHGENERATORDETAILS_H
#define IMESHGENERATORDETAILS_H


#include "IMesh.h"
#include <algorithm>


namespace IEngine
{


    namespace MeshGenerator
    {


        static const auto pi        = M_PI;
        static const auto pi_2      = pi*scalar(2.f);
        static const auto pi_0_5    = pi*scalar(0.5f);


        using VertexIndex = std::size_t;



        void AddTriangulatedQuad(
                IMesh& mesh,
                bool alternateGrid,
                std::uint32_t u, std::uint32_t v,
                VertexIndex i0, VertexIndex i1,
                VertexIndex i2, VertexIndex i3,
                VertexIndex indexOffset = 0
                );


    } // /namespace MeshGenerator

}

#endif // IMESHGENERATORDETAILS_H
