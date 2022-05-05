#include "IMeshGeneratorDetails.h"




namespace IEngine
{

    namespace MeshGenerator
    {

        void AddTriangulatedQuad(IMesh &mesh,
                                 bool alternateGrid,
                                 uint32_t u, uint32_t v,
                                 VertexIndex i0, VertexIndex i1,
                                 VertexIndex i2, VertexIndex i3,
                                 VertexIndex indexOffset)
        {
            auto Triangulate = [&mesh, indexOffset](VertexIndex a, VertexIndex b, VertexIndex c)
            {

                mesh.AppendTriangle( indexOffset + a,
                                     indexOffset + b,
                                     indexOffset + c );
            };

            if (!alternateGrid || u % 2 == v % 2)
            {
                /*
                  1-----2
                  |   / |
                  | /   |
                  0-----3
                  */
                Triangulate(i0, i1, i2);
                Triangulate(i0, i2, i3);
            }
            else
            {
                /*
                  1-----2
                  | \   |
                  |   \ |
                  0-----3
                  */
                Triangulate(i0, i1, i3);
                Triangulate(i1, i2, i3);

            }
        }


    } // /namespace MeshGenerator


} // /namespace IEngine


