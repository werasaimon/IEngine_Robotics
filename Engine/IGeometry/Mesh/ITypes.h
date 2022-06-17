#ifndef ITYPES_H
#define ITYPES_H

namespace IEngine
{

    static constexpr int InvalidID = -1;
    static constexpr int NonManifoldID = -2;
    static constexpr int MarkerID1 = -10;
    static constexpr int MarkerID2 = -11;
    static constexpr int MarkerID3 = -12;

    typedef int VertexID;
    typedef int TriangleID;
    typedef int EdgeID;

    typedef unsigned int GroupID;
    static constexpr GroupID InvalidGroupID = (1<<30);

}

#endif // ITYPES_H
