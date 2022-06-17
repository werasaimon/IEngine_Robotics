#ifndef IMESH_H
#define IMESH_H


#ifndef IOBJECTMESH_H
#define IOBJECTMESH_H



#include <string>
#include <map>


#include "../../imaths.hpp"
#include "../IAABBox3D.h"

#include "memory/dvector.h"
#include "memory/refcount_vector.h"
#include "memory/small_list_set.h"

#include "IIndexUtil.h"

#include "../../IHierarchy/IHierarchyTransform.h"


namespace IEngine
{



//--------------------------------------------------//

struct CuboidDescriptor;

//--------------------------------------------------//


namespace
{
    template <class Real>
    IMath::IVector3D<Real> Lerp(const IMath::IVector3D<Real> & v1, const IMath::IVector3D<Real> & v2, Real t)
    {
        return (1-t) * v1 + (t) * v2;
    }


    static bool  IRayIntersectionToTriangle(const Vector3& vertA ,
                                            const Vector3& vertB ,
                                            const Vector3& vertC ,
                                            const Vector3& origin,
                                            const Vector3& direction,
                                            float &t,
                                            float &u,
                                            float &v)
    {
        Vector3 e1 = vertB - vertA;
        Vector3 e2 = vertC - vertA;
        Vector3 p = direction.Cross(e2);
        float a = e1.Dot(p);

        if(a == 0)
            return false;

        float f = 1.0f/a;

        Vector3 s = origin - vertA;
        u = f * s.Dot(p);
        if(u < 0.0f || u > 1.0f)
            return false;

        Vector3 q = s.Cross(e1);
        v = f * direction.Dot(q);

        if(v < 0.0f || u+v > 1.0f)
            return false;

        t = f * e2.Dot(q);

        bool success = (t >= 0.0f && t <= 1.0f);
        return !success;

    }
}




enum class MeshResult
{
    Ok = 0,
    Failed_NotAVertex = 1,
    Failed_NotATriangle = 2,
    Failed_NotAnEdge = 3,

    Failed_BrokenTopology = 10,
    Failed_HitValenceLimit = 11,

    Failed_IsBoundaryEdge = 20,
    Failed_FlippedEdgeExists = 21,
    Failed_IsBowtieVertex = 22,
    Failed_InvalidNeighbourhood = 23,       // these are all failures for CollapseEdge
    Failed_FoundDuplicateTriangle = 24,
    Failed_CollapseTetrahedron = 25,
    Failed_CollapseTriangle = 26,
    Failed_NotABoundaryEdge = 27,
    Failed_SameOrientation = 28,

    Failed_WouldCreateBowtie = 30,
    Failed_VertexAlreadyExists = 31,
    Failed_CannotAllocateVertex = 32,

    Failed_WouldCreateNonmanifoldEdge = 50,
    Failed_TriangleAlreadyExists = 51,
    Failed_CannotAllocateTriangle = 52
};

enum class MeshComponents
{
    None = 0,
    VertexNormals = 1,
    VertexColors = 2,
    VertexUVs = 4,
    FaceGroups = 8
};

enum class MeshHints
{
    None = 0,
    IsCompact = 1
};



/*
* Abstracts construction of meshes, so that we can construct different types, etc
*/
struct VertexInfo
{
    Vector3 v;
    Vector3 n, t, c;
    Vector2 uv;
    bool bHaveN, bHaveT, bHaveUV, bHaveC;

    VertexInfo()
    {
        this->v = Vector3::ZERO; n = c = Vector3::ZERO; uv = Vector2::ZERO;
        bHaveN = bHaveT = bHaveC = bHaveUV = false;
    }

    VertexInfo(const Vector3 & v)
    {
        this->v = v; n = c = Vector3::ZERO; uv = Vector2::ZERO;
        bHaveN = bHaveT = bHaveC = bHaveUV = false;
    }

    VertexInfo(const Vector3 & v, const Vector3 & n)
    {
        this->v = v; this->n = n; c = Vector3::ZERO; uv = Vector2::ZERO;
        bHaveN = true; bHaveT = bHaveC = bHaveUV = false;
    }

    VertexInfo(const Vector3 & v, const Vector3 & n, const Vector3 & c)
    {
        this->v = v; this->n = n; this->c = c; uv = Vector2::ZERO;
        bHaveN = bHaveC = true; bHaveT = bHaveUV = false;
    }

    VertexInfo(const Vector3 & v, const Vector3 & n, const Vector3 & c, const Vector2 & uv)
    {
        this->v = v; this->n = n; this->c = c; this->uv = uv;
        bHaveN = bHaveC = bHaveUV = true; bHaveT = false;
    }


    VertexInfo(bool tanget , const Vector3 & v, const Vector3 & n, const Vector3 &t, const Vector2 & uv)
    {
        bHaveT = tanget;
        this->v = v; this->n = n; this->t = t; this->uv = uv;
        bHaveN = bHaveC = bHaveUV = true; bHaveT = false;
    }

    VertexInfo(const Vector3 & v, const Vector3 & n, const Vector3 &t, const Vector3 &c, const Vector2 & uv)
    {
        this->v = v; this->n = n; this->t = t; this->c = c; this->uv = uv;
        bHaveT = bHaveN = bHaveC = bHaveUV = true; bHaveT = false;
    }


    VertexInfo(const Vector3 & v, const Vector2 & uv)
    {
        this->v = v;  this->uv = uv;
        bHaveUV = true;
    }
};









//
// IObjectMesh is a dynamic triangle mesh class. The mesh has has connectivity,
//  is an indexed mesh, and allows for gaps in the index space.
//
// internally, all data is stored in POD-type buffers, except for the vertex->edge
// links, which are stored as List<int>'s. The arrays of POD data are stored in
// dvector's, so they grow in chunks, which is relatively efficient. The actual
// blocks are arrays, so they can be efficiently mem-copied into larger buffers
// if necessary.
//
// Reference counts for verts/tris/edges are stored as separate refcount_vector
// instances.
//
// Vertices are stored as floats, although this should be easily changed
// if necessary, as the internal data structure is not exposed
//
// Per-vertex Vertex Normals, Colors, and UVs are optional and stored as floats.
//
// For each vertex, mVertexEdges[i] is the unordered list of connected edges. The
// elements of the list are indices into the edges list.
// This list is unsorted but can be traversed in-order (ie cw/ccw) at some additional cost.
//
// Triangles are stored as 3 ints, with optionally a per-triangle integer group id.
//
// The edges of a triangle are similarly stored as 3 ints, in triangle_edes. If the
// triangle is [v1,v2,v3], then the triangle edges [e1,e2,e3] are
// e1=edge(v1,v2), e2=edge(v2,v3), e3=edge(v3,v1), where the e# are indexes into edges.
//
// Edges are stored as tuples of 4 ints. If the edge is between v1 and v2, with neighbour
// tris t1 and t2, then the edge is [min(v1,v2), max(v1,v2), t1, t2]. For a boundary
// edge, t2 is InvalidID. t1 is never InvalidID.
//
// Most of the class assumes that the mesh is manifold. Many functions will
// work if the topology is non-manifold, but behavior of operators like Split/Flip/Collapse
// edge is untested.
//
// The function CheckValidity() does extensive sanity checking on the mesh data structure.
// Use this to test your code, both for mesh construction and editing!!
//
//
// TODO:
//  - dvector w/ 'stride' option, so that we can guarantee that tuples are in single block.
//    The can have custom accessor that looks up entire tuple
class IMesh : public IHierarchyTransform
{

public:

    static constexpr float max_float = std::numeric_limits<float>::max();
    static constexpr int InvalidID = -1;
    static constexpr int NonManifoldID = -2;

    static Vector3  InvalidVertex()   { return Vector3(max_float, 0, 0); }
    static Index3i  InvalidTriangle() { return Index3i(InvalidID, InvalidID, InvalidID); }
    static Index2i  InvalidEdge()     { return Index2i(InvalidID, InvalidID); }


protected:

    IMemory::refcount_vector vertices_refcount;
    IMemory::dvector<float>  mVertices;
    IMemory::dvector<float>  mNormals;
    IMemory::dvector<float>  mTangents;
    IMemory::dvector<float>  mColors;
    IMemory::dvector<float>  mUV;


    // [TODO] this is optional if we only want to use this class as an iterable mesh-with-nbrs
     //   make it optional with a flag? (however find_edge depends on it...)
    IMemory::small_list_set  mVertexEdges;


    IMemory::refcount_vector triangles_refcount;
    IMemory::dvector<int>    mTriangles;
    IMemory::dvector<int>    mTriangleEdges;
    IMemory::dvector<int>    mTriangleGroups;


    IMemory::refcount_vector edges_refcount;
    IMemory::dvector<int>    mEdges;

    int timestamp = 0;
    int shape_timestamp = 0;
    int max_group_id = 0;


    //int mTextureID;

    ///// <summary>
    ///// Support attaching arbitrary data to mesh.
    ///// Note that metadata is currently **NOT** copied when copying a mesh.
    ///// </summary>
    //Dictionary<string, object> Metadata = nullptr;


    IAABBox3D mAxisAlignedBoundingBox;

public:



//    IMesh() = default;
//    IMesh(const IMesh&) = default;


    IMesh(CuboidDescriptor __descriptor__ );



    IMesh(bool bWantNormals = true, bool bWantTangents = true, bool bWantColors = false, bool bWantUVs = false, bool bWantTriGroups = false)
    {
        mVertices = IMemory::dvector<float>();
        if ( bWantNormals) mNormals = IMemory::dvector<float>();
        if ( bWantTangents) mTangents = IMemory::dvector<float>();
        if ( bWantColors ) mColors = IMemory::dvector<float>();
        if ( bWantUVs )    mUV = IMemory::dvector<float>();

        mVertexEdges = IMemory::small_list_set();

        vertices_refcount = IMemory::refcount_vector();

        mTriangles = IMemory::dvector<int>();
        mTriangleEdges = IMemory::dvector<int>();
        triangles_refcount = IMemory::refcount_vector();

        if ( bWantTriGroups ) mTriangleGroups = IMemory::dvector<int>();
        max_group_id = 0;

        mEdges = IMemory::dvector<int>();
        edges_refcount = IMemory::refcount_vector();
    }



    IMesh(MeshComponents flags) :
        IMesh( ((int)flags & (int)MeshComponents::VertexNormals) != 0,  ((int)flags & (int)MeshComponents::VertexColors) != 0,
               ((int)flags & (int)MeshComponents::VertexUVs) != 0,      ((int)flags & (int)MeshComponents::FaceGroups) != 0 )
    {
    }


    // normals/colors/uvs will only be copied if they exist
     IMesh(const IMesh & copy, bool bCompact = false, bool bWantNormals = true, bool bWantColors = true, bool bWantUVs = true)
     {
         if (bCompact)
         {
             CompactCopy(copy, bWantNormals, bWantColors, bWantUVs);
         }
         else
         {
             Copy(copy, bWantNormals, bWantColors, bWantUVs);
         }
     }

     IMesh(const IMesh& copy, bool bCompact, MeshComponents flags) :
         IMesh(copy, bCompact,
        ((int)flags & (int)MeshComponents::VertexNormals) != 0,
        ((int)flags & (int)MeshComponents::VertexColors) != 0,
        ((int)flags & (int)MeshComponents::VertexUVs) != 0 )
     {
     }



     void ClearFullData()
     {
         mAxisAlignedBoundingBox = IAABBox3D::Zero;

         vertices_refcount.rebuild_free_list();
         mVertices.clear();
         mNormals.clear();
         mTangents.clear();
         mColors.clear();
         mUV.clear();

         mVertexEdges.Resize(0);

         triangles_refcount.rebuild_free_list();
         mTriangles.clear();
         mTriangleEdges.clear();
         mTriangleGroups.clear();

         edges_refcount.rebuild_free_list();
         mEdges.clear();
     }


public:


    // iterators
    //   The functions vertices() / triangles() / edges() are provided so you can do:
    //      for ( int eid : edges() ) { ... }
    //   and other related begin() / end() idioms
    //
    typedef typename IMemory::refcount_vector::index_enumerable vertex_iterator;
    vertex_iterator VertexIndices() const
    {
        return vertices_refcount.indices();
    }

    typedef typename IMemory::refcount_vector::index_enumerable triangle_iterator;
    triangle_iterator TriangleIndices() const
    {
        return triangles_refcount.indices();
    }

    typedef typename IMemory::refcount_vector::index_enumerable edge_iterator;
    edge_iterator EdgeIndices() const
    {
        return edges_refcount.indices();
    }


    //---------------------------------//

    IAABBox3D GetAABBTransform() const
    {
        return mAxisAlignedBoundingBox.GetAxisAlignedBoxTransform(mTransformMatrix);
    }

    IAABBox3D GetAABBTransform(Matrix4 transform ) const
    {
        return mAxisAlignedBoundingBox.GetAxisAlignedBoxTransform(transform);
    }

    IAABBox3D GetAABBLocal() const
    {
        return mAxisAlignedBoundingBox.GetAxisAlignedBoxTransform(Matrix4::IDENTITY);
    }

    //--------------------------------//



    void CalculateNormals()
    {
        mNormals.resize(mVertices.size());
        for (int i=0; i < TriangleCount(); ++i)
        {
             Index3i index = GetTriangle(i);

             Vector3 p = GetVertex(index.x);
             Vector3 q = GetVertex(index.y);
             Vector3 r = GetVertex(index.z);

//             SetVertexNormal( index.x , p);
//             SetVertexNormal( index.y , q);
//             SetVertexNormal( index.z , r);


             Vector3 normal = (q-p).Cross(r-p).Normalized();
             if(p.Dot(normal) < 0) normal = -normal;

             SetVertexNormal( index.x , (GetVertexNormal(index.x) + normal));
             SetVertexNormal( index.y , (GetVertexNormal(index.y) + normal));
             SetVertexNormal( index.z , (GetVertexNormal(index.z) + normal));
        }

        for (int i=0; i < TriangleCount(); ++i)
        {
             Index3i index = GetTriangle(i);
             SetVertexNormal( index.x , GetVertexNormal(index.x).Normalized());
             SetVertexNormal( index.y , GetVertexNormal(index.y).Normalized());
             SetVertexNormal( index.z , GetVertexNormal(index.z).Normalized());
        }
    }




    void CalculateTangets()
    {
        mTangents.resize(mVertices.size());
        for (int i=0; i < TriangleCount(); ++i)
        {
             Index3i index = GetTriangle(i);

             // Get the vertices positions
             Vector3 p = GetVertex(index.x);
             Vector3 q = GetVertex(index.y);
             Vector3 r = GetVertex(index.z);

             // Get the texture coordinates of each vertex
             Vector2 uvP = GetVertexUV(index.x);
             Vector2 uvQ = GetVertexUV(index.y);
             Vector2 uvR = GetVertexUV(index.z);

             // Get the three edges
             Vector3 edge1 = q - p;
             Vector3 edge2 = r - p;
             Vector2 edge1UV = uvQ - uvP;
             Vector2 edge2UV = uvR - uvP;

             float cp = edge1UV.y * edge2UV.x - edge1UV.x * edge2UV.y;

             // Compute the tangent
             if (cp != 0.0f)
             {
                 float factor = 1.0f / cp;
                 Vector3 tangent = (edge1 * -edge2UV.y + edge2 * edge1UV.y) * factor;
                 tangent.Normalize();

                 SetVertexTangents( index.x , (GetVertexTangents(index.x) + tangent));
                 SetVertexTangents( index.y , (GetVertexTangents(index.y) + tangent));
                 SetVertexTangents( index.z , (GetVertexTangents(index.z) + tangent));
             }

             //-------------------------------------------//

//             Vector3 v0 = GetVertex(index.x);
//             Vector3 v1 = GetVertex(index.y);
//             Vector3 v2 = GetVertex(index.z);

//             Vector3 Edge1 = v1 - v0;
//             Vector3 Edge2 = v2 - v0;


//             Vector2 v0_tex = GetVertexUV(index.x);
//             Vector2 v1_tex = GetVertexUV(index.y);
//             Vector2 v2_tex = GetVertexUV(index.z);

//             float DeltaU1 = v1_tex.x - v0_tex.x;
//             float DeltaV1 = v1_tex.y - v0_tex.y;
//             float DeltaU2 = v2_tex.x - v0_tex.x;
//             float DeltaV2 = v2_tex.y - v0_tex.y;

//             float f = 1.0f / (DeltaU1 * DeltaV2 - DeltaU2 * DeltaV1);

//             Vector3 Tangent, Bitangent;

//             Tangent.x = f * (DeltaV2 * Edge1.x - DeltaV1 * Edge2.x);
//             Tangent.y = f * (DeltaV2 * Edge1.y - DeltaV1 * Edge2.y);
//             Tangent.z = f * (DeltaV2 * Edge1.z - DeltaV1 * Edge2.z);

//             Bitangent.x = f * (-DeltaU2 * Edge1.x - DeltaU1 * Edge2.x);
//             Bitangent.y = f * (-DeltaU2 * Edge1.y - DeltaU1 * Edge2.y);
//             Bitangent.z = f * (-DeltaU2 * Edge1.z - DeltaU1 * Edge2.z);


//             SetVertexTangents( index.x , (GetVertexTangents(index.x) + Tangent));
//             SetVertexTangents( index.y , (GetVertexTangents(index.y) + Tangent));
//             SetVertexTangents( index.z , (GetVertexTangents(index.z) + Tangent));
        }

        for (int i=0; i < TriangleCount(); ++i)
        {
             Index3i index = GetTriangle(i);
             SetVertexTangents( index.x , GetVertexTangents(index.x).Normalized());
             SetVertexTangents( index.y , GetVertexTangents(index.y).Normalized());
             SetVertexTangents( index.z , GetVertexTangents(index.z).Normalized());
        }

    }


    //--------------------------------//

//    Vector3* GetVerteices() const
//    {
//        Vector3 v(VerticesBuffer().front());
//        return &v;
//    }


protected:

    // internal
    void set_triangle(int tid, int v0, int v1, int v2)
    {
        int i = 3 * tid;
        mTriangles[i] = v0;
        mTriangles[i + 1] = v1;
        mTriangles[i + 2] = v2;
    }
    void set_triangle_edges(int tid, int e0, int e1, int e2)
    {
        int i = 3 * tid;
        mTriangleEdges[i] = e0;
        mTriangleEdges[i + 1] = e1;
        mTriangleEdges[i + 2] = e2;
    }

    int add_edge(int vA, int vB, int tA, int tB = InvalidID)
    {
        if (vB < vA) {
            int t = vB; vB = vA; vA = t;
        }
        int eid = edges_refcount.allocate();
        int i = 4 * eid;
        mEdges.insertAt(vA, i);
        mEdges.insertAt(vB, i + 1);
        mEdges.insertAt(tA, i + 2);
        mEdges.insertAt(tB, i + 3);

        mVertexEdges.Insert(vA, eid);
        mVertexEdges.Insert(vB, eid);
        return eid;
    }

    int replace_tri_vertex(int tID, int vOld, int vNew)
    {
        int i = 3 * tID;
        if (mTriangles[i] == vOld) { mTriangles[i] = vNew; return 0; }
        if (mTriangles[i + 1] == vOld) { mTriangles[i + 1] = vNew; return 1; }
        if (mTriangles[i + 2] == vOld) { mTriangles[i + 2] = vNew; return 2; }
        return -1;
    }

    int add_triangle_only(int a, int b, int c, int e0, int e1, int e2)
    {
        int tid = triangles_refcount.allocate();
        int i = 3 * tid;
        mTriangles.insertAt(c, i + 2);
        mTriangles.insertAt(b, i + 1);
        mTriangles.insertAt(a, i);
        mTriangleEdges.insertAt(e2, i + 2);
        mTriangleEdges.insertAt(e1, i + 1);
        mTriangleEdges.insertAt(e0, i + 0);
        return tid;
    }



    void allocate_edges_list(int vid)
    {
        if (vid < (int)mVertexEdges.Size())
            mVertexEdges.Clear(vid);
            mVertexEdges.AllocateAt(vid);
    }


    std::vector<int> vertex_edges_list(int vid) const
    {
        std::vector<int> list;
        for (int eid : mVertexEdges.values(vid))
            list.push_back(vid);
        return list;
    }
    //List<int> vertex_vertices_list(int vid)
    //{
    //	List<int> vnbrs = List<int>();
    //	foreach(int eid in mVertexEdges.ValueItr(vid))
    //		vnbrs.Add(edge_other_v(eid, vid));
    //	return vnbrs;
    //}


    void set_edge_vertices(int eID, int a, int b)
    {
        int i = 4 * eID;
        mEdges[i] = std::min(a, b);
        mEdges[i + 1] = std::max(a, b);
    }

    void set_edge_triangles(int eID, int t0, int t1)
    {
        int i = 4 * eID;
        mEdges[i + 2] = t0;
        mEdges[i + 3] = t1;
    }

    int replace_edge_vertex(int eID, int vOld, int vNew)
    {
        int i = 4 * eID;
        int a = mEdges[i], b = mEdges[i + 1];
        if (a == vOld)
        {
            mEdges[i] = std::min(b, vNew);
            mEdges[i + 1] = std::max(b, vNew);
            return 0;
        }
        else if (b == vOld)
        {
            mEdges[i] = std::min(a, vNew);
            mEdges[i + 1] = std::max(a, vNew);
            return 1;
        }
        else
            return -1;
    }


    int replace_edge_triangle(int eID, int tOld, int tNew)
    {
        int i = 4 * eID;
        int a = mEdges[i + 2], b = mEdges[i + 3];
        if (a == tOld) {
            if (tNew == InvalidID)
            {
                mEdges[i + 2] = b;
                mEdges[i + 3] = InvalidID;
            }
            else
                mEdges[i + 2] = tNew;
            return 0;
        }
        else if (b == tOld)
        {
            mEdges[i + 3] = tNew;
            return 1;
        }
        else
            return -1;
    }

    int replace_triangle_edge(int tID, int eOld, int eNew)
    {
        int i = 3 * tID;
        if (mTriangleEdges[i] == eOld)
        {
            mTriangleEdges[i] = eNew;
            return 0;
        }
        else if (mTriangleEdges[i + 1] == eOld)
        {
            mTriangleEdges[i + 1] = eNew;
            return 1;
        }
        else if (mTriangleEdges[i + 2] == eOld)
        {
            mTriangleEdges[i + 2] = eNew;
            return 2;
        }
        else
            return -1;
    }



public:
    // TODO make this work
    struct CompactInfo
    {
        std::map<int, int> MapV;
    };

    CompactInfo CompactCopy(const IMesh& copy, bool bNormals = true, bool bTangents = true, bool bColors = true, bool bUVs = true)
    {
        // TODO can't do until CompactInfo works
        //if ( copy.IsCompact() ) {
        //    Copy(copy, bNormals, bColors, bUVs);
        //    CompactInfo ci = CompactInfo() { MapV = IdentityIndexMap() };
        //    return ci;
        //}

        mVertices = IMemory::dvector<float>();
        mVertexEdges = IMemory::small_list_set();
        vertices_refcount = IMemory::refcount_vector();
        mTriangles = IMemory::dvector<int>();
        mTriangleEdges = IMemory::dvector<int>();
        triangles_refcount = IMemory::refcount_vector();
        mEdges = IMemory::dvector<int>();
        edges_refcount = IMemory::refcount_vector();
        max_group_id = 0;

        mNormals = IMemory::dvector<float>();
        mTangents = IMemory::dvector<float>();
        mColors = IMemory::dvector<float>();
        mUV = IMemory::dvector<float>();
        mTriangleGroups = IMemory::dvector<int>();

        // [TODO] if we ksome of these were dense we could copy directly...
        VertexInfo vinfo;
        std::vector<int> mapV; mapV.resize(copy.MaxVertexID());
        for ( int vid : VertexIndices() )
        {
            copy.GetVertex(vid, vinfo, bNormals, bTangents, bColors, bUVs);
            mapV[vid] = AppendVertex(vinfo);
        }

        // [TODO] would be much faster to explicitly copy triangle & edge data structures!!
        for ( int tid : TriangleIndices() )
        {
            Index3i t = copy.GetTriangle(tid);
            t = Index3i(mapV[t.x], mapV[t.y], mapV[t.z]);
            int g = (copy.HasTriangleGroups()) ? copy.GetTriangleGroup(tid) : InvalidID;
            AppendTriangle(t, g);
            max_group_id = std::max(max_group_id, g+1);
        }

        //return CompactInfo() {
        //    MapV = IndexMap(mapV, this.MaxVertexID)
        //};
        return CompactInfo();
    }


    void Copy(const IMesh& copy, bool bNormals = true, bool bTangents = true, bool bColors = true, bool bUVs = true)
    {
        //mTransformMatrix = copy.mTransformMatrix;

        mVertices = IMemory::dvector<float>(copy.mVertices);

        mNormals = (bNormals && copy.HasVertexNormals()) ? IMemory::dvector<float>(copy.mNormals) : IMemory::dvector<float>();
        mTangents = (bTangents && copy.HasVertexTangents()) ? IMemory::dvector<float>(copy.mTangents) : IMemory::dvector<float>();
        mColors = (bColors && copy.HasVertexColors()) ? IMemory::dvector<float>(copy.mColors) : IMemory::dvector<float>();
        mUV = (bUVs && copy.HasVertexUVs()) ? IMemory::dvector<float>(copy.mUV) : IMemory::dvector<float>();

        vertices_refcount = IMemory::refcount_vector(copy.vertices_refcount);

        mVertexEdges = IMemory::small_list_set(copy.mVertexEdges);

        mTriangles = IMemory::dvector<int>(copy.mTriangles);
        mTriangleEdges = IMemory::dvector<int>(copy.mTriangleEdges);
        triangles_refcount = IMemory::refcount_vector(copy.triangles_refcount);
        if (copy.HasTriangleGroups())
            mTriangleGroups = IMemory::dvector<int>(copy.mTriangleGroups);
        max_group_id = copy.max_group_id;

        mEdges = IMemory::dvector<int>(copy.mEdges);
        edges_refcount = IMemory::refcount_vector(copy.edges_refcount);
    }


    /// <summary>
    /// Copy IMesh into this mesh. Currently always compacts.
    /// [TODO] if we get dense hint, we could be smarter w/ vertex map, etc
    /// </summary>
    //CompactInfo Copy(IMesh copy, MeshHints hints, bool bNormals = true, bool bColors = true, bool bUVs = true)
    //{
    //    mVertices = IMemory::dvector<float>();
    //    mVertexEdges = small_list_set();
    //    vertices_refcount = refcount_vector();
    //    triangles = IMemory::dvector<int>();
    //    mTriangleEdges = IMemory::dvector<int>();
    //    mTriangles_refcount = refcount_vector();
    //    edges = IMemory::dvector<int>();
    //    edges_refcount = refcount_vector();
    //    max_group_id = 0;

    //    normals = (bNormals && copy.HasVertexNormals) ? IMemory::dvector<float>() : nullptr;
    //    colors = (bColors && copy.HasVertexColors) ? IMemory::dvector<float>() : nullptr;
    //    uv = (bUVs && copy.HasVertexUVs) ? IMemory::dvector<float>() : nullptr;
    //    mTriangleGroups = (copy.HasTriangleGroups) ? IMemory::dvector<int>() : nullptr;


    //    // [TODO] if we ksome of these were dense we could copy directly...

    //    NewVertexInfo vinfo = NewVertexInfo();
    //    int[] mapV = int[copy.MaxVertexID];
    //    foreach (int vid in copy.VertexIndices()) {
    //        vinfo = copy.GetVertexAll(vid);
    //        mapV[vid] = AppendVertex(vinfo);
    //    }

    //    // [TODO] would be much faster to explicitly copy triangle & edge data structures!!

    //    foreach (int tid in copy.TriangleIndices()) {
    //        Index3i t = copy.GetTriangle(tid);
    //        t.a = mapV[t.a]; t.b = mapV[t.b]; t.c = mapV[t.c];
    //        int g = (copy.HasTriangleGroups) ? copy.GetTriangleGroup(tid) : InvalidID;
    //        AppendTriangle(t, g);
    //        max_group_id = std::max(max_group_id, g + 1);
    //    }

    //    return CompactInfo() {
    //        MapV = IndexMap(mapV, this.MaxVertexID)
    //    };
    //}


protected:
    void updateTimeStamp(bool bShapeChange)
    {
        timestamp++;
        if (bShapeChange)
            shape_timestamp++;
    }


public:
    /// <summary>
    /// Timestamp is incremented any time any change is made to the mesh
    /// </summary>
    int Timestamp() const
    {
        return timestamp;
    }

    /// <summary>
    /// ShapeTimestamp is incremented any time any vertex position is changed or the mesh topology is modified
    /// </summary>
    int ShapeTimestamp() const
    {
        return shape_timestamp;
    }


    // IMesh impl

    int VertexCount() const
    {
        return (int)vertices_refcount.count();
    }

    int TriangleCount() const
    {
        return (int)triangles_refcount.count();
    }

    int EdgeCount() const
    {
        return (int)edges_refcount.count();
    }

    // these values are (max_used+1), ie so an iteration should be < MaxTriangleID, not <=
    int MaxVertexID() const
    {
        return (int)vertices_refcount.max_index();
    }

    int MaxTriangleID() const
    {
        return (int)triangles_refcount.max_index();
    }

    int MaxEdgeID() const
    {
        return (int)edges_refcount.max_index();
    }

    int MaxGroupID() const
    {
        return max_group_id;
    }

    bool HasVertexColors() const { return mColors.size() ==  mVertices .size(); }
    bool HasVertexNormals() const { return mNormals.size() ==  mVertices.size(); }
    bool HasVertexTangents() const { return mTangents.size() ==  mVertices.size(); }
    bool HasVertexUVs() const { return mUV.size() / 2 ==  mVertices .size() / 3; }
    bool HasTriangleGroups() const { return mTriangleGroups.size() == mTriangles.size() / 3; }

    int Components() const
    {
        int c = 0;
        if (HasVertexNormals()) c |= (int)MeshComponents::VertexNormals;
        if (HasVertexColors()) c |= (int)MeshComponents::VertexColors;
        if (HasVertexUVs()) c |= (int)MeshComponents::VertexUVs;
        if (HasTriangleGroups()) c |= (int)MeshComponents::FaceGroups;
        return c;
    }

    // info

    bool IsVertex(int vID) const
    {
        return vertices_refcount.isValid(vID);
    }

    bool IsTriangle(int tID) const
    {
        return triangles_refcount.isValid(tID);
    }

    bool IsEdge(int eID) const
    {
        return edges_refcount.isValid(eID);
    }

protected:

    void debug_check_is_vertex(int v) const
    {
        //assert(IsVertex(v));
    }

    void debug_check_is_triangle(int t) const
    {
        //assert(IsTriangle(t));
    }

    void debug_check_is_edge(int e) const
    {
        //assert(IsEdge(e));
    }

public:



    std::vector<Vector3> GenerateSTLVertices()
    {
        std::vector<Vector3> vertices(VertexCount());
        for(int i=0; i<VertexCount(); ++i)
        {
            vertices[i] = GetVertex(i);
        }
        return vertices;
    }



    // getters
    Vector3 GetVertex(int vID) const
    {
        debug_check_is_vertex(vID);
        int i = 3 * vID;
        return Vector3( mVertices [i], mVertices[i + 1], mVertices[i + 2]);
    }


    Vector3 GetVertexf(int vID) const
    {
        debug_check_is_vertex(vID);
        int i = 3 * vID;
        return Vector3((float)mVertices[i], (float)mVertices[i + 1], (float)mVertices[i + 2]);
    }

    void SetVertex(int vID, const Vector3 & vNewPos)
    {
        //assert(IsFinite(vNewPos));
        debug_check_is_vertex(vID);

        int i = 3*vID;
        mVertices[i] = vNewPos.x;
        mVertices[i+1] = vNewPos.y;
        mVertices[i+2] = vNewPos.z;
        updateTimeStamp(true);
    }

    Vector3 GetVertexNormal(int vID) const
    {
        if (HasVertexNormals() == false)
            return Vector3::Y;

        debug_check_is_vertex(vID);
        int i = 3 * vID;
        return Vector3(mNormals[i], mNormals[i + 1], mNormals[i + 2]);
    }

    void SetVertexNormal(int vID, Vector3 vNewNormal)
    {
        if ( HasVertexNormals() )
        {
            debug_check_is_vertex(vID);
            int i = 3*vID;
            mNormals[i] = vNewNormal.x; mNormals[i+1] = vNewNormal.y; mNormals[i+2] = vNewNormal.z;
            updateTimeStamp(false);
        }
    }


    Vector3 GetVertexTangents(int vID) const
    {
        if (HasVertexTangents() == false)
            return Vector3::Y;

        debug_check_is_vertex(vID);
        int i = 3 * vID;
        return Vector3(mTangents[i], mTangents[i + 1], mTangents[i + 2]);
    }


    void SetVertexTangents(int vID, Vector3 vNewTangents)
    {
        if ( HasVertexNormals() )
        {
            debug_check_is_vertex(vID);
            int i = 3*vID;
            mTangents[i] = vNewTangents.x; mTangents[i+1] = vNewTangents.y; mTangents[i+2] = vNewTangents.z;
            updateTimeStamp(false);
        }
    }



    Vector3 GetVertexColor(int vID) const
    {
        if (HasVertexColors() == false)
            return Vector3::IDENTITY;
        debug_check_is_vertex(vID);
        int i = 3 * vID;
        return Vector3(mColors[i], mColors[i + 1], mColors[i + 2]);
    }

    void SetVertexColor(int vID, Vector3 vNewColor)
    {
        if ( HasVertexColors() )
        {
            debug_check_is_vertex(vID);
            int i = 3*vID;
            mColors[i] = vNewColor.x; mColors[i+1] = vNewColor.y; mColors[i+2] = vNewColor.z;
            updateTimeStamp(false);
        }
    }

    Vector2 GetVertexUV(int vID) const
    {
        if (HasVertexUVs() == false)
            return Vector2::ZERO;
        debug_check_is_vertex(vID);
        int i = 2 * vID;
        return Vector2(mUV[i], mUV[i + 1]);
    }

    void SetVertexUV(int vID, Vector2 vNewUV)
    {
        if ( HasVertexUVs() )
        {
            debug_check_is_vertex(vID);
            int i = 2*vID;
            mUV[i] = vNewUV.x;
            mUV[i+1] = vNewUV.y;
            updateTimeStamp(false);
        }
    }

    bool GetVertex(int vID, VertexInfo& vinfo, bool bWantNormals, bool bWantTangents, bool bWantColors, bool bWantUVs) const
    {
        if (vertices_refcount.isValid(vID) == false)
            return false;
        vinfo.v = Vector3(mVertices[3 * vID], mVertices[3 * vID + 1], mVertices[3 * vID + 2]);
        vinfo.bHaveN = vinfo.bHaveUV = vinfo.bHaveC = false;
        if (HasVertexNormals() && bWantNormals)
        {
            vinfo.bHaveN = true;
            vinfo.n = Vector3(mNormals[3 * vID], mNormals[3 * vID + 1], mNormals[3 * vID + 2]);
        }

        if (HasVertexTangents() && bWantTangents)
        {
            vinfo.bHaveN = true;
            vinfo.n = Vector3(mNormals[3 * vID], mNormals[3 * vID + 1], mNormals[3 * vID + 2]);
        }

        if (HasVertexColors() && bWantColors)
        {
            vinfo.bHaveC = true;
            vinfo.c = Vector3(mColors[3 * vID], mColors[3 * vID + 1], mColors[3 * vID + 2]);
        }

        if (HasVertexUVs() && bWantUVs)
        {
            vinfo.bHaveUV = true;
            vinfo.uv = Vector2(mUV[2 * vID], mUV[2 * vID + 1]);
        }

        return true;
    }

    int GetVtxEdgeCount(int vID) const
    {
        return vertices_refcount.isValid(vID) ? mVertexEdges.Count(vID) : -1;
    }

    int GetMaxVtxEdgeCount() const
    {
        int max = 0;
        for ( int vid : VertexIndices() )
            max = std::max(max, mVertexEdges.Count(vid));
        return max;
    }

    VertexInfo GetVertexAll(int i) const
    {
        VertexInfo vi = VertexInfo();
        vi.v = GetVertex(i);
        if ( HasVertexNormals() )
        {
            vi.bHaveN = true;
            vi.n = GetVertexNormal(i);
        }
        else
            vi.bHaveN = false;
        if ( HasVertexColors())
        {
            vi.bHaveC = true;
            vi.c = GetVertexColor(i);
        }
        else
            vi.bHaveC = false;
        if ( HasVertexUVs())
        {
            vi.bHaveUV = true;
            vi.uv = GetVertexUV(i);
        }
        else
            vi.bHaveUV = false;
        return vi;
    }


//    /// <summary>
//    /// Compute a normal/tangent frame at vertex that is "stable" as long as
//    /// the mesh topology doesn't change, meaning that one axis of the frame
//    /// will be computed from projection of outgoing edge.
//    /// Requires that vertex normals are available.
//    /// by default, frame.Z is normal, and .X points along mesh edge
//    /// if bFrameNormalY, then frame.Y is normal (X still points along mesh edge)
//    /// </summary>
//    Frame3d GetVertexFrame(int vID, bool bFrameNormalY = false) const
//    {
//        assert(HasVertexNormals());

//        int vi = 3 * vID;
//        Vector3 v(vertices[vi], vertices[vi + 1], vertices[vi + 2]);
//        Vector3 normal(normals[vi], normals[vi + 1], normals[vi + 2]);
//        int eid = mVertexEdges.First(vID);
//        int ovi = 3 * edge_other_v(eid, vID);
//        Vector3 ov(vertices[ovi], vertices[ovi + 1], vertices[ovi + 2]);
//        Vector3 edge = (ov - v);
//        edge.normalize();

//        Vector3 other = normal.cross(edge);
//        edge = other.cross(normal);
//        if (bFrameNormalY)
//            return Frame3d(v, edge, normal, -other);
//        else
//            return Frame3d(v, edge, other, normal);
//    }




    Index3i GetTriangle(int tID) const
    {
        debug_check_is_triangle(tID);
        int i = 3 * tID;
        return Index3i(mTriangles[i], mTriangles[i + 1], mTriangles[i + 2]);
    }

    Index3i GetTriEdges(int tID) const
    {
        debug_check_is_triangle(tID);
        int i = 3 * tID;
        return Index3i(mTriangleEdges[i], mTriangleEdges[i + 1], mTriangleEdges[i + 2]);
    }

    int GetTriEdge(int tid, int j) const
    {
        debug_check_is_triangle(tid);
        return mTriangleEdges[3*tid+j];
    }


    Index3i GetTriNeighbourTris(int tID) const
    {
        if (triangles_refcount.isValid(tID))
        {
            int tei = 3 * tID;
            Index3i nbr_t = Index3i::ZERO;
            for (int j = 0; j < 3; ++j)
            {
                int ei = 4 * mTriangleEdges[tei + j];
                nbr_t[j] = (mEdges[ei + 2] == tID) ? mEdges[ei + 3] : mEdges[ei + 2];
            }
            return nbr_t;
        } else
            return InvalidTriangle();
    }

    //IEnumerable<int> TriTrianglesItr(int tID) {
    //    if (triangles_refcount.isValid(tID)) {
    //        int tei = 3 * tID;
    //        for (int j = 0; j < 3; ++j) {
    //            int ei = 4 * mTriangleEdges[tei + j];
    //            int nbr_t = (mEdges[ei + 2] == tID) ? mEdges[ei + 3] : mEdges[ei + 2];
    //            if (nbr_t != DMesh3.InvalidID)
    //                yield return nbr_t;
    //        }
    //    }
    //}



    int GetTriangleGroup(int tID) const
    {
        return (HasTriangleGroups()) ? -1
            : ( triangles_refcount.isValid(tID) ? mTriangleGroups[tID] : 0 );
    }

    void SetTriangleGroup(int tid, int group_id)
    {
        if (HasTriangleGroups())
        {
            debug_check_is_triangle(tid);
            mTriangleGroups[tid] = group_id;
            max_group_id = std::max(max_group_id, group_id+1);
            updateTimeStamp(false);
        }
    }

    int AllocateTriangleGroup()
    {
        return ++max_group_id;
    }


    void GetTriVertices(int tID, Vector3 & v0, Vector3 & v1, Vector3 & v2) const
    {
        int ai = 3 * mTriangles[3 * tID];
        v0.x = mVertices[ai];
        v0.y = mVertices[ai + 1];
        v0.z = mVertices[ai + 2];

        int bi = 3 * mTriangles[3 * tID + 1];
        v1.x = mVertices[bi];
        v1.y = mVertices[bi + 1];
        v1.z = mVertices[bi + 2];

        int ci = 3 * mTriangles[3 * tID + 2];
        v2.x = mVertices[ci];
        v2.y = mVertices[ci + 1];
        v2.z = mVertices[ci + 2];
    }

    Vector3 GetTriVertex(int tid, int j) const
    {
        int a = mTriangles[3 * tid + j];
        return Vector3(mVertices[3 * a], mVertices[3 * a + 1], mVertices[3 * a + 2]);
    }

    Vector3 GetTriBaryPoint(int tID, float bary0, float bary1, float bary2) const
    {
        int ai = 3 * mTriangles[3 * tID],
            bi = 3 * mTriangles[3 * tID + 1],
            ci = 3 * mTriangles[3 * tID + 2];
        return Vector3(
            (bary0*mVertices[ai] + bary1*mVertices[bi] + bary2*mVertices[ci]),
            (bary0*mVertices[ai + 1] + bary1*mVertices[bi + 1] + bary2*mVertices[ci + 1]),
            (bary0*mVertices[ai + 2] + bary1*mVertices[bi + 2] + bary2*mVertices[ci + 2]));
    }

    Vector3 GetTriNormal(int tID) const
    {
        Vector3 v0, v1, v2;
        GetTriVertices(tID, v0, v1, v2);
        return Vector3::triNormal(v0, v1, v2);
    }

    float GetTriArea(int tID) const
    {
        Vector3 v0, v1, v2;
        GetTriVertices(tID, v0, v1, v2);
        return Vector3::Area(v0, v1, v2);
    }

    /// <summary>
    /// Compute triangle normal, area, and centroid all at once. Re-uses vertex
    /// lookups and computes normal & area simultaneously. *However* does not produce
    /// the same normal/area as separate calls, because of this.
    /// </summary>
    void GetTriInfo(int tID, Vector3 & normal, float & fArea, Vector3 & vCentroid) const
    {
        Vector3 v0, v1, v2;
        GetTriVertices(tID, v0, v1, v2);
        vCentroid = (1.0 / 3.0) * (v0 + v1 + v2);
        fArea = Vector3::Area(v0, v1, v2);
        normal = Vector3::triNormal(v0, v1, v2);
        //normal = FastNormalArea(ref v0, ref v1, ref v2, out fArea);
    }


    /// <summary>
    /// interpolate vertex normals of triangle using barycentric coordinates
    /// </summary>
    Vector3 GetTriBaryNormal(int tID, float bary0, float bary1, float bary2)
    {
        int ai = 3 * mTriangles[3 * tID],
            bi = 3 * mTriangles[3 * tID + 1],
            ci = 3 * mTriangles[3 * tID + 2];
        Vector3 n = Vector3(
            (bary0*mNormals[ai] + bary1*mNormals[bi] + bary2*mNormals[ci]),
            (bary0*mNormals[ai + 1] + bary1*mNormals[bi + 1] + bary2*mNormals[ci + 1]),
            (bary0*mNormals[ai + 2] + bary1*mNormals[bi + 2] + bary2*mNormals[ci + 2]));
        n.Normalize();
        return n;
    }

    /// <summary>
    /// efficiently compute centroid of triangle
    /// </summary>
    Vector3 GetTriCentroid(int tID) const
    {
        int ai = 3 * mTriangles[3 * tID],
            bi = 3 * mTriangles[3 * tID + 1],
            ci = 3 * mTriangles[3 * tID + 2];
        float f = (1.0 / 3.0);
        return Vector3(
            (mVertices[ai] + mVertices[bi] + mVertices[ci]) * f,
            (mVertices[ai + 1] + mVertices[bi + 1] + mVertices[ci + 1]) * f,
            (mVertices[ai + 2] + mVertices[bi + 2] + mVertices[ci + 2]) * f );
    }


    /// <summary>
    /// Compute interpolated vertex attributes at point of triangle
    /// </summary>
    void GetTriBaryPoint(int tID, float bary0, float bary1, float bary2, VertexInfo & vinfo)
    {
        vinfo = VertexInfo();
        int ai = 3 * mTriangles[3 * tID],
            bi = 3 * mTriangles[3 * tID + 1],
            ci = 3 * mTriangles[3 * tID + 2];

        vinfo.v = Vector3(
            (bary0 * mVertices[ai] + bary1 * mVertices[bi] + bary2 * mVertices[ci]),
            (bary0 * mVertices[ai + 1] + bary1 * mVertices[bi + 1] + bary2 * mVertices[ci + 1]),
            (bary0 * mVertices[ai + 2] + bary1 * mVertices[bi + 2] + bary2 * mVertices[ci + 2]));

        vinfo.bHaveN = HasVertexNormals();
        if (vinfo.bHaveN) {
            vinfo.n = Vector3(
                        (float)(bary0 * mNormals[ai] + bary1 * mNormals[bi] + bary2 * mNormals[ci]),
                        (float)(bary0 * mNormals[ai + 1] + bary1 * mNormals[bi + 1] + bary2 * mNormals[ci + 1]),
                        (float)(bary0 * mNormals[ai + 2] + bary1 * mNormals[bi + 2] + bary2 * mNormals[ci + 2]));
            vinfo.n.Normalize();
        }

        vinfo.bHaveT = HasVertexTangents();
        if (vinfo.bHaveT) {
            vinfo.t = Vector3(
                (float)(bary0 * mTangents[ai] + bary1 * mTangents[bi] + bary2 * mTangents[ci]),
                (float)(bary0 * mTangents[ai + 1] + bary1 * mTangents[bi + 1] + bary2 * mTangents[ci + 1]),
                (float)(bary0 * mTangents[ai + 2] + bary1 * mTangents[bi + 2] + bary2 * mTangents[ci + 2]));
            vinfo.t.Normalize();
        }

        vinfo.bHaveC = HasVertexColors();
        if (vinfo.bHaveC) {
            vinfo.c = Vector3(
                (float)(bary0 * mColors[ai] + bary1 * mColors[bi] + bary2 * mColors[ci]),
                (float)(bary0 * mColors[ai + 1] + bary1 * mColors[bi + 1] + bary2 * mColors[ci + 1]),
                (float)(bary0 * mColors[ai + 2] + bary1 * mColors[bi + 2] + bary2 * mColors[ci + 2]));
        }

        vinfo.bHaveUV = HasVertexUVs();
        if (vinfo.bHaveUV) {
            ai = 2 * mTriangles[3 * tID];
            bi = 2 * mTriangles[3 * tID + 1];
            ci = 2 * mTriangles[3 * tID + 2];
            vinfo.uv = Vector2(
                (float)(bary0 * mUV[ai] + bary1 * mUV[bi] + bary2 * mUV[ci]),
                (float)(bary0 * mUV[ai + 1] + bary1 * mUV[bi + 1] + bary2 * mUV[ci + 1]));
        }
    }


//    /// <summary>
//    /// construct bounding box of triangle as efficiently as possible
//    /// </summary>
//    AxisAlignedBox3d GetTriBounds(int tID) const
//    {
//        int vi = 3 * triangles[3 * tID];
//        float x = vertices[vi], y = vertices[vi + 1], z = vertices[vi + 2];
//        float minx = x, maxx = x, miny = y, maxy = y, minz = z, maxz = z;
//        for (int i = 1; i < 3; ++i) {
//            vi = 3 * triangles[3 * tID + i];
//            x = vertices[vi]; y = vertices[vi + 1]; z = vertices[vi + 2];
//            if (x < minx) minx = x; else if (x > maxx) maxx = x;
//            if (y < miny) miny = y; else if (y > maxy) maxy = y;
//            if (z < minz) minz = z; else if (z > maxz) maxz = z;
//        }
//        return AxisAlignedBox3d(Wml::Vector3(minx, miny, minz), Wml::Vector3(maxx, maxy, maxz));
//    }


//    /// <summary>
//    /// Construct stable frame at triangle centroid, where frame.Z is face normal,
//    /// and frame.X is aligned with edge nEdge of triangle.
//    /// </summary>
//    Frame3d GetTriFrame(int tID, int nEdge = 0)
//    {
//        int ti = 3 * tID;
//        int a = 3 * triangles[ti + (nEdge % 3)];
//        int b = 3 * triangles[ti + ((nEdge+1) % 3)];
//        int c = 3 * triangles[ti + ((nEdge+2) % 3)];
//        Vector3 v1(vertices[a], vertices[a + 1], vertices[a + 2]);
//        Vector3 v2(vertices[b], vertices[b + 1], vertices[b + 2]);
//        Vector3 v3(vertices[c], vertices[c + 1], vertices[c + 2]);

//        Vector3 edge1 = v2 - v1;  edge1.normalize();
//        Vector3 edge2 = v3 - v2;  edge2.normalize();
//        Vector3 normal = edge1.cross(edge2); normal.normalize();

//        Vector3 other = normal.cross(edge1);

//        Vector3 center = (v1 + v2 + v3) / 3;
//        return Frame3d(center, edge1, other, normal);
//    }



    /// <summary>
    /// compute solid angle of oriented triangle tID relative to point p - see WindingNumber()
    /// </summary>
    float GetTriSolidAngle(int tID, const Vector3 & p) const
    {
        int ti = 3 * tID;
        int ta = 3 * mTriangles[ti];
        Vector3 a(mVertices[ta] - p.x, mVertices[ta + 1] - p.y, mVertices[ta + 2] - p.z);
        int tb = 3 * mTriangles[ti + 1];
        Vector3 b(mVertices[tb] - p.x, mVertices[tb + 1] - p.y, mVertices[tb + 2] - p.z);
        int tc = 3 * mTriangles[ti + 2];
        Vector3 c(mVertices[tc] - p.x, mVertices[tc + 1] - p.y, mVertices[tc + 2] - p.z);
        // note: top and bottom are reversed here from formula in the paper? but it doesn't work otherwise...
        float la = a.Length(), lb = b.Length(), lc = c.Length();
        float bottom = (la * lb * lc) + a.Dot(b) * lc + b.Dot(c) * la + c.Dot(a) * lb;
        float top = a.x * (b.y * c.z - c.y * b.z) - a.y * (b.x * c.z - c.x * b.z) + a.z * (b.x * c.y - c.x * b.y);
        return 2.0 * atan2(top, bottom);
    }



//    /// <summary>
//    /// compute internal angle at vertex i of triangle (where i is 0,1,2);
//    /// TODO can be more efficient here, probably...
//    /// </summary>
//    float GetTriInternalAngleR(int tID, int i)
//    {
//        int ti = 3 * tID;
//        int ta = 3 * triangles[ti];
//        Vector3 a(mVertices[ta], mVertices[ta + 1], mVertices[ta + 2]);
//        int tb = 3 * triangles[ti + 1];
//        Vector3 b(mVertices[tb], mVertices[tb + 1], mVertices[tb + 2]);
//        int tc = 3 * triangles[ti + 2];
//        Vector3 c(mVertices[tc], mVertices[tc + 1], mVertices[tc + 2]);
//        if ( i == 0 )
//            return VectorAngleR( (b-a).Normalized(), (c-a).Normalized());
//        else if ( i == 1 )
//            return VectorAngleR( (a-b).Normalized(), (c-b).Normalized());
//        else
//            return VectorAngleR( (a-c).Normalized(), (b-c).Normalized());
//    }



    Index2i GetEdgeV(int eID) const
    {
        debug_check_is_edge(eID);
        int i = 4 * eID;
        return Index2i(mEdges[i], mEdges[i + 1]);
    }


    bool GetEdgeV(int eID, Vector3 & a, Vector3 & b) const
    {
        debug_check_is_edge(eID);
        int iv0 = 3 * mEdges[4 * eID];
        a.x = mVertices[iv0]; a.y = mVertices[iv0 + 1]; a.z = mVertices[iv0 + 2];
        int iv1 = 3 * mEdges[4 * eID + 1];
        b.x = mVertices[iv1]; b.y = mVertices[iv1 + 1]; b.z = mVertices[iv1 + 2];
        return true;
    }


    Index2i GetEdgeT(int eID) const
    {
        debug_check_is_edge(eID);
        int i = 4 * eID;
        return Index2i(mEdges[i + 2], mEdges[i + 3]);
    }

    /// <summary>
    /// return [v0,v1,t0,t1], or Index4i.Max if eid is invalid
    /// </summary>
    Index4i GetEdge(int eID) const
    {
        debug_check_is_edge(eID);
        int i = 4 * eID;
        return Index4i(mEdges[i], mEdges[i + 1], mEdges[i + 2], mEdges[i + 3]);
    }

    bool GetEdge(int eID, int & a, int & b, int & t0, int & t1) const
    {
        debug_check_is_edge(eID);
        int i = eID*4;
        a = mEdges[i]; b = mEdges[i+1]; t0 = mEdges[i+2]; t1 = mEdges[i+3];
        return true;
    }

    // return same indices as GetEdgeV, but oriented based on attached triangle
    Index2i GetOrientedBoundaryEdgeV(int eID) const
    {
        if ( edges_refcount.isValid(eID) )
        {
            int ei = 4 * eID;
            if ( mEdges[ei+3] == InvalidID)
            {
                int a = mEdges[ei], b = mEdges[ei + 1];
                int ti = 3 * mEdges[ei + 2];
                Index3i tri = Index3i(mTriangles[ti], mTriangles[ti + 1], mTriangles[ti + 2]);
                int ai = find_edge_index_in_tri(a, b, tri);
                return Index2i(tri[ai], tri[(ai + 1) % 3]);
            }
        }
        assert(false);
        return InvalidEdge();
    }


    // average of 1 or 2 face normals
    Vector3 GetEdgeNormal(int eID) const
    {
        if (edges_refcount.isValid(eID))
        {
            int ei = 4 * eID;
            Vector3 n = GetTriNormal(mEdges[ei + 2]);
            if (mEdges[ei + 3] != InvalidID)
            {
                n += GetTriNormal(mEdges[ei + 3]);
                n.Normalize();
            }
            return n;
        }
        assert(false);
        return Vector3::ZERO;
    }

    Vector3 GetEdgePoint(int eID, float t) const
    {
        if (edges_refcount.isValid(eID))
        {
            int ei = 4 * eID;
            int iv0 = 3 * mEdges[ei];
            int iv1 = 3 * mEdges[ei + 1];
            float mt = 1.0 - t;
            return Vector3(
                mt*mVertices[iv0] + t*mVertices[iv1],
                mt*mVertices[iv0 + 1] + t*mVertices[iv1 + 1],
                mt*mVertices[iv0 + 2] + t*mVertices[iv1 + 2]);
        }
        assert(false);
        return Vector3::ZERO;
    }


    // mesh-building


    /// <summary>
    /// Append vertex at position, returns vid
    /// </summary>
    int AppendVertex(const Vector3 & v)
    {
        VertexInfo vinfo(v);
        return AppendVertex(vinfo);
    }

    /// <summary>
    /// Append vertex at position and other fields, returns vid
    /// </summary>
    int AppendVertex(const VertexInfo & info)
    {
        /// Calculate Axis Aligned Box ...
        mAxisAlignedBoundingBox.Insert(info.v);


        int vid = vertices_refcount.allocate();
        int i = 3*vid;
        mVertices.insertAt(info.v[2], i + 2);
        mVertices.insertAt(info.v[1], i + 1);
        mVertices.insertAt(info.v[0], i);

        if ( HasVertexNormals() )
        {
            Vector3 n = (info.bHaveN) ? info.n : Vector3::Y;
            mNormals.insertAt(n[2], i + 2);
            mNormals.insertAt(n[1], i + 1);
            mNormals.insertAt(n[0], i);
        }

        if ( HasVertexTangents() )
        {
            Vector3 t = (info.bHaveT) ? info.t : Vector3::Y;
            mTangents.insertAt(t[2], i + 2);
            mTangents.insertAt(t[1], i + 1);
            mTangents.insertAt(t[0], i);
        }

        if ( HasVertexColors() )
        {
            Vector3 c = (info.bHaveC) ? info.c : Vector3::IDENTITY;
            mColors.insertAt(c[2], i + 2);
            mColors.insertAt(c[1], i + 1);
            mColors.insertAt(c[0], i);
        }

        if ( HasVertexUVs() || true )
        {
            Vector2 u = (info.bHaveUV) ? info.uv : Vector2::ZERO;
            int j = 2*vid;

            mUV.insertAt(u[1], j + 1);
            mUV.insertAt(u[0], j);
        }

        allocate_edges_list(vid);

        updateTimeStamp(true);
        return vid;
    }


    /// <summary>
    /// copy vertex fromVID from existing source mesh, returns vid
    /// </summary>
    int AppendVertex(IMesh from, int fromVID)
    {
        int bi = 3 * fromVID;

        int vid = vertices_refcount.allocate();
        int i = 3*vid;
        mVertices.insertAt(from.mVertices[bi+2], i + 2);
        mVertices.insertAt(from.mVertices[bi+1], i + 1);
        mVertices.insertAt(from.mVertices[bi], i);

        if ( HasVertexNormals() )
        {
            if ( from.HasVertexNormals() )
            {
                mNormals.insertAt(from.mNormals[bi + 2], i + 2);
                mNormals.insertAt(from.mNormals[bi + 1], i + 1);
                mNormals.insertAt(from.mNormals[bi], i);
            }
            else
            {
                mNormals.insertAt(0, i + 2);
                mNormals.insertAt(1, i + 1);       // y-up
                mNormals.insertAt(0, i);
            }
        }


        if ( HasVertexTangents() )
        {
            if ( from.HasVertexTangents() )
            {
                mTangents.insertAt(from.mTangents[bi + 2], i + 2);
                mTangents.insertAt(from.mTangents[bi + 1], i + 1);
                mTangents.insertAt(from.mTangents[bi], i);
            }
            else
            {
                mTangents.insertAt(0, i + 2);
                mTangents.insertAt(1, i + 1);       // y-up
                mTangents.insertAt(0, i);
            }
        }


        if ( HasVertexColors() )
        {
            if (from.HasVertexColors())
            {
                mColors.insertAt(from.mColors[bi + 2], i + 2);
                mColors.insertAt(from.mColors[bi + 1], i + 1);
                mColors.insertAt(from.mColors[bi], i);
            }
            else
            {
                mColors.insertAt(1, i + 2);
                mColors.insertAt(1, i + 1);       // white
                mColors.insertAt(1, i);
            }
        }

        if ( HasVertexUVs() )
        {
            int j = 2*vid;
            if (from.HasVertexUVs())
            {
                int bj = 2 * fromVID;
                mUV.insertAt(from.mUV[bj + 1], j + 1);
                mUV.insertAt(from.mUV[bj], j);
            }
            else
            {
                mUV.insertAt(0, j + 1);
                mUV.insertAt(0, j);
            }
        }

        allocate_edges_list(vid);

        updateTimeStamp(true);
        return vid;
    }



    /// <summary>
    /// insert vertex at given index, assuming it is unused
    /// If bUnsafe, we use fast id allocation that does not update free list.
    /// You should only be using this between BeginUnsafeVerticesInsert() / EndUnsafeVerticesInsert() calls
    /// </summary>
    MeshResult InsertVertex(int vid, const VertexInfo & info, bool bUnsafe = false)
    {
        if (vertices_refcount.isValid(vid))
            return MeshResult::Failed_VertexAlreadyExists;

        bool bOK = (bUnsafe) ? vertices_refcount.allocate_at_unsafe(vid) :
                               vertices_refcount.allocate_at(vid);
        if (bOK == false)
            return MeshResult::Failed_CannotAllocateVertex;

        int i = 3 * vid;
        mVertices.insertAt(info.v[2], i + 2);
        mVertices.insertAt(info.v[1], i + 1);
        mVertices.insertAt(info.v[0], i);

        if (HasVertexNormals())
        {
            Vector3 n = (info.bHaveN) ? info.n : Vector3::Y;
            mNormals.insertAt(n[2], i + 2);
            mNormals.insertAt(n[1], i + 1);
            mNormals.insertAt(n[0], i);
        }


        if (HasVertexTangents())
        {
            Vector3 t = (info.bHaveT) ? info.t : Vector3::Y;
            mTangents.insertAt(t[2], i + 2);
            mTangents.insertAt(t[1], i + 1);
            mTangents.insertAt(t[0], i);
        }


        if (HasVertexColors())
        {
            Vector3 c = (info.bHaveC) ? info.c : Vector3::IDENTITY;
            mColors.insertAt(c[2], i + 2);
            mColors.insertAt(c[1], i + 1);
            mColors.insertAt(c[0], i);
        }

        if (HasVertexUVs())
        {
            Vector2 u = (info.bHaveUV) ? info.uv : Vector2::ZERO;
            int j = 2 * vid;
            mUV.insertAt(u[1], j + 1);
            mUV.insertAt(u[0], j);
        }

        allocate_edges_list(vid);

        updateTimeStamp(true);
        return MeshResult::Ok;
    }


    //========================================================================//

    bool IntersectionRaycast(const IRay &ray , const Matrix4& transform , IRaycast* _raycast = nullptr) const
    {

        //Matrix4 transform = GetTransformMatrixHierarchy();

        IRay local_ray(  ray.Origin    * transform.GetInverse(),
                         ray.Direction * transform.GetRotMatrix().GetInverse());


        if(!mAxisAlignedBoundingBox.TestRayIntersect(local_ray)) return false;

        for(std::size_t i = 0; i < mTriangles.size(); i+=3)
        {
            Vector3 a = GetVertex(mTriangles[i+0]);// * mTransformMatrix;
            Vector3 b = GetVertex(mTriangles[i+1]);// * mTransformMatrix;
            Vector3 c = GetVertex(mTriangles[i+2]);// * mTransformMatrix;

            scalar u,v,t;
            bool intersection = IRayIntersectionToTriangle(a,b,c,local_ray.Origin,local_ray.Direction,u,v,t);

            if(_raycast) _raycast->distance = u;

            if(intersection)
            {
                return true;
            }
        }

        return false;
    }


    //========================================================================//


    virtual void BeginUnsafeVerticesInsert()
    {
        // do nothing...
    }
    virtual void EndUnsafeVerticesInsert()
    {
        vertices_refcount.rebuild_free_list();
    }



    int AppendTriangle(int v0, int v1, int v2, int gid = -1)
    {
        return AppendTriangle(Index3i(v0, v1, v2), gid);
    }


    int AppendTriangle(Index3i tv, int gid = -1)
    {
        if (IsVertex(tv[0]) == false || IsVertex(tv[1]) == false || IsVertex(tv[2]) == false)
        {
           // assert(false);
            return InvalidID;
        }
        if (tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2])
        {
            //assert(false);
            return InvalidID;
        }

        // look up edges. if any already have two triangles, this would
        // create non-manifold geometry and so we do not allow it
        int e0 = find_edge(tv[0], tv[1]);
        int e1 = find_edge(tv[1], tv[2]);
        int e2 = find_edge(tv[2], tv[0]);
        if ((e0 != InvalidID && IsBoundaryEdge(e0) == false)
                || (e1 != InvalidID && IsBoundaryEdge(e1) == false)
                || (e2 != InvalidID && IsBoundaryEdge(e2) == false))
        {
            return NonManifoldID;
        }

        // now safe to insert triangle
        int tid = triangles_refcount.allocate();
        int i = 3*tid;
        mTriangles.insertAt(tv[2], i + 2);
        mTriangles.insertAt(tv[1], i + 1);
        mTriangles.insertAt(tv[0], i);
        if ( HasTriangleGroups() ) {
            mTriangleGroups.insertAt(gid, tid);
            max_group_id = std::max(max_group_id, gid+1);
        }

        // increment ref counts and update/create edges
        vertices_refcount.increment(tv[0]);
        vertices_refcount.increment(tv[1]);
        vertices_refcount.increment(tv[2]);

        add_tri_edge(tid, tv[0], tv[1], 0, e0);
        add_tri_edge(tid, tv[1], tv[2], 1, e1);
        add_tri_edge(tid, tv[2], tv[0], 2, e2);

        updateTimeStamp(true);
        return tid;
    }
    // helper fn for above, just makes code cleaner
    void add_tri_edge(int tid, int v0, int v1, int j, int eid)
    {
        if (eid != InvalidID)
        {
            mEdges[4 * eid + 3] = tid;
            mTriangleEdges.insertAt(eid, 3 * tid + j);
        }
        else
        {
            mTriangleEdges.insertAt(add_edge(v0, v1, tid), 3 * tid + j);
        }
    }





    /// <summary>
    /// Insert triangle at given index, assuming it is unused.
    /// If bUnsafe, we use fast id allocation that does not update free list.
    /// You should only be using this between BeginUnsafeTrianglesInsert() / EndUnsafeTrianglesInsert() calls
    /// </summary>
    MeshResult InsertTriangle(int tid, Index3i tv, int gid = -1, bool bUnsafe = false)
    {
        if (triangles_refcount.isValid(tid))
            return MeshResult::Failed_TriangleAlreadyExists;

        if (IsVertex(tv[0]) == false || IsVertex(tv[1]) == false || IsVertex(tv[2]) == false)
        {
            assert(false);
            return MeshResult::Failed_NotAVertex;
        }
        if (tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2])
        {
            assert(false);
            return MeshResult::Failed_InvalidNeighbourhood;
        }

        // look up edges. if any already have two triangles, this would
        // create non-manifold geometry and so we do not allow it
        int e0 = find_edge(tv[0], tv[1]);
        int e1 = find_edge(tv[1], tv[2]);
        int e2 = find_edge(tv[2], tv[0]);
        if ((e0 != InvalidID && IsBoundaryEdge(e0) == false)
                || (e1 != InvalidID && IsBoundaryEdge(e1) == false)
                || (e2 != InvalidID && IsBoundaryEdge(e2) == false))
        {
            return MeshResult::Failed_WouldCreateNonmanifoldEdge;
        }

        bool bOK = (bUnsafe) ? triangles_refcount.allocate_at_unsafe(tid) :
                                triangles_refcount.allocate_at(tid);
        if (bOK == false)
            return MeshResult::Failed_CannotAllocateTriangle;

        // now safe to insert triangle
        int i = 3 * tid;
        mTriangles.insertAt(tv[2], i + 2);
        mTriangles.insertAt(tv[1], i + 1);
        mTriangles.insertAt(tv[0], i);
        if (HasTriangleGroups())
        {
            mTriangleGroups.insertAt(gid, tid);
            max_group_id = std::max(max_group_id, gid + 1);
        }

        // increment ref counts and update/create edges
        vertices_refcount.increment(tv[0]);
        vertices_refcount.increment(tv[1]);
        vertices_refcount.increment(tv[2]);

        add_tri_edge(tid, tv[0], tv[1], 0, e0);
        add_tri_edge(tid, tv[1], tv[2], 1, e1);
        add_tri_edge(tid, tv[2], tv[0], 2, e2);

        updateTimeStamp(true);
        return MeshResult::Ok;
    }


    virtual void BeginUnsafeTrianglesInsert()
    {
        // do nothing...
    }
    virtual void EndUnsafeTrianglesInsert()
    {
        triangles_refcount.rebuild_free_list();
    }





    void EnableVertexNormals(Vector3 initial_normal)
    {
        if (HasVertexNormals())
            return;
        mNormals = IMemory::dvector<float>();
        int NV = MaxVertexID();
        mNormals.resize(3*NV);
        for (int i = 0; i < NV; ++i)
        {
            int vi = 3 * i;
            mNormals[vi] = initial_normal.x;
            mNormals[vi + 1] = initial_normal.y;
            mNormals[vi + 2] = initial_normal.z;
        }
    }
    void DiscardVertexNormals()
    {
        mNormals = IMemory::dvector<float>();
    }


    void EnableVertexTangents(Vector3 initial_normal)
    {
        if (HasVertexTangents())
            return;

        mTangents = IMemory::dvector<float>();
        int NV = MaxVertexID();
        mTangents.resize(3*NV);
        for (int i = 0; i < NV; ++i)
        {
            int vi = 3 * i;
            mTangents[vi] = initial_normal.x;
            mTangents[vi + 1] = initial_normal.y;
            mTangents[vi + 2] = initial_normal.z;
        }
    }
    void DiscardVertexTangents()
    {
        mTangents = IMemory::dvector<float>();
    }

    void EnableVertexColors(Vector3 initial_color)
    {
        if (HasVertexColors())
            return;
        mColors = IMemory::dvector<float>();
        int NV = MaxVertexID();
        mColors.resize(3*NV);
        for (int i = 0; i < NV; ++i)
        {
            int vi = 3 * i;
            mColors[vi] = initial_color.x;
            mColors[vi + 1] = initial_color.y;
            mColors[vi + 2] = initial_color.z;
        }
    }
    void DiscardVertexColors()
    {
        mColors = IMemory::dvector<float>();
    }

    void EnableVertexUVs(Vector2 initial_uv)
    {
        if (HasVertexUVs())
            return;
        mUV = IMemory::dvector<float>();
        int NV = MaxVertexID();
        mUV.resize(2*NV);
        for (int i = 0; i < NV; ++i)
        {
            int vi = 2 * i;
            mUV[vi] = initial_uv.x;
            mUV[vi + 1] = initial_uv.y;
        }
    }
    void DiscardVertexUVs()
    {
        mUV = IMemory::dvector<float>();
    }

    void EnableTriangleGroups(int initial_group = 0)
    {
        if (HasTriangleGroups())
            return;
        mTriangleGroups = IMemory::dvector<int>();
        int NT = MaxTriangleID();
        mTriangleGroups.resize(NT);
        for (int i = 0; i < NT; ++i)
            mTriangleGroups[i] = initial_group;
        max_group_id = 0;
    }


    void DiscardTriangleGroups()
    {
        mTriangleGroups = IMemory::dvector<int>();
        max_group_id = 0;
    }



    // iterators
    /// <summary> Enumerate boundary edges </summary>
    IMemory::refcount_vector::filtered_enumerable BoundaryEdgeIndices()
    {
        return edges_refcount.filtered_indices( [=](int eid)
        {
            return mEdges[4 * eid + 3] == InvalidID;
        });
    }

    template <typename T> using value_iteration = IMemory::refcount_vector::mapped_enumerable<T>;

    /// <summary> Enumerate vertices </summary>
    value_iteration<Vector3> Vertices() const
    {
        return vertices_refcount.mapped_indices<Vector3>( [=](int vid)
        {
            int i = 3 * vid;
            return Vector3(mVertices[i], mVertices[i + 1], mVertices[i + 2]);
        });
    }


    /// <summary> Enumerate triangles </summary>
    value_iteration<Index3i> Triangles() const
    {
        return triangles_refcount.mapped_indices<Index3i>( [=](int tid)
        {
            int i = 3 * tid;
            return Index3i(mTriangles[i], mTriangles[i + 1], mTriangles[i + 2]);
        });
    }

    /// <summary> Enumerate edges. return value is [v0,v1,t0,t1], where t1 will be InvalidID if this is a boundary edge </summary>
    value_iteration<Index4i> Edges() const
    {
        return edges_refcount.mapped_indices<Index4i>([=](int eid)
        {
            int i = 4 * eid;
            return Index4i(mEdges[i], mEdges[i + 1], mEdges[i + 2], mEdges[i + 3]);
        });
    }


    // queries

    /// <summary>
    /// Find edgeid for edge [a,b]
    /// </summary>
    int FindEdge(int vA, int vB) const
    {
        debug_check_is_vertex(vA);
        debug_check_is_vertex(vB);
        return find_edge(vA, vB);
    }

    /// <summary>
    /// Find edgeid for edge [a,b] from triangle that contains the edge.
    /// This is faster than FindEdge() because it is constant-time
    /// </summary>
    int FindEdgeFromTri(int vA, int vB, int tID) const
    {
        return find_edge_from_tri(vA, vB, tID);
    }

    /// <summary>
    /// If edge has vertices [a,b], and is connected two triangles [a,b,c] and [a,b,d],
    /// this returns [c,d], or [c,InvalidID] for a boundary edge
    /// </summary>
    Index2i GetEdgeOpposingV(int eID) const
    {
        // [TODO] there was a comment here saying this does more work than necessary??
        // ** it is important that verts returned maintain [c,d] order!!
        int i = 4*eID;
        int a = mEdges[i], b = mEdges[i + 1];
        int t0 = mEdges[i + 2], t1 = mEdges[i + 3];
        int c = find_tri_other_vtx(a, b, mTriangles, t0);
        if (t1 != InvalidID) {
            int d = find_tri_other_vtx(a, b, mTriangles, t1);
            return Index2i(c, d);
        } else
            return Index2i(c, InvalidID);
    }


    /// <summary>
    /// Find triangle made up of any permutation of vertices [a,b,c]
    /// </summary>
    int FindTriangle(int a, int b, int c) const
    {
        int eid = find_edge(a, b);
        if (eid == InvalidID)
            return InvalidID;
        int ei = 4 * eid;

        // triangles attached to edge [a,b] must contain verts a and b...
        int ti = 3 * mEdges[ei + 2];
        if (mTriangles[ti] == c || mTriangles[ti + 1] == c || mTriangles[ti + 2] == c )
            return mEdges[ei + 2];
        if (mEdges[ei + 3] != InvalidID) {
            ti = 3 * mEdges[ei + 3];
            if (mTriangles[ti] == c || mTriangles[ti + 1] == c || mTriangles[ti + 2] == c )
                return mEdges[ei + 3];
        }

        return InvalidID;
    }



    /// <summary>
    /// Enumerate "other" vertices of edges connected to vertex (ie vertex one-ring)
    /// </summary>
    IMemory::small_list_set::value_enumerable VtxVerticesItr(int vID) const
    {
        assert(vertices_refcount.isValid(vID));
        return mVertexEdges.values(vID, [vID, this](int eid) { return edge_other_v(eid, vID); } );
    }


    /// <summary>
    /// Enumerate edge ids connected to vertex (ie edge one-ring)
    /// </summary>
    IMemory::small_list_set::value_enumerable VtxEdgesItr(int vID) const
    {
        assert(vertices_refcount.isValid(vID));
        return mVertexEdges.values(vID);
    }


    /// <summary>
    /// Returns count of boundary edges at vertex, and
    /// the first two boundary edges if found.
    /// If return is > 2, call VtxAllBoundaryEdges
    /// </summary>
    int VtxBoundaryEdges(int vID, int & e0, int & e1) const
    {
        if ( vertices_refcount.isValid(vID) )
        {
            int count = 0;
            for (int eid : mVertexEdges.values(vID) )
            {
                int ei = 4 * eid;
                if ( mEdges[ei+3] == InvalidID )
                {
                    if (count == 0)
                        e0 = eid;
                    else if (count == 1)
                        e1 = eid;
                    count++;
                }
            }
            return count;
        }
        assert(false);
        return -1;
    }

    /// <summary>
    /// Find edge ids of boundary edges connected to vertex.
    /// e needs to be large enough (ie call VtxBoundaryEdges, or as large as max one-ring)
    /// returns count, ie number of elements of e that were filled
    /// </summary>
    int VtxAllBoundaryEdges(int vID, int * e) const
    {
        if (vertices_refcount.isValid(vID))
        {
            int count = 0;
            for (int eid : mVertexEdges.values(vID))
            {
                int ei = 4 * eid;
                if ( mEdges[ei+3] == InvalidID )
                    e[count++] = eid;
            }
            return count;
        }
        assert(false);
        return -1;
    }


    /// <summary>
    /// Get triangle one-ring at vertex.
    /// bUseOrientation is more efficient but returns incorrect result if vertex is a bowtie
    /// </summary>
    MeshResult GetVtxTriangles(int vID, std::vector<int> & vTriangles, bool bUseOrientation) const
    {
        if (!IsVertex(vID))
            return MeshResult::Failed_NotAVertex;

        if (bUseOrientation)
        {
            for (int eid : mVertexEdges.values(vID))
            {
                int vOther = edge_other_v(eid, vID);
                int i = 4*eid;
                int et0 = mEdges[i + 2];
                if (tri_has_sequential_v(et0, vID, vOther))
                    vTriangles.push_back(et0);
                int et1 = mEdges[i + 3];
                if (et1 != InvalidID && tri_has_sequential_v(et1, vID, vOther))
                    vTriangles.push_back(et1);
            }
        }
        else
        {
            // brute-force method
            for (int eid : mVertexEdges.values(vID))
            {
                int i = 4*eid;
                int t0 = mEdges[i + 2];
                if ( std::find(vTriangles.begin(), vTriangles.end(), t0) == vTriangles.end() )
                    vTriangles.push_back(t0);
                int t1 = mEdges[i + 3];
                if (t1 != InvalidID && std::find(vTriangles.begin(), vTriangles.end(), t1) == vTriangles.end())
                    vTriangles.push_back(t1);
            }
        }
        return MeshResult::Ok;
    }


    /// <summary>
    /// return # of triangles attached to vID, or -1 if invalid vertex
    /// if bBruteForce = true, explicitly checks, which creates a list and is expensive
    /// default is false, uses orientation, no memory allocation
    /// </summary>
    int GetVtxTriangleCount(int vID, bool bBruteForce = false) const
    {
        if ( bBruteForce )
        {
            std::vector<int> vTriangles;
            if (GetVtxTriangles(vID, vTriangles, false) != MeshResult::Ok)
                return -1;
            return (int)vTriangles.size();
        }

        if (!IsVertex(vID))
            return -1;
        int N = 0;
        for (int eid : mVertexEdges.values(vID))
        {
            int vOther = edge_other_v(eid, vID);
            int i = 4*eid;
            int et0 = mEdges[i + 2];
            if (tri_has_sequential_v(et0, vID, vOther))
                N++;
            int et1 = mEdges[i + 3];
            if (et1 != InvalidID && tri_has_sequential_v(et1, vID, vOther))
                N++;
        }
        return N;
    }


    using vtx_triangles_enumerable = IMemory::expand_enumerable<int,int, IMemory::small_list_set::value_iterator>;

    /// <summary>
    /// iterate over triangle IDs of vertex one-ring
    /// </summary>
    vtx_triangles_enumerable VtxTrianglesItr(int vID)
    {
        assert(vertices_refcount.isValid(vID));

        std::function<int(int, int&)> expand_f = [=](int eid, int & k)
        {
            int vOther = edge_other_v(eid, vID);
            int i = 4 * eid;
            if (k == -1) {
                int et0 = mEdges[i + 2];
                if (tri_has_sequential_v(et0, vID, vOther))
                {
                    k = 0;
                    return et0;
                }
            }

            if (k != 1)
            {
                int et1 = mEdges[i + 3];
                if (et1 != InvalidID && tri_has_sequential_v(et1, vID, vOther))
                {
                    k = 1;
                    return et1;
                }
            }
            k = -1;
            return -1;
        };

        return vtx_triangles_enumerable(mVertexEdges.values(vID), expand_f);
    }


    /// <summary>
    ///  from edge and vert, returns other vert, two opposing verts, and two triangles
    /// </summary>
    void GetVtxNbrhood(int eID, int vID, int & vOther, int & oppV1, int & oppV2, int & t1, int & t2) const
    {
        int i = 4*eID;
        vOther = (mEdges[i] == vID) ? mEdges[i+1] : mEdges[i];
        t1 = mEdges[i + 2];
        oppV1 = find_tri_other_vtx(vID, vOther, mTriangles, t1);
        t2 = mEdges[i + 3];
        if ( t2 != InvalidID )
            oppV2 = find_tri_other_vtx(vID, vOther, mTriangles, t2);
        else
            t2 = InvalidID;
    }


    /// <summary>
    /// Fastest possible one-ring centroid. This is used inside many other algorithms
    /// so it helps to have it be maximally efficient
    /// </summary>
    void VtxOneRingCentroid(int vID, Vector3 & centroid) const
    {
        centroid = Vector3::ZERO;
        if (vertices_refcount.isValid(vID))
        {
            int n = 0;
            for (int eid : mVertexEdges.values(vID))
            {
                int other_idx = 3 * edge_other_v(eid, vID);
                centroid.x +=mVertices[other_idx];
                centroid.y +=mVertices[other_idx + 1];
                centroid.z +=mVertices[other_idx + 2];
                n++;
            }
            if (n > 0)
            {
                centroid *= 1.0 / n;
            }
        }
    }




    ///=========================++++++++++++++++++++++++++++++++===============================///


protected:

    bool tri_has_v(int tID, int vID) const {
        int i = 3*tID;
        return mTriangles[i] == vID
            || mTriangles[i + 1] == vID
            || mTriangles[i + 2] == vID;
    }

    bool tri_is_boundary(int tID) const {
        int i = 3*tID;
        return IsBoundaryEdge(mTriangleEdges[i])
            || IsBoundaryEdge(mTriangleEdges[i + 1])
            || IsBoundaryEdge(mTriangleEdges[i + 2]);
    }

    bool tri_has_neighbour_t(int tCheck, int tNbr) const {
        int i = 3*tCheck;
        return edge_has_t(mTriangleEdges[i], tNbr)
            || edge_has_t(mTriangleEdges[i + 1], tNbr)
            || edge_has_t(mTriangleEdges[i + 2], tNbr);
    }

    bool tri_has_sequential_v(int tID, int vA, int vB) const
    {
        int i = 3*tID;
        int v0 = mTriangles[i], v1 = mTriangles[i + 1], v2 = mTriangles[i + 2];
        if (v0 == vA && v1 == vB) return true;
        if (v1 == vA && v2 == vB) return true;
        if (v2 == vA && v0 == vB) return true;
        return false;
    }

    //! returns edge ID
    int find_tri_neighbour_edge(int tID, int vA, int vB) const
    {
        int i = 3*tID;
        int tv0 = mTriangles[i], tv1 = mTriangles[i+1];
        if ( same_pair_unordered(tv0, tv1, vA, vB) ) return mTriangleEdges[3*tID];
        int tv2 = mTriangles[i+2];
        if ( same_pair_unordered(tv1, tv2, vA, vB) ) return mTriangleEdges[3*tID+1];
        if ( same_pair_unordered(tv2, tv0, vA, vB) ) return mTriangleEdges[3*tID+2];
        return InvalidID;
    }

    // returns 0/1/2
    int find_tri_neighbour_index(int tID, int vA, int vB) const
    {
        int i = 3*tID;
        int tv0 = mTriangles[i], tv1 = mTriangles[i+1];
        if ( same_pair_unordered(tv0, tv1, vA, vB) ) return 0;
        int tv2 = mTriangles[i+2];
        if ( same_pair_unordered(tv1, tv2, vA, vB) ) return 1;
        if ( same_pair_unordered(tv2, tv0, vA, vB) ) return 2;
        return InvalidID;
    }



    bool edge_has_v(int eid, int vid) const
    {
        int i = 4*eid;
        return (mEdges[i] == vid) || (mEdges[i + 1] == vid);
    }

    bool edge_has_t(int eid, int tid) const
    {
        int i = 4*eid;
        return (mEdges[i + 2] == tid) || (mEdges[i + 3] == tid);
    }

    int edge_other_v(int eID, int vID) const
    {
        int i = 4*eID;
        int ev0 = mEdges[i], ev1 = mEdges[i + 1];
        return (ev0 == vID) ? ev1 : ((ev1 == vID) ? ev0 : InvalidID);
    }

    int edge_other_t(int eID, int tid) const
    {
        int i = 4*eID;
        int et0 = mEdges[i + 2], et1 = mEdges[i + 3];
        return (et0 == tid) ? et1 : ((et1 == tid) ? et0 : InvalidID);
    }



    int find_edge(int vA, int vB) const
    {
        // [RMS] edge vertices must be sorted (min,max),
        //   that means we only need one index-check in inner loop.
        //   commented out code is robust to incorrect ordering, but slower.
        int vO = std::max(vA, vB);
        int vI = std::min(vA, vB);
        for (int eid : mVertexEdges.values(vI))
        {
            if (mEdges[4 * eid + 1] == vO)
                //if (edge_has_v(eid, vO))
                return eid;
        }
        return InvalidID;

        // this is slower, likely because it creates func<> every time. can we do w/o that?
        //return mVertexEdges.Find(vI, (eid) => { return edges[4 * eid + 1] == vO; }, InvalidID);
    }

    int find_edge_from_tri(int vA, int vB, int tID) const
    {
        int i = 3 * tID;
        int t0 = mTriangles[i], t1 = mTriangles[i + 1];
        if (same_pair_unordered(vA, vB, t0, t1))
            return mTriangleEdges[i];
        int t2 = mTriangles[i + 2];
        if (same_pair_unordered(vA, vB, t1, t2))
            return mTriangleEdges[i + 1];
        if (same_pair_unordered(vA, vB, t2, t0))
            return mTriangleEdges[i + 2];
        return InvalidID;
    }



public:

    bool IsBoundaryEdge(int eid) const
    {
        return mEdges[4 * eid + 3] == InvalidID;
    }


    bool IsBoundaryVertex(int vID) const
    {
        for (int eid : mVertexEdges.values(vID))
        {
            if (mEdges[4 * eid + 3] == InvalidID)
                return true;
        }
        return false;
    }


    /// <summary>
    /// Returns true if any edge of triangle is a boundary edge
    /// </summary>
    bool IsBoundaryTriangle(int tID) const
    {
        debug_check_is_triangle(tID);
        int i = 3 * tID;
        return IsBoundaryEdge(mTriangleEdges[i]) ||
               IsBoundaryEdge(mTriangleEdges[i + 1]) ||
               IsBoundaryEdge(mTriangleEdges[i + 2]);
    }



    // queries


       /// <summary>
       /// Returns true if the two triangles connected to edge have different group IDs
       /// </summary>
       bool IsGroupBoundaryEdge(int eID) const
       {
           assert(IsEdge(eID));
           assert(HasTriangleGroups());

           int et1 = mEdges[4 * eID + 3];
           if (et1 == InvalidID)
               return false;
           int g1 = mTriangleGroups[et1];
           int et0 = mEdges[4 * eID + 2];
           int g0 = mTriangleGroups[et0];
           return g1 != g0;
       }


       /// <summary>
       /// returns true if vertex has more than one tri group in its tri nbrhood
       /// </summary>
       bool IsGroupBoundaryVertex(int vID) const
       {
           assert(IsVertex(vID));
           assert(HasTriangleGroups());

           int group_id = InvalidGroupID;
           for (int eID : mVertexEdges.values(vID))
           {
               int et0 = mEdges[4 * eID + 2];
               int g0 = mTriangleGroups[et0];
               if (group_id != g0)
               {
                   if (group_id == InvalidGroupID)
                       group_id = g0;
                   else
                       return true;        // saw multiple group IDs
               }
               int et1 = mEdges[4 * eID + 3];
               if (et1 != InvalidID)
               {
                   int g1 = mTriangleGroups[et1];
                   if (group_id != g1)
                       return true;        // saw multiple group IDs
               }
           }
           return false;
       }



       /// <summary>
       /// returns true if more than two group boundary edges meet at vertex (ie 3+ groups meet at this vertex)
       /// </summary>
       bool IsGroupJunctionVertex(int vID) const
       {
           assert(IsVertex(vID));
           assert(HasTriangleGroups());

           Index2i groups(InvalidGroupID, InvalidGroupID);
           for (int eID : mVertexEdges.values(vID))
           {
               Index2i et = Index2i(mEdges[4 * eID + 2], mEdges[4 * eID + 3]);
               for (int k = 0; k < 2; ++k)
               {
                   if (et[k] == InvalidID)
                       continue;
                   int g0 = mTriangleGroups[et[k]];
                   if (g0 != groups[0] && g0 != groups[1])
                   {
                       if (groups[0] != InvalidGroupID && groups[1] != InvalidGroupID)
                           return true;
                       if (groups[0] == InvalidGroupID)
                           groups[0] = g0;
                       else
                           groups[1] = g0;
                   }
               }
           }
           return false;
       }


//       /// <summary>
//       /// returns up to 4 group IDs at vertex. Returns false if > 4 encountered
//       /// </summary>
//       bool GetVertexGroups(int vID, Index4i & groups) const
//       {
//           assert(IsVertex(vID));
//           assert(HasTriangleGroups());

//           groups = Index4i(InvalidGroupID, InvalidGroupID, InvalidGroupID, InvalidGroupID);
//           int ng = 0;

//           for (int eID : mVertexEdges.values(vID)) {
//               int et0 = mEdges[4 * eID + 2];
//               int g0 = mTriangleGroups[et0];
//               if ( Contains(groups, g0) == false )
//                   groups[ng++] = g0;
//               if (ng == 4)
//                   return false;
//               int et1 = mEdges[4 * eID + 3];
//               if ( et1 != InvalidID ) {
//                   int g1 = mTriangleGroups[et1];
//                   if ( Contains(groups, g1) == false)
//                       groups[ng++] = g1;
//                   if (ng == 4)
//                       return false;
//               }
//           }
//           return true;
//       }



//       /// <summary>
//       /// returns all group IDs at vertex
//       /// </summary>
//       bool GetAllVertexGroups(int vID, std::vector<int> & groups) const
//       {
//           assert(IsVertex(vID));
//           assert(HasTriangleGroups());

//           for (int eID : mVertexEdges.values(vID)) {
//               int et0 = mEdges[4 * eID + 2];
//               int g0 = mTriangleGroups[et0];
//               if ( Contains(groups, g0) == false)
//                   groups.push_back(g0);
//               int et1 = mEdges[4 * eID + 3];
//               if ( et1 != InvalidID ) {
//                   int g1 = mTriangleGroups[et1];
//                   if (Contains(groups, g1) == false)
//                       groups.push_back(g1);
//               }
//           }
//           return true;
//       }
//       std::vector<int> GetAllVertexGroups(int vID) const {
//           std::vector<int> result;
//           GetAllVertexGroups(vID, result);
//           return result;
//       }



       /// <summary>
       /// returns true if vID is a "bowtie" vertex, ie multiple disjoint triangle sets in one-ring
       /// </summary>
       bool IsBowtieVertex(int vID) const
       {
           assert(vertices_refcount.isValid(vID));

           int nEdges = mVertexEdges.Count(vID);
           if (nEdges == 0)
               return false;

           // find a boundary edge to start at
           int start_eid = -1;
           bool start_at_boundary = false;
           for (int eid : mVertexEdges.values(vID))
           {
               if (mEdges[4 * eid + 3] == InvalidID)
               {
                   start_at_boundary = true;
                   start_eid = eid;
                   break;
               }
           }
           // if no boundary edge, start at arbitrary edge
           if (start_eid == -1)
               start_eid = mVertexEdges.First(vID);
           // initial triangle
           int start_tid = mEdges[4 * start_eid + 2];

           int prev_tid = start_tid;
           int prev_eid = start_eid;

           // walk forward to next edge. if we hit start edge or boundary edge,
           // we are done the walk. count number of edges as we go.
           int count = 1;
           while (true) {
               int i = 3 * prev_tid;
               Index3i tv = Index3i(mTriangles[i], mTriangles[i+1], mTriangles[i+2]);
               Index3i te = Index3i(mTriangleEdges[i], mTriangleEdges[i+1], mTriangleEdges[i+2]);
               int vert_idx = find_tri_index(vID, tv);
               int e1 = te[vert_idx], e2 = te[(vert_idx+2) % 3];
               int next_eid = (e1 == prev_eid) ? e2 : e1;
               if (next_eid == start_eid)
                   break;
               Index2i next_eid_tris = GetEdgeT(next_eid);
               int next_tid = (next_eid_tris[0] == prev_tid) ? next_eid_tris[1] : next_eid_tris[0];
               if (next_tid == InvalidID) {
                   break;
               }
               prev_eid = next_eid;
               prev_tid = next_tid;
               count++;
           }

           // if we did not see all edges at vertex, we have a bowtie
           int target_count = (start_at_boundary) ? nEdges - 1 : nEdges;
           bool is_bowtie = (target_count != count);
           return is_bowtie;
       }


       /// <summary>
       /// Computes bounding box of all vertices.
       /// </summary>
      IAABBox3D CalculateBounds() const
       {
           float x = 0, y = 0, z = 0;
           for ( int vi : VertexIndices() )
           {
               x = mVertices[3*vi];
               y = mVertices[3*vi + 1];
               z = mVertices[3*vi + 2];
               break;
           }
           float minx = x, maxx = x, miny = y, maxy = y, minz = z, maxz = z;
           for (int vi : VertexIndices())
           {
               x = mVertices[3*vi];
               y = mVertices[3*vi + 1];
               z = mVertices[3*vi + 2];
               if (x < minx) minx = x; else if (x > maxx) maxx = x;
               if (y < miny) miny = y; else if (y > maxy) maxy = y;
               if (z < minz) minz = z; else if (z > maxz) maxz = z;
           }
           return IAABBox3D(Vector3(minx,miny,minz),
                            Vector3(maxx,maxy,maxz));
       }


      void RecalculateBounds()
      {
         mAxisAlignedBoundingBox = CalculateBounds();
      }

//       AxisAlignedBox3d cached_bounds;
//       int cached_bounds_timestamp = -1;

//       /// <summary>
//       /// cached bounding box, lazily re-computed on access if mesh has changed
//       /// </summary>
//       AxisAlignedBox3d CachedBounds()
//       {
//           if (cached_bounds_timestamp != Timestamp()) {
//               cached_bounds = GetBounds();
//               cached_bounds_timestamp = Timestamp();
//           }
//           return cached_bounds;
//       }


       bool cached_is_closed = false;
       int cached_is_closed_timestamp = -1;

       bool IsClosed() const
       {
           if (TriangleCount() == 0)
               return false;
           // [RMS] under possibly-mistaken belief that foreach() has some overhead...
           if (MaxEdgeID() / EdgeCount() > 5)
           {
               for (int eid : EdgeIndices() )
                   if (IsBoundaryEdge(eid))
                       return false;
           }
           else
           {
               int N = MaxEdgeID();
               for (int i = 0; i < N; ++i)
                   if (edges_refcount.isValid(i) && IsBoundaryEdge(i))
                       return false;
           }
           return true;
       }

       bool CachedIsClosed()
       {
           if (cached_is_closed_timestamp != Timestamp())
           {
               cached_is_closed = IsClosed();
               cached_is_closed_timestamp = Timestamp();
           }
           return cached_is_closed;
       }




       /// <summary> returns true if vertices, edges, and triangles are all "dense" (Count == MaxID) </summary>
       bool IsCompact() const
       {
           return vertices_refcount.is_dense() && edges_refcount.is_dense() && triangles_refcount.is_dense();
       }

       /// <summary> Returns true if vertex count == max vertex id </summary>
       bool IsCompactV() const
       {
           return vertices_refcount.is_dense();
       }

       /// <summary> returns true if triangle count == max triangle id </summary>
       bool IsCompactT() const
       {
           return triangles_refcount.is_dense();
       }

       /// <summary> returns measure of compactness in range [0,1], where 1 is fully compacted </summary>
       float CompactMetric() const
       {
           return ((float)VertexCount() / (float)MaxVertexID() + (float)TriangleCount() / (float)MaxTriangleID()) * 0.5;
       }



       /// <summary>
       /// Compute mesh winding number, from Jacobson et al, Robust Inside-Outside Segmentation using Generalized Winding Numbers
       /// http://igl.ethz.ch/projects/winding-number/
       /// returns ~0 for points outside a closed, consistently oriented mesh, and a positive or negative integer
       /// for points inside, with value > 1 depending on how many "times" the point inside the mesh (like in 2D polygon winding)
       /// </summary>
       float WindingNumber(Vector3 v) const
       {
           float sum = 0;
           for ( int tid : TriangleIndices() )
               sum += GetTriSolidAngle(tid, v);
           return sum / (4.0 * M_PI);
       }




       // Metadata support
       // [RMS] disabled for now

       //bool HasMetadata {
       //    get { return Metadata != nullptr && Metadata.Keys.Count > 0; }
       //}
       //void AttachMetadata(string key, object o)
       //{
       //    if (Metadata == nullptr)
       //        Metadata = Dictionary<string, object>();
       //    Metadata.Add(key, o);
       //}
       //object FindMetadata(string key)
       //{
       //    if (Metadata == nullptr)
       //        return nullptr;
       //    object o = nullptr;
       //    bool bFound = Metadata.TryGetValue(key, out o);
       //    return (bFound) ? o : nullptr;
       //}
       //bool RemoveMetadata(string key)
       //{
       //    if (Metadata == nullptr)
       //        return false;
       //    return Metadata.Remove(key);
       //}
       //void ClearMetadata()
       //{
       //    if (Metadata != nullptr) {
       //        Metadata.Clear();
       //        Metadata = nullptr;
       //    }
       //}







       // direct access to internal dvectors - dangerous!!

       const IMemory::dvector<float>& VerticesBuffer() const
       {
           return mVertices;
       }

       const  IMemory::refcount_vector& VerticesRefCounts() const
       {
           return vertices_refcount;
       }


       const IMemory::dvector<float>& NormalsBuffer() const
       {
           return mNormals;
       }


       const IMemory::dvector<float>& TangentsBuffer() const
       {
           return mTangents;
       }

       const IMemory::dvector<float>& ColorsBuffer() const
       {
           return mColors;
       }

       const IMemory::dvector<float>& UVBuffer() const
       {
           return mUV;
       }

       const IMemory::dvector<int> &TrianglesBuffer() const
       {
           return mTriangles;
       }

       const  IMemory::refcount_vector &TrianglesRefCounts() const
       {
           return triangles_refcount;
       }

       const IMemory::dvector<int> &GroupsBuffer() const
       {
           return mTriangleGroups;
       }

       const IMemory::dvector<int> &EdgesBuffer() const
       {
           return mEdges;
       }

       const  IMemory::refcount_vector &EdgesRefCounts() const
       {
           return edges_refcount;
       }

       const  IMemory::small_list_set &VertexEdges() const
       {
           return mVertexEdges;
       }



       /// <summary>
       /// Rebuild mesh topology.
       /// assumes that we have initialized vertices, triangles, and edges buffers,
       /// and edges refcounts. Rebuilds vertex and tri refcounts, triangle edges, vertex edges.
       /// </summary>
       void RebuildFromEdgeRefcounts()
       {
           int MaxVID = (int)mVertices.length() / 3;
           int MaxTID = (int)mTriangles.length() / 3;

           mTriangleEdges.resize(mTriangles.length());
           triangles_refcount.RawRefCounts().resize(MaxTID);

           mVertexEdges.Resize(MaxVID);
           vertices_refcount.RawRefCounts().resize(MaxVID);

           int MaxEID = (int)mEdges.length() / 4;
           for ( int eid = 0; eid < MaxEID; ++eid )
           {
               if (edges_refcount.isValid(eid) == false)
                   continue;
               int va = mEdges[4 * eid];
               int vb = mEdges[4 * eid + 1];
               int t0 = mEdges[4 * eid + 2];
               int t1 = mEdges[4 * eid + 3];

               // set vertex and tri refcounts to 1
               // find mEdges [a,b] in each triangle and set its tri-edge to this edge

               if (vertices_refcount.isValidUnsafe(va) == false)
               {
                   allocate_edges_list(va);
                   vertices_refcount.set_Unsafe(va, 1);
               }

               if (vertices_refcount.isValidUnsafe(vb) == false)
               {
                   allocate_edges_list(vb);
                   vertices_refcount.set_Unsafe(vb, 1);
               }

               triangles_refcount.set_Unsafe(t0, 1);
               Index3i tri0 = GetTriangle(t0);
               int idx0 = find_edge_index_in_tri(va, vb, tri0);
               mTriangleEdges[3 * t0 + idx0] = eid;

               if (t1 != InvalidID)
               {
                   triangles_refcount.set_Unsafe(t1, 1);
                   Index3i tri1 = GetTriangle(t1);
                   int idx1 = find_edge_index_in_tri(va, vb, tri1);
                   mTriangleEdges[3 * t1 + idx1] = eid;
               }

               // add this edge to both vertices
               mVertexEdges.Insert(va, eid);
               mVertexEdges.Insert(vb, eid);
           }

           // iterate over triangles and increment vtx refcount for each tri
           bool has_groups = HasTriangleGroups();
           max_group_id = 0;
           for ( int tid = 0; tid < MaxTID; ++tid )
           {
               if (triangles_refcount.isValid(tid) == false)
                   continue;
               int a = mTriangles[3 * tid], b = mTriangles[3 * tid + 1], c = mTriangles[3 * tid + 2];
               vertices_refcount.increment(a);
               vertices_refcount.increment(b);
               vertices_refcount.increment(c);

               if (has_groups)
                   max_group_id = std::max(max_group_id, mTriangleGroups[tid]);
           }
           max_group_id++;

           vertices_refcount.rebuild_free_list();
           triangles_refcount.rebuild_free_list();
           edges_refcount.rebuild_free_list();

           updateTimeStamp(true);
       }



       /// <summary>
       /// Compact mesh in-place, by moving vertices around and rewriting indices.
       /// Should be faster if the amount of compacting is not too significant, and
       /// is useful in some places.
       /// [TODO] mVertexEdges is not compacted. does not affect indices, but does keep memory.
       ///
       /// If bComputeCompactInfo=false, the returned CompactInfo is not initialized
       /// </summary>
       // [RMS] disabled for now
       CompactInfo CompactInPlace(bool bComputeCompactInfo = false)
       {
           //IndexMap mapV = (bComputeCompactInfo) ? IndexMap(MaxVertexID, VertexCount) : nullptr;
           std::map<int, int> mapV;
           CompactInfo ci = CompactInfo();
           ci.MapV = mapV;

           // find first free vertex, and last used vertex
           int iLastV = MaxVertexID() - 1, iCurV = 0;
           while (vertices_refcount.isValidUnsafe(iLastV) == false)
               iLastV--;
           while (vertices_refcount.isValidUnsafe(iCurV))
               iCurV++;

           IMemory::dvector<short> vref = vertices_refcount.RawRefCounts();

           while (iCurV < iLastV) {
               int kc = iCurV * 3, kl = iLastV * 3;
              mVertices[kc] =mVertices[kl]; mVertices[kc+1] =mVertices[kl+1]; mVertices[kc+2] =mVertices[kl+2];
               if ( HasVertexNormals() ) {
                   mNormals[kc] = mNormals[kl];  mNormals[kc+1] = mNormals[kl+1];  mNormals[kc+2] = mNormals[kl+2];
               }
               if (HasVertexColors()) {
                   mColors[kc] = mColors[kl];  mColors[kc+1] = mColors[kl+1];  mColors[kc+2] = mColors[kl+2];
               }
               if (HasVertexUVs()) {
                   int ukc = iCurV * 2, ukl = iLastV * 2;
                   mUV[ukc] = mUV[ukl]; mUV[ukc+1] = mUV[ukl+1];
               }

               for ( int eid : mVertexEdges.values(iLastV) )
               {
                   // replace vertex in edges
                   replace_edge_vertex(eid, iLastV, iCurV);

                   // replace vertex in triangles
                   int t0 = mEdges[4*eid + 2];
                   replace_tri_vertex(t0, iLastV, iCurV);
                   int t1 = mEdges[4*eid + 3];
                   if ( t1 != InvalidID )
                       replace_tri_vertex(t1, iLastV, iCurV);
               }

               // shift vertex refcount to position
               vref[iCurV] = vref[iLastV];
               vref[iLastV] = IMemory::refcount_vector::invalid;

               // move edge list
               mVertexEdges.Move(iLastV, iCurV);

               if (bComputeCompactInfo)
                   mapV[iLastV] = iCurV;

               // move cur forward one, last back one, and  then search for next valid
               iLastV--; iCurV++;
               while (vertices_refcount.isValidUnsafe(iLastV) == false)
                   iLastV--;
               while (vertices_refcount.isValidUnsafe(iCurV) && iCurV < iLastV)
                   iCurV++;
           }

           // trim vertices data structures
           vertices_refcount.trim(VertexCount());
           mVertices.resize(VertexCount() * 3);
           if (HasVertexNormals())
               mNormals.resize(VertexCount() * 3);
           if (HasVertexColors())
               mColors.resize(VertexCount() * 3);
           if (HasVertexUVs())
               mUV.resize(VertexCount() * 2);

           // [TODO] mVertexEdges!!!

           /** shift triangles **/

           // find first free triangle, and last valid triangle
           int iLastT = MaxTriangleID() - 1, iCurT = 0;
           while (triangles_refcount.isValidUnsafe(iLastT) == false)
               iLastT--;
           while (triangles_refcount.isValidUnsafe(iCurT))
               iCurT++;

           IMemory::dvector<short> tref = triangles_refcount.RawRefCounts();

           while (iCurT < iLastT)
           {
               int kc = iCurT * 3, kl = iLastT * 3;

               // shift triangle
               for (int j = 0; j < 3; ++j)
               {
                   mTriangles[kc + j] =mTriangles[kl + j];
                   mTriangleEdges[kc + j] = mTriangleEdges[kl + j];
               }
               if (HasTriangleGroups())
                   mTriangleGroups[iCurT] = mTriangleGroups[iLastT];

               // update edges
               for ( int j = 0; j < 3; ++j )
               {
                   int eid = mTriangleEdges[kc + j];
                   replace_edge_triangle(eid, iLastT, iCurT);
               }

               // shift triangle refcount to position
               tref[iCurT] = tref[iLastT];
               tref[iLastT] = IMemory::refcount_vector::invalid;

               // move cur forward one, last back one, and  then search for next valid
               iLastT--; iCurT++;
               while (triangles_refcount.isValidUnsafe(iLastT) == false)
                   iLastT--;
               while (triangles_refcount.isValidUnsafe(iCurT) && iCurT < iLastT)
                   iCurT++;
           }

           // trim triangles data structures
           triangles_refcount.trim(TriangleCount());
           mTriangles.resize(TriangleCount() * 3);
           mTriangleEdges.resize(TriangleCount() * 3);
           if (HasTriangleGroups())
               mTriangleGroups.resize(TriangleCount());

           /** shift edges **/

           // find first free edge, and last used edge
           int iLastE = MaxEdgeID() - 1, iCurE = 0;
           while (edges_refcount.isValidUnsafe(iLastE) == false)
               iLastE--;
           while (edges_refcount.isValidUnsafe(iCurE))
               iCurE++;

           IMemory::dvector<short> eref = edges_refcount.RawRefCounts();

           while (iCurE < iLastE) {
               int kc = iCurE * 4, kl = iLastE * 4;

               // shift edge
               for (int j = 0; j < 4; ++j) {
                   mEdges[kc + j] = mEdges[kl + j];
               }

               // replace edge in vertex edges lists
               int v0 = mEdges[kc], v1 = mEdges[kc + 1];
               mVertexEdges.Replace(v0, [iLastE](int eid) { return eid == iLastE; }, iCurE);
               mVertexEdges.Replace(v1, [iLastE](int eid) { return eid == iLastE; }, iCurE);

               // replace edge in triangles
               replace_triangle_edge(mEdges[kc + 2], iLastE, iCurE);
               if (mEdges[kc + 3] != InvalidID)
                   replace_triangle_edge(mEdges[kc + 3], iLastE, iCurE);

               // shift triangle refcount to position
               eref[iCurE] = eref[iLastE];
               eref[iLastE] = IMemory::refcount_vector::invalid;

               // move cur forward one, last back one, and  then search for next valid
               iLastE--; iCurE++;
               while (edges_refcount.isValidUnsafe(iLastE) == false)
                   iLastE--;
               while (edges_refcount.isValidUnsafe(iCurE) && iCurE < iLastE)
                   iCurE++;
           }

           // trim edge data structures
           edges_refcount.trim(EdgeCount());
           mEdges.resize(EdgeCount() * 4);


           ci.MapV = mapV;

           return ci;
       }






       // edits
       MeshResult ReverseTriOrientation(int tID)
       {
           if (!IsTriangle(tID))
               return MeshResult::Failed_NotATriangle;
           internal_reverse_tri_orientation(tID);
           updateTimeStamp(true);
           return MeshResult::Ok;
       }

       void internal_reverse_tri_orientation(int tID)
       {
           Index3i t = GetTriangle(tID);
           set_triangle(tID, t[1], t[0], t[2]);
           Index3i te = GetTriEdges(tID);
           set_triangle_edges(tID, te[0], te[2], te[1]);
       }

       void ReverseOrientation(bool bFlipNormals = true)
       {
           for ( int tid : TriangleIndices() )
           {
               internal_reverse_tri_orientation(tid);
           }

           if ( bFlipNormals && HasVertexNormals() )
           {
               for ( int vid : VertexIndices() )
               {
                   int i = 3*vid;
                   mNormals[i] = -mNormals[i];
                   mNormals[i+1] = -mNormals[i+1];
                   mNormals[i+2] = -mNormals[i+2];
               }
           }
           updateTimeStamp(true);
       }




       /// <summary>
       /// Remove vertex vID, and all connected triangles if bRemoveAllTriangles = true
       /// (if false, them throws exception if there are still any triangles!)
       /// if bPreserveManifold, checks that we will not create a bowtie vertex first
       /// </summary>
       MeshResult RemoveVertex(int vID, bool bRemoveAllTriangles = true, bool bPreserveManifold = false)
       {
           if (vertices_refcount.isValid(vID) == false)
               return MeshResult::Failed_NotAVertex;

           if ( bRemoveAllTriangles )
           {

               // if any one-ring vtx is a boundary vtx and one of its outer-ring edges is an
               // interior edge then we will create a bowtie if we remove that triangle
               if ( bPreserveManifold )
               {
                   for ( int tid : VtxTrianglesItr(vID) )
                   {
                       Index3i tri = GetTriangle(tid);
                       int j = find_tri_index(vID, tri);
                       int oa = tri[(j + 1) % 3], ob = tri[(j + 2) % 3];
                       int eid = find_edge(oa,ob);
                       if (IsBoundaryEdge(eid))
                           continue;
                       if (IsBoundaryVertex(oa) || IsBoundaryVertex(ob))
                           return MeshResult::Failed_WouldCreateBowtie;
                   }
               }


               std::vector<int> tris;
               GetVtxTriangles(vID, tris, true);
               for (int tID : tris)
               {
                   MeshResult result = RemoveTriangle(tID, false, bPreserveManifold);
                   if (result != MeshResult::Ok)
                       return result;
               }
           }

           assert(vertices_refcount.refCount(vID) == 1);
           //if ( vertices_refcount.refCount(vID) != 1)
           //    throw std::exception("DMesh3.RemoveVertex: vertex is still referenced");

           vertices_refcount.decrement(vID);
           assert(vertices_refcount.isValid(vID) == false);
           mVertexEdges.Clear(vID);

           updateTimeStamp(true);
           return MeshResult::Ok;
       }



       /// <summary>
       /// Remove a tID from the mesh. Also removes any unreferenced edges after tri is removed.
       /// If bRemoveIsolatedVertices is false, then if you remove all tris from a vert, that vert is also removed.
       /// If bPreserveManifold, we check that you will not create a bowtie vertex (and return false).
       ///   If this check is not done, you have to make sure you don't create a bowtie, because other
       ///   code assumes we don't have bowties, and will not handle it properly
       /// </summary>
       MeshResult RemoveTriangle(int tID, bool bRemoveIsolatedVertices = true, bool bPreserveManifold = false)
       {
           if ( ! triangles_refcount.isValid(tID) )
           {
               assert(false);
               return MeshResult::Failed_NotATriangle;
           }

           Index3i tv = GetTriangle(tID);
           Index3i te = GetTriEdges(tID);

           // if any tri vtx is a boundary vtx connected to two interior edges, then
           // we cannot remove this triangle because it would create a bowtie vertex!
           // (that vtx already has 2 boundary edges, and we would add two more)
           if (bPreserveManifold)
           {
               for (int j = 0; j < 3; ++j)
               {
                   if (IsBoundaryVertex(tv[j]))
                   {
                       if (IsBoundaryEdge(te[j]) == false &&
                          IsBoundaryEdge(te[(j + 2) % 3]) == false)
                           return MeshResult::Failed_WouldCreateBowtie;
                   }
               }
           }

           // Remove triangle from its edges. if edge has no triangles left,
           // then it is removed.
           for (int j = 0; j < 3; ++j)
           {
               int eid = te[j];
               replace_edge_triangle(eid, tID, InvalidID);
               if (mEdges[4 * eid + 2] == InvalidID)
               {
                   int a = mEdges[4 * eid];
                   mVertexEdges.Remove(a, eid);

                   int b = mEdges[4 * eid + 1];
                   mVertexEdges.Remove(b, eid);

                   edges_refcount.decrement(eid);
               }
           }

           // free this triangle
           triangles_refcount.decrement( tID );
           assert( triangles_refcount.isValid( tID ) == false );

           // Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
           // we need to remove that vertex
           for (int j = 0; j < 3; ++j)
           {
               int vid = tv[j];
               vertices_refcount.decrement(vid);
               if ( bRemoveIsolatedVertices && vertices_refcount.refCount(vid) == 1)
               {
                   vertices_refcount.decrement(vid);
                   assert(vertices_refcount.isValid(vid) == false);
                   mVertexEdges.Clear(vid);
               }
           }

           updateTimeStamp(true);
           return MeshResult::Ok;
       }






       virtual MeshResult SetTriangle(int tID, Index3i newv, bool bRemoveIsolatedVertices = true)
       {
           Index3i tv = GetTriangle(tID);
           Index3i te = GetTriEdges(tID);
           if (tv[0] == newv[0] && tv[1] == newv[1])
               te[0] = -1;
           if (tv[1] == newv[1] && tv[2] == newv[2])
               te[1] = -1;
           if (tv[2] == newv[2] && tv[0] == newv[0])
               te[2] = -1;

           if (!triangles_refcount.isValid(tID))
           {
               assert(false);
               return MeshResult::Failed_NotATriangle;
           }
           if (IsVertex(newv[0]) == false || IsVertex(newv[1]) == false || IsVertex(newv[2]) == false)
           {
               assert(false);
               return MeshResult::Failed_NotAVertex;
           }
           if (newv[0] == newv[1] || newv[0] == newv[2] || newv[1] == newv[2])
           {
               assert(false);
               return MeshResult::Failed_BrokenTopology;
           }
           // look up edges. if any already have two triangles, this would
           // create non-manifold geometry and so we do not allow it
           int e0 = find_edge(newv[0], newv[1]);
           int e1 = find_edge(newv[1], newv[2]);
           int e2 = find_edge(newv[2], newv[0]);
           if ((te[0] != -1 && e0 != InvalidID && IsBoundaryEdge(e0) == false)
                   || (te[1] != -1 && e1 != InvalidID && IsBoundaryEdge(e1) == false)
                   || (te[2] != -1 && e2 != InvalidID && IsBoundaryEdge(e2) == false))
           {
               return MeshResult::Failed_BrokenTopology;
           }


           // [TODO] check that we are not going to create invalid stuff...

           // Remove triangle from its edges. if edge has no triangles left, then it is removed.
           for (int j = 0; j < 3; ++j)
           {
               int eid = te[j];
               if (eid == -1)      // we don't need to modify this edge
                   continue;
               replace_edge_triangle(eid, tID, InvalidID);
               if (mEdges[4 * eid + 2] == InvalidID)
               {
                   int a = mEdges[4 * eid];
                   mVertexEdges.Remove(a, eid);

                   int b = mEdges[4 * eid + 1];
                   mVertexEdges.Remove(b, eid);

                   edges_refcount.decrement(eid);
               }
           }

           // Decrement vertex refcounts. If any hit 1 and we got remove-isolated flag,
           // we need to remove that vertex
           for (int j = 0; j < 3; ++j)
           {
               int vid = tv[j];
               if (vid == newv[j])     // we don't need to modify this vertex
                   continue;
               vertices_refcount.decrement(vid);
               if (bRemoveIsolatedVertices && vertices_refcount.refCount(vid) == 1)
               {
                   vertices_refcount.decrement(vid);
                   assert(vertices_refcount.isValid(vid) == false);
                   mVertexEdges.Clear(vid);
               }
           }


           // ok now re-insert with vertices
           int i = 3 * tID;
           for (int j = 0; j < 3; ++j)
           {
               if (newv[j] != tv[j]) {
                   mTriangles[i + j] = newv[j];
                   vertices_refcount.increment(newv[j]);
               }
           }

           if ( te[0] != -1 )
               add_tri_edge(tID, newv[0], newv[1], 0, e0);
           if (te[1] != -1)
               add_tri_edge(tID, newv[1], newv[2], 1, e1);
           if (te[2] != -1)
               add_tri_edge(tID, newv[2], newv[0], 2, e2);

           updateTimeStamp(true);
           return MeshResult::Ok;
       }







       struct EdgeSplitInfo {
           bool bIsBoundary;
           int vNew;
           int eNewBN;      // edge [vNew,vB] (original was AB)
           int eNewCN;      // edge [vNew,vC] (C is "first" other vtx in ring)
           int eNewDN;		// edge [vNew,vD] (D is "second" other, which doesn't exist on bdry)
           int eNewT2;
           int eNewT3;
       };
       MeshResult SplitEdge(int vA, int vB, EdgeSplitInfo & split)
       {
           int eid = find_edge(vA, vB);
           if ( eid == InvalidID ) {
               split = EdgeSplitInfo();
               return MeshResult::Failed_NotAnEdge;
           }
           return SplitEdge(eid, split);
       }
       /// <summary>
       /// Split edge eab.
       /// split_t defines position along edge, and is assumed to be based on order of vertices returned by GetEdgeV()
       /// </summary>
       MeshResult SplitEdge(int eab, EdgeSplitInfo & split, float split_t = 0.5)
       {
           split = EdgeSplitInfo();
           if (! IsEdge(eab) )
               return MeshResult::Failed_NotAnEdge;

           // look up primary edge & triangle
           int eab_i = 4*eab;
           int a = mEdges[eab_i], b = mEdges[eab_i + 1];
           int t0 = mEdges[eab_i + 2];
           if (t0 == InvalidID)
               return MeshResult::Failed_BrokenTopology;
           Index3i T0tv = GetTriangle(t0);
           int c = orient_tri_edge_and_find_other_vtx(a, b, T0tv);
           if (vertices_refcount.rawRefCount(c) > 32764)
               return MeshResult::Failed_HitValenceLimit;
           if (a != mEdges[eab_i])
               split_t = 1.0 - split_t;    // if we flipped a/b order we need to reverse t

           // quite a bit of code is duplicated between boundary and non-boundary case, but it
           //  is too hard to follow later if we factor it out...
           if ( IsBoundaryEdge(eab) )
           {

               // create vertex
               Vector3 vNew = Lerp(GetVertex(a), GetVertex(b), split_t);
               int f = AppendVertex(vNew);
               if (HasVertexNormals())
                   SetVertexNormal(f, Lerp( GetVertexNormal(a), GetVertexNormal(b), (float)split_t).Normalized());
               if (HasVertexColors())
                   SetVertexColor(f, Lerp(GetVertexColor(a), GetVertexColor(b), (float)split_t));
               if (HasVertexUVs())
                   SetVertexUV(f, Lerp(GetVertexUV(a), GetVertexUV(b), (float)split_t));

               // look up edge bc, which needs to be modified
               Index3i T0te = GetTriEdges(t0);
               int ebc = T0te[ find_edge_index_in_tri(b, c, T0tv) ];

               // rewrite existing triangle
               replace_tri_vertex(t0, b, f);

               // add second triangle
               int t2 = add_triangle_only(f,b,c, InvalidID, InvalidID, InvalidID);
               if ( HasTriangleGroups() )
                   mTriangleGroups.insertAt(mTriangleGroups[t0], t2);

               // rewrite edge bc, create edge af
               replace_edge_triangle(ebc, t0, t2);
               int eaf = eab;
               replace_edge_vertex(eaf, b, f);
               mVertexEdges.Remove(b, eab);
               mVertexEdges.Insert(f, eaf);

               // create edges fb and fc
               int efb = add_edge(f, b, t2);
               int efc = add_edge(f, c, t0, t2);

               // update triangle edge-nbrs
               replace_triangle_edge(t0, ebc, efc);
               set_triangle_edges(t2, efb, ebc, efc);

               // update vertex refcounts
               vertices_refcount.increment(c);
               vertices_refcount.increment(f, 2);

               split.bIsBoundary = true;
               split.vNew = f;
               split.eNewBN = efb;
               split.eNewCN = efc;
               split.eNewDN = InvalidID;
               split.eNewT2 = t2;
               split.eNewT3 = InvalidID;

               updateTimeStamp(true);
               return MeshResult::Ok;

           }
           else
           {
               // interior triangle branch

               // look up other triangle
               int t1 = mEdges[eab_i + 3];
               Index3i T1tv = GetTriangle(t1);
               int d = find_tri_other_vtx( a, b, T1tv );
               if (vertices_refcount.rawRefCount(d) > 32764)
                   return MeshResult::Failed_HitValenceLimit;

               // create vertex
               Vector3 vNew = Lerp(GetVertex(a), GetVertex(b), split_t);
               int f = AppendVertex(vNew);
               if (HasVertexNormals())
                   SetVertexNormal(f, Lerp(GetVertexNormal(a), GetVertexNormal(b), (float)split_t).Normalized());
               if (HasVertexColors())
                   SetVertexColor(f, Lerp(GetVertexColor(a), GetVertexColor(b), (float)split_t));
               if (HasVertexUVs())
                   SetVertexUV(f, Lerp(GetVertexUV(a), GetVertexUV(b), (float)split_t));

               // look up edges that we are going to need to update
               // [TODO OPT] could use ordering to reduce # of compares here
               Index3i T0te = GetTriEdges(t0);
               int ebc = T0te[find_edge_index_in_tri( b, c, T0tv )];
               Index3i T1te = GetTriEdges(t1);
               int edb = T1te[find_edge_index_in_tri( d, b, T1tv )];

               // rewrite existing triangles
               replace_tri_vertex(t0, b, f);
               replace_tri_vertex(t1, b, f);

               // add two triangles to close holes we just created
               int t2 = add_triangle_only(f,b,c, InvalidID, InvalidID, InvalidID);
               int t3 = add_triangle_only(f, d, b, InvalidID, InvalidID, InvalidID);
               if ( HasTriangleGroups() )
               {
                   mTriangleGroups.insertAt(mTriangleGroups[t0], t2);
                   mTriangleGroups.insertAt(mTriangleGroups[t1], t3);
               }

               // update the edges we found above, to point to triangles
               replace_edge_triangle(ebc, t0, t2);
               replace_edge_triangle(edb, t1, t3);

               // edge eab became eaf
               int eaf = eab; //Edge * eAF = eAB;
               replace_edge_vertex(eaf, b, f);

               // update a/b/f vertex-edges
               mVertexEdges.Remove(b, eab);
               mVertexEdges.Insert(f, eaf);

               // create edges connected to f  (also updates vertex-edges)
               int efb = add_edge( f, b, t2, t3 );
               int efc = add_edge( f, c, t0, t2 );
               int edf = add_edge( d, f, t1, t3 );

               // update triangle edge-nbrs
               replace_triangle_edge(t0, ebc, efc);
               replace_triangle_edge(t1, edb, edf);
               set_triangle_edges(t2, efb, ebc, efc);
               set_triangle_edges(t3, edf, edb, efb);

               // update vertex refcounts
               vertices_refcount.increment( c );
               vertices_refcount.increment( d );
               vertices_refcount.increment( f, 4 );

               split.bIsBoundary = false;
               split.vNew = f;
               split.eNewBN = efb;
               split.eNewCN = efc;
               split.eNewDN = edf;
               split.eNewT2 = t2;
               split.eNewT3 = t3;

               updateTimeStamp(true);
               return MeshResult::Ok;
           }

       }






       struct EdgeFlipInfo
       {
           int eID;
           int v0,v1;
           int ov0,ov1;
           int t0,t1;
       };


//       MeshResult FlipEdge(int vA, int vB, EdgeFlipInfo & flip) {
//           int eid = find_edge(vA, vB);
//           if ( eid == InvalidID ) {
//               flip = EdgeFlipInfo();
//               return MeshResult::Failed_NotAnEdge;
//           }
//           return FlipEdge(eid, flip);
//       }


//       MeshResult FlipEdge(int eab, EdgeFlipInfo & flip)
//       {
//           flip = EdgeFlipInfo();
//           if (! IsEdge(eab) )
//               return MeshResult::Failed_NotAnEdge;
//           if ( IsBoundaryEdge(eab) )
//               return MeshResult::Failed_IsBoundaryEdge;

//           // find oriented edge [a,b], tris t0,t1, and other verts c in t0, d in t1
//           int eab_i = 4*eab;
//           int a = edges[eab_i], b = edges[eab_i + 1];
//           int t0 = edges[eab_i + 2], t1 = edges[eab_i + 3];
//           Index3i T0tv = GetTriangle(t0), T1tv = GetTriangle(t1);
//           int c = orient_tri_edge_and_find_other_vtx( a, b, T0tv );
//           int d = find_tri_other_vtx(a, b, T1tv);
//           if ( c == InvalidID || d == InvalidID ) {
//               return MeshResult::Failed_BrokenTopology;
//           }

//           int flipped = find_edge(c,d);
//           if ( flipped != InvalidID )
//               return MeshResult::Failed_FlippedEdgeExists;

//           // find edges bc, ca, ad, db
//           int ebc = find_tri_neighbour_edge(t0, b, c);
//           int eca = find_tri_neighbour_edge(t0, c, a);
//           int ead = find_tri_neighbour_edge(t1,a,d);
//           int edb = find_tri_neighbour_edge(t1,d,b);

//           // update triangles
//           set_triangle(t0, c, d, b);
//           set_triangle(t1, d, c, a);

//           // update edge AB, which becomes flipped edge CD
//           set_edge_vertices(eab, c, d);
//           set_edge_triangles(eab, t0,t1);
//           int ecd = eab;

//           // update the two other edges whose triangle nbrs have changed
//           if ( replace_edge_triangle(eca, t0,t1) == -1 )
//               throw std::exception("DMesh3.FlipEdge: first replace_edge_triangle failed");
//           if ( replace_edge_triangle(edb, t1, t0) == -1 )
//               throw std::exception("DMesh3.FlipEdge: second replace_edge_triangle failed");

//           // update triangle nbr lists (these are edges)
//           set_triangle_edges(t0, ecd, edb, ebc);
//           set_triangle_edges(t1, ecd, eca, ead);

//           // remove old eab from verts a and b, and decrement ref counts
//           if ( mVertexEdges.Remove(a, eab) == false )
//               throw std::exception("DMesh3.FlipEdge: first edge list remove failed");
//           if ( mVertexEdges.Remove(b, eab) == false )
//               throw std::exception("DMesh3.FlipEdge: second edge list remove failed");
//           vertices_refcount.decrement(a);
//           vertices_refcount.decrement(b);
//           if ( IsVertex(a) == false || IsVertex(b) == false )
//               throw std::exception("DMesh3.FlipEdge: either a or b is not a vertex?");

//           // add edge ecd to verts c and d, and increment ref counts
//           mVertexEdges.Insert(c, ecd);
//           mVertexEdges.Insert(d, ecd);
//           vertices_refcount.increment(c);
//           vertices_refcount.increment(d);

//           // success! collect up results
//           flip.eID = eab;
//           flip.v0 = a; flip.v1 = b;
//           flip.ov0 = c; flip.ov1 = d;
//           flip.t0 = t0; flip.t1 = t1;

//           updateTimeStamp(true);
//           return MeshResult::Ok;
//       }



       void debug_fail(const std::string & s)
       {
       #ifdef DEBUG
           //System.Console.WriteLine("DMesh3.CollapseEdge: check failed: " + s);
           assert(false);
           //throw Exception("DMesh3.CollapseEdge: check failed: " + s);
       #endif
       }


       void check_tri(int t)
       {
           Index3i tv = GetTriangle(t);
           if ( tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2] )
               assert(false);
       }

       void check_edge(int e)
       {
           Index2i tv = GetEdgeT(e);
           if ( tv[0] == -1 )
               assert(false);
       }


       struct EdgeCollapseInfo
       {
           int vKept;
           int vRemoved;
           bool bIsBoundary;

           int eCollapsed;              // edge we collapsed
           int tRemoved0, tRemoved1;    // tris we removed (second may be invalid)
           int eRemoved0, eRemoved1;    // edges we removed (second may be invalid)
           int eKept0, eKept1;          // edges we kept (second may be invalid)
       };

       MeshResult CollapseEdge(int vKeep, int vRemove, EdgeCollapseInfo & collapse)
       {
           collapse = EdgeCollapseInfo();

           if ( IsVertex(vKeep) == false || IsVertex(vRemove) == false )
               return MeshResult::Failed_NotAnEdge;

           int b = vKeep;		// renaming for sanity. We remove a and keep b
           int a = vRemove;

           int eab = find_edge( a, b );
           if (eab == InvalidID)
               return MeshResult::Failed_NotAnEdge;

           int t0 = mEdges[4*eab+2];
           if (t0 == InvalidID)
               return MeshResult::Failed_BrokenTopology;
           Index3i T0tv = GetTriangle(t0);
           int c = find_tri_other_vtx(a, b, T0tv);

           // look up opposing triangle/vtx if we are not in boundary case
           bool bIsBoundaryEdge = false;
           int d = InvalidID;
           int t1 = mEdges[4*eab+3];
           if (t1 != InvalidID)
           {
               Index3i T1tv = GetTriangle(t1);
               d = find_tri_other_vtx( a, b, T1tv );
               if (c == d)
                   return MeshResult::Failed_FoundDuplicateTriangle;
           }
           else
           {
               bIsBoundaryEdge = true;
           }

           // We cannot collapse if edge lists of a and b share vertices other
           //  than c and d  (because then we will make a triangle [x b b].
           //  Unfortunately I cannot see a way to do this more efficiently than brute-force search
           //  [TODO] if we had tri iterator for a, couldn't we check each tri for b  (skipping t0 and t1) ?
           int edges_a_count = mVertexEdges.Count(a);
           int eac = InvalidID, ead = InvalidID, ebc = InvalidID, ebd = InvalidID;
           for ( int eid_a : mVertexEdges.values(a) ) {
               int vax =  edge_other_v(eid_a, a);
               if ( vax == c )
               {
                   eac = eid_a;
                   continue;
               }
               if ( vax == d )
               {
                   ead = eid_a;
                   continue;
               }
               if ( vax == b )
                   continue;

               for (int eid_b : mVertexEdges.values(b))
               {
                   if ( edge_other_v(eid_b, b) == vax )
                       return MeshResult::Failed_InvalidNeighbourhood;
               }
           }

           // [RMS] I am not sure this tetrahedron case will detect bowtie vertices.
           // But the single-triangle case does

           // We cannot collapse if we have a tetrahedron. In this case a has 3 nbr edges,
           //  and edge cd exists. But that is not conclusive, we also have to check that
           //  cd is an internal edge, and that each of its tris contain a or b
           if (edges_a_count == 3 && bIsBoundaryEdge == false)
           {
               int edc = find_edge( d, c );
               int edc_i = 4*edc;
               if (edc != InvalidID && mEdges[edc_i+3] != InvalidID )
               {
                   int edc_t0 = mEdges[edc_i+2];
                   int edc_t1 = mEdges[edc_i+3];

                   if ( (tri_has_v(edc_t0,a) && tri_has_v(edc_t1, b))
                       || (tri_has_v(edc_t0, b) && tri_has_v(edc_t1, a)) )
                   return MeshResult::Failed_CollapseTetrahedron;
               }

           }
           else if (bIsBoundaryEdge == true && IsBoundaryEdge(eac) )
           {
               // Cannot collapse edge if we are down to a single triangle
               ebc = find_edge_from_tri(b, c, t0);
               if ( IsBoundaryEdge(ebc) )
                   return MeshResult::Failed_CollapseTriangle;
           }

           // [RMS] this was added from C++ version...seems like maybe I never hit
           //   this case? Conceivably could be porting bug but looking at the
           //   C++ code I cannot see how we could possibly have caught this case...
           //
           // cannot collapse an edge where both vertices are boundary vertices
           // because that would create a bowtie
           //
           // NOTE: potentially scanning all edges here...couldn't we
           //  pick up eac/bc/ad/bd as we go? somehow?
           if ( bIsBoundaryEdge == false && IsBoundaryVertex(a) && IsBoundaryVertex(b) )
               return MeshResult::Failed_InvalidNeighbourhood;


           // 1) remove edge ab from vtx b
           // 2) find edges ad and ac, and tris tad, tac across those edges  (will use later)
           // 3) for other edges, replace a with b, and add that edge to b
           // 4) replace a with b in all triangles connected to a
           int tad = InvalidID, tac = InvalidID;
           for ( int eid : mVertexEdges.values(a))
           {
               int o = edge_other_v(eid, a);
               if (o == b)
               {
                   if ( mVertexEdges.Remove(b,eid) != true )
                       debug_fail("remove case o == b");
               }
               else if (o == c)
               {
                   if ( mVertexEdges.Remove(c, eid) != true )
                       debug_fail("remove case o == c");
                   tac = edge_other_t(eid, t0);
               }
               else if (o == d)
               {
                   if (mVertexEdges.Remove(d, eid) != true )
                       debug_fail("remove case o == c, step 1");
                   tad = edge_other_t(eid, t1);
               }
               else
               {
                   if ( replace_edge_vertex(eid, a, b) == -1 )
                       debug_fail("remove case else");
                       mVertexEdges.Insert(b, eid);
               }

               // [TODO] perhaps we can already have unique tri list because of the manifold-nbrhood check we need to do...
               for (int j = 0; j < 2; ++j)
               {
                   int t_j = mEdges[4*eid + 2 + j];
                   if (t_j != InvalidID && t_j != t0 && t_j != t1)
                   {
                       if ( tri_has_v(t_j, a) )
                       {
                           if ( replace_tri_vertex(t_j, a, b) == -1 )
                               debug_fail("remove last check");
                           vertices_refcount.increment(b);
                           vertices_refcount.decrement(a);
                       }
                   }
               }
           }

           if (bIsBoundaryEdge == false)
           {

               // remove all edges from vtx a, then remove vtx a
               mVertexEdges.Clear(a);
               assert( vertices_refcount.refCount(a) == 3 );		// in t0,t1, and initial ref
               vertices_refcount.decrement( a, 3 );
               assert( vertices_refcount.isValid( a ) == false );

               // remove triangles T0 and T1, and update b/c/d refcounts
               triangles_refcount.decrement( t0 );
               triangles_refcount.decrement( t1 );
               vertices_refcount.decrement( c );
               vertices_refcount.decrement( d );
               vertices_refcount.decrement( b, 2 );
               assert( triangles_refcount.isValid( t0 ) == false );
               assert( triangles_refcount.isValid( t1 ) == false );

               // remove edges ead, eab, eac
               edges_refcount.decrement( ead );
               edges_refcount.decrement( eab );
               edges_refcount.decrement( eac );
               assert( edges_refcount.isValid( ead ) == false );
               assert( edges_refcount.isValid( eab ) == false );
               assert( edges_refcount.isValid( eac ) == false );

               // replace t0 and t1 in edges ebd and ebc that we kept
               ebd = find_edge_from_tri( b, d, t1 );
               if ( ebc == InvalidID )   // we may have already looked this up
                   ebc = find_edge_from_tri( b, c, t0 );

               if( replace_edge_triangle(ebd, t1, tad ) == -1 )
                   debug_fail("isboundary=false branch, ebd replace triangle");

               if ( replace_edge_triangle(ebc, t0, tac ) == -1 )
                   debug_fail("isboundary=false branch, ebc replace triangle");

               // update tri-edge-nbrs in tad and tac
               if (tad != InvalidID) {
                   if ( replace_triangle_edge(tad, ead, ebd ) == -1 )
                       debug_fail("isboundary=false branch, ebd replace triangle");
               }
               if (tac != InvalidID) {
                   if ( replace_triangle_edge(tac, eac, ebc ) == -1 )
                       debug_fail("isboundary=false branch, ebd replace triangle");
               }

           } else {

               //  this is basically same code as above, just not referencing t1/d

               // remove all edges from vtx a, then remove vtx a
               mVertexEdges.Clear(a);
               assert( vertices_refcount.refCount( a ) == 2 );		// in t0 and initial ref
               vertices_refcount.decrement( a, 2 );
               assert( vertices_refcount.isValid( a ) == false );

               // remove triangle T0 and update b/c refcounts
               triangles_refcount.decrement( t0 );
               vertices_refcount.decrement( c );
               vertices_refcount.decrement( b );
               assert( triangles_refcount.isValid( t0 ) == false );

               // remove edges eab and eac
               edges_refcount.decrement( eab );
               edges_refcount.decrement( eac );
               assert( edges_refcount.isValid( eab ) == false );
               assert( edges_refcount.isValid( eac ) == false );

               // replace t0 in edge ebc that we kept
               ebc = find_edge_from_tri( b, c, t0 );
               if ( replace_edge_triangle(ebc, t0, tac ) == -1 )
                   debug_fail("isboundary=false branch, ebc replace triangle");

               // update tri-edge-nbrs in tac
               if (tac != InvalidID) {
                   if ( replace_triangle_edge(tac, eac, ebc ) == -1 )
                       debug_fail("isboundary=true branch, ebd replace triangle");
               }
           }

           collapse.vKept = vKeep;
           collapse.vRemoved = vRemove;
           collapse.bIsBoundary = bIsBoundaryEdge;
           collapse.eCollapsed = eab;
           collapse.tRemoved0 = t0; collapse.tRemoved1 = t1;
           collapse.eRemoved0 = eac; collapse.eRemoved1 = ead;
           collapse.eKept0 = ebc; collapse.eKept1 = ebd;

           updateTimeStamp(true);
           return MeshResult::Ok;
       }





       struct MergeEdgesInfo
       {
           int eKept;
           int eRemoved;

           Vector2i vKept;
           Vector2i vRemoved;           // either may be InvalidID if it was same as vKept

           Vector2i eRemovedExtra;      // bonus collapsed edge, or InvalidID
           Vector2i eKeptExtra;			// edge paired w/ eRemovedExtra
       };

       MeshResult MergeEdges(int eKeep, int eDiscard, MergeEdgesInfo & merge_info)
       {
           merge_info = MergeEdgesInfo();
           if (IsEdge(eKeep) == false || IsEdge(eDiscard) == false)
               return MeshResult::Failed_NotAnEdge;

           Index4i edgeinfo_keep = GetEdge(eKeep);
           Index4i edgeinfo_discard = GetEdge(eDiscard);
           if (edgeinfo_keep[3] != InvalidID || edgeinfo_discard[3] != InvalidID)
               return MeshResult::Failed_NotABoundaryEdge;

           int a = edgeinfo_keep[0], b = edgeinfo_keep[1];
           int tab = edgeinfo_keep[2];
           int eab = eKeep;
           int c = edgeinfo_discard[0], d = edgeinfo_discard[1];
           int tcd = edgeinfo_discard[2];
           int ecd = eDiscard;

           // Need to correctly orient a,b and c,d and then check that
           // we will not join triangles with incompatible winding order
           // I can't see how to do this purely topologically.
           // So relying on closest-pairs testing.
           orient_tri_edge(a, b, GetTriangle(tab));
           //int tcd_otherv = orient_tri_edge_and_find_other_vtx(ref c, ref d, GetTriangle(tcd));
           orient_tri_edge(c, d, GetTriangle(tcd));
           int x = c; c = d; d = x;   // joinable bdry edges have opposing orientations, so flip to get ac and b/d correspondences
           Vector3 Va = GetVertex(a), Vb = GetVertex(b), Vc = GetVertex(c), Vd = GetVertex(d);
           if ( ((Va-Vc).LengthSquare() + (Vb-Vd).LengthSquare()) >
                ((Va-Vd).LengthSquare() + (Vb-Vc).LengthSquare()))
               return MeshResult::Failed_SameOrientation;

           // alternative that detects normal flip of triangle tcd. This is a more
           // robust geometric test, but fails if tri is degenerate...also more expensive
           //Vector3 otherv = GetVertex(tcd_otherv);
           //Vector3 Ncd = MathUtil.FastNormalDirection(GetVertex(c), GetVertex(d), otherv);
           //Vector3 Nab = MathUtil.FastNormalDirection(GetVertex(a), GetVertex(b), otherv);
           //if (Ncd.Dot(Nab) < 0)
           //return MeshResult::Failed_SameOrientation;

           merge_info.eKept = eab;
           merge_info.eRemoved = ecd;

           // if a/c or b/d are connected by an existing edge, we can't merge
           if (a != c && find_edge(a,c) != InvalidID )
               return MeshResult::Failed_InvalidNeighbourhood;
           if (b != d && find_edge(b, d) != InvalidID)
               return MeshResult::Failed_InvalidNeighbourhood;

           // if vertices at either end already share a common neighbour vertex, and we
           // do the merge, that would create duplicate edges. This is something like the
           // 'link condition' in edge collapses.
           // Note that we have to catch cases where both edges to the shared vertex are
           // boundary edges, in that case we will also merge this edge later on
           if ( a != c )
           {
               int ea = 0, ec = 0, other_v = (b == d) ? b : -1;
               for ( int cnbr : VtxVerticesItr(c) )
               {
                   if (cnbr != other_v && (ea = find_edge(a, cnbr)) != InvalidID)
                   {
                       ec = find_edge(c, cnbr);
                       if (IsBoundaryEdge(ea) == false || IsBoundaryEdge(ec) == false)
                           return MeshResult::Failed_InvalidNeighbourhood;
                   }
               }
           }
           if ( b != d )
           {
               int eb = 0, ed = 0, other_v = (a == c) ? a : -1;
               for ( int dnbr : VtxVerticesItr(d))
               {
                   if (dnbr != other_v && (eb = find_edge(b, dnbr)) != InvalidID)
                   {
                       ed = find_edge(d, dnbr);
                       if (IsBoundaryEdge(eb) == false || IsBoundaryEdge(ed) == false)
                           return MeshResult::Failed_InvalidNeighbourhood;
                   }
               }
           }


           // [TODO] this acts on each interior tri twice. could avoid using vtx-tri iterator?
           if (a != c)
           {
               // replace c w/ a in edges and tris connected to c, and move edges to a
               for ( int eid : mVertexEdges.values(c))
               {
                   if (eid == eDiscard)
                       continue;
                   replace_edge_vertex(eid, c, a);
                   short rc = 0;
                   if (replace_tri_vertex(mEdges[4 * eid + 2], c, a) >= 0)
                       rc++;
                   if (mEdges[4 * eid + 3] != InvalidID) {
                       if (replace_tri_vertex(mEdges[4 * eid + 3], c, a) >= 0)
                           rc++;
                   }
                   mVertexEdges.Insert(a, eid);
                   if (rc > 0) {
                       vertices_refcount.increment(a, rc);
                       vertices_refcount.decrement(c, rc);
                   }
               }
               mVertexEdges.Clear(c);
               vertices_refcount.decrement(c);
               merge_info.vRemoved[0] = c;
           } else {
               mVertexEdges.Remove(a, ecd);
               merge_info.vRemoved[0] = InvalidID;
           }
           merge_info.vKept[0] = a;

           if (d != b)
           {
               // replace d w/ b in edges and tris connected to d, and move edges to b
               for (int eid : mVertexEdges.values(d))
               {
                   if (eid == eDiscard)
                       continue;
                   replace_edge_vertex(eid, d, b);
                   short rc = 0;
                   if (replace_tri_vertex(mEdges[4 * eid + 2], d, b) >= 0)
                       rc++;
                   if (mEdges[4 * eid + 3] != InvalidID)
                   {
                       if (replace_tri_vertex(mEdges[4 * eid + 3], d, b) >= 0)
                           rc++;
                   }
                   mVertexEdges.Insert(b, eid);
                   if (rc > 0)
                   {
                       vertices_refcount.increment(b, rc);
                       vertices_refcount.decrement(d, rc);
                   }

               }
               mVertexEdges.Clear(d);
               vertices_refcount.decrement(d);
               merge_info.vRemoved[1] = d;
           } else {
               mVertexEdges.Remove(b, ecd);
               merge_info.vRemoved[1] = InvalidID;
           }
           merge_info.vKept[1] = b;

           // replace edge cd with edge ab in triangle tcd
           replace_triangle_edge(tcd, ecd, eab);
           edges_refcount.decrement(ecd);

           // update edge-tri adjacency
           set_edge_triangles(eab, tab, tcd);

           // Once we merge ab to cd, there may be additional edges (now) connected
           // to either a or b that are connected to the same vertex on their 'other' side.
           // So we now have two boundary edges connecting the same two vertices - disaster!
           // We need to find and merge these edges.
           // Q: I don't think it is possible to have multiple such edge-pairs at a or b
           //    But I am not certain...is a bit tricky to handle because we modify edges_v...
           merge_info.eRemovedExtra = Vector2i(InvalidID, InvalidID);
           merge_info.eKeptExtra = merge_info.eRemovedExtra;
           for (int vi = 0; vi < 2; ++vi)
           {
               int v1 = a, v2 = c;   // vertices of merged edge
               if ( vi == 1 ) {
                   v1 = b; v2 = d;
               }
               if (v1 == v2)
                   continue;
               std::vector<int> edges_v = vertex_edges_list(v1);
               int Nedges = (int)edges_v.size();
               bool found = false;
               // in this loop, we compare 'other' vert_1 and vert_2 of edges around v1.
               // problem case is when vert_1 == vert_2  (ie two edges w/ same other vtx).
               //restart_merge_loop:
               for (int i = 0; i < Nedges && found == false; ++i)
               {
                   int edge_1 = edges_v[i];
                   if ( IsBoundaryEdge(edge_1) == false)
                       continue;
                   int vert_1 = edge_other_v(edge_1, v1);
                   for (int j = i + 1; j < Nedges; ++j)
                   {
                       int edge_2 = edges_v[j];
                       int vert_2 = edge_other_v(edge_2, v1);
                       if (vert_1 == vert_2 && IsBoundaryEdge(edge_2))
                       { // if ! boundary here, we are in deep trouble...
                           // replace edge_2 w/ edge_1 in tri, update edge and vtx-edge-nbr lists
                           int tri_1 = mEdges[4 * edge_1 + 2];
                           int tri_2 = mEdges[4 * edge_2 + 2];
                           replace_triangle_edge(tri_2, edge_2, edge_1);
                           set_edge_triangles(edge_1, tri_1, tri_2);
                           mVertexEdges.Remove(v1, edge_2);
                           mVertexEdges.Remove(vert_1, edge_2);
                           edges_refcount.decrement(edge_2);
                           merge_info.eRemovedExtra[vi] = edge_2;
                           merge_info.eKeptExtra[vi] = edge_1;

                           //edges_v = mVertexEdges_list(v1);      // this code allows us to continue checking, ie in case we had
                           //Nedges = edges_v.Count;               // multiple such edges. but I don't think it's possible.
                           //goto restart_merge_loop;
                           found = true;			  // exit outer i loop
                           break;					  // exit inner j loop
                       }
                   }
               }
           }

           updateTimeStamp(true);
           return MeshResult::Ok;
       }







       struct PokeTriangleInfo
       {
           int new_vid;
           int new_t1, new_t2;
           Index3i new_edges;
       };

       virtual MeshResult PokeTriangle(int tid, PokeTriangleInfo & result)
       {
           return PokeTriangle(tid, Vector3::IDENTITY / 3.0, result);
       }


       virtual MeshResult PokeTriangle(int tid, const Vector3 & baryCoordinates, PokeTriangleInfo & result)
       {
           result = PokeTriangleInfo();

           if (!IsTriangle(tid))
               return MeshResult::Failed_NotATriangle;

           Index3i tv = GetTriangle(tid);
           Index3i te = GetTriEdges(tid);

           // create vertex with interpolated vertex attribs
           VertexInfo vinfo;
           GetTriBaryPoint(tid, baryCoordinates[0], baryCoordinates[1], baryCoordinates[2], vinfo);
           int center = AppendVertex(vinfo);

           // add in edges to center vtx, do not connect to triangles yet
           int eaC = add_edge(tv[0], center, -1, -1);
           int ebC = add_edge(tv[1], center, -1, -1);
           int ecC = add_edge(tv[2], center, -1, -1);
           vertices_refcount.increment(tv[0]);
           vertices_refcount.increment(tv[1]);
           vertices_refcount.increment(tv[2]);
           vertices_refcount.increment(center, 3);

           // old triangle becomes tri along first edge
           set_triangle(tid, tv[0], tv[1], center);
           set_triangle_edges(tid, te[0], ebC, eaC);

           // add two triangles
           int t1 = add_triangle_only(tv[1], tv[2], center, te[1], ecC, ebC );
           int t2 = add_triangle_only(tv[2], tv[0], center, te[2], eaC, ecC);

           // second and third edges of original tri have neighbours
           replace_edge_triangle(te[1], tid, t1);
           replace_edge_triangle(te[2], tid, t2);

           // set the triangles for the edges we created above
           set_edge_triangles(eaC, tid, t2);
           set_edge_triangles(ebC, tid, t1);
           set_edge_triangles(ecC, t1, t2);

           // transfer groups
           if ( HasTriangleGroups() )
           {
               int g = mTriangleGroups[tid];
               mTriangleGroups.insertAt(g, t1);
               mTriangleGroups.insertAt(g, t2);
           }

           result.new_vid = center;
           result.new_t1 = t1;
           result.new_t2 = t2;
           result.new_edges = Index3i(eaC, ebC, ecC);

           updateTimeStamp(true);
           return MeshResult::Ok;
       }



//       // convert to vertex and triangle array mesh representation used by libigl
//       void ToIGLMesh(Eigen::MatrixXd & V, Eigen::MatrixXi & F)
//       {
//           int NV = VertexCount();
//           V.resize(NV, 3);
//           for (int vid : VertexIndices()) {
//               int i = 3 * vid;
//               V(vid, 0) = vertices[i];
//               V(vid, 1) = vertices[i + 1];
//               V(vid, 2) = vertices[i + 2];
//           }

//           int NT = TriangleCount();
//           F.resize(NT, 3);
//           for (int tid : TriangleIndices()) {
//               int i = 3 * tid;
//               F(tid, 0) = triangles[i];
//               F(tid, 1) = triangles[i + 1];
//               F(tid, 2) = triangles[i + 2];
//           }
//       }




       std::string MeshInfoString()
       {
           std::ostringstream b;
           b << "Vertices  " << " count " << VertexCount() << " max " << MaxVertexID() << " " << vertices_refcount.UsageStats() << std::endl;
           b << "Triangles " << " count " << TriangleCount() << " max " << MaxTriangleID() << " " << triangles_refcount.UsageStats() << std::endl;
           b << "Edges     " << " count " << EdgeCount() << " max " << MaxEdgeID() << " " << edges_refcount.UsageStats() << std::endl;
           b << "Normals " << HasVertexNormals() << "  Colors " << HasVertexColors() << "  UVs " << HasVertexUVs() << "  Groups " << HasTriangleGroups() << std::endl;
           b << "Closed " << CachedIsClosed() << " Compact " << IsCompact() << " timestamp " << timestamp << " shape_timestamp " << shape_timestamp << "  MaxGroupID " << MaxGroupID() << std::endl;
           b << "VertexEdges " << mVertexEdges.MemoryUsage() << std::endl;
           return b.str();
       }


//       /// <summary>
//       /// Check if this m2 is the same as this mesh. By default only checks
//       /// vertices and triangles, turn on other parameters w/ flags
//       /// </summary>
//       bool IsSameMesh(const IMesh & m2, bool bCheckConnectivity, bool bCheckEdgeIDs = false,
//           bool bCheckNormals = false, bool bCheckColors = false, bool bCheckUVs = false,
//           bool bCheckGroups = false,
//           float Epsilon = MACHINE_EPSILON)
//       {
//           if (VertexCount() != m2.VertexCount())
//               return false;
//           if (TriangleCount() != m2.TriangleCount())
//               return false;
//           for (int vid : VertexIndices()) {
//               if (m2.IsVertex(vid) == false || EpsilonEqual(GetVertex(vid), m2.GetVertex(vid), Epsilon) == false)
//                   return false;
//           }
//           for (int tid : TriangleIndices()) {
//               if (m2.IsTriangle(tid) == false || (GetTriangle(tid) != m2.GetTriangle(tid)) )
//                   return false;
//           }
//           if (bCheckConnectivity) {
//               for (int eid : EdgeIndices()) {
//                   Index4i e = GetEdge(eid);
//                   int other_eid = m2.FindEdge(e[0], e[1]);
//                   if (other_eid == InvalidID)
//                       return false;
//                   Index4i oe = m2.GetEdge(other_eid);
//                   if (std::min(e[2], e[3]) != std::min(oe[2], oe[3]) || std::max(e[2], e[3]) != std::max(oe[2], oe[3]))
//                       return false;
//               }
//           }
//           if (bCheckEdgeIDs) {
//               if (EdgeCount() != m2.EdgeCount())
//                   return false;
//               for (int eid : EdgeIndices()) {
//                   if (m2.IsEdge(eid) == false || GetEdge(eid) != m2.GetEdge(eid) )
//                       return false;
//               }
//           }
//           if (bCheckNormals) {
//               if (HasVertexNormals() != m2.HasVertexNormals())
//                   return false;
//               if (HasVertexNormals()) {
//                   for (int vid : VertexIndices()) {
//                       if ( EpsilonEqual(GetVertexNormal(vid), m2.GetVertexNormal(vid), Epsilon) == false)
//                           return false;
//                   }
//               }
//           }
//           if (bCheckColors) {
//               if (HasVertexColors() != m2.HasVertexColors())
//                   return false;
//               if (HasVertexColors()) {
//                   for (int vid : VertexIndices()) {
//                       if ( EpsilonEqual( GetVertexColor(vid), m2.GetVertexColor(vid), Epsilon) == false)
//                           return false;
//                   }
//               }
//           }
//           if (bCheckUVs) {
//               if (HasVertexUVs() != m2.HasVertexUVs())
//                   return false;
//               if (HasVertexUVs()) {
//                   for (int vid : VertexIndices()) {
//                       if ( EpsilonEqual( GetVertexUV(vid), m2.GetVertexUV(vid), Epsilon) == false)
//                           return false;
//                   }
//               }
//           }
//           if (bCheckGroups) {
//               if (HasTriangleGroups() != m2.HasTriangleGroups())
//                   return false;
//               if (HasTriangleGroups()) {
//                   for (int tid : TriangleIndices()) {
//                       if (GetTriangleGroup(tid) != m2.GetTriangleGroup(tid))
//                           return false;
//                   }
//               }
//           }
//           return true;
//       }




//       enum class FailMode { DevAssert, Throw, ReturnOnly };


//       /// <summary>
//       // This function checks that the mesh is well-formed, ie all internal data
//       // structures are consistent
//       /// </summary>
//       bool CheckValidity(bool bAllowNonManifoldVertices = false, FailMode eFailMode = FailMode::Throw) const {

//           std::vector<int> triToVtxRefs;
//           triToVtxRefs.resize(MaxVertexID());
//           //int[] triToVtxRefs = new int[this.MaxVertexID];

//           bool is_ok = true;
//           std::function<void(bool)> CheckOrFailF = [&](bool b) {
//               is_ok = is_ok && b;
//           };
//           if (eFailMode == FailMode::DevAssert) {
//               CheckOrFailF = [&](bool b) {
//                   assert(b);
//                   is_ok = is_ok && b;
//               };
//           } else if (eFailMode == FailMode::Throw) {
//               CheckOrFailF = [&](bool b) {
//                   if (b == false)
//                       throw std::exception("DMesh3.CheckValidity: check failed!");
//                   is_ok = is_ok && b;
//               };
//           }

//           //if (normals != null)
//           //	CheckOrFailF(normals.size == vertices.size);
//           //if (colors != null)
//           //	CheckOrFailF(colors.size == vertices.size);
//           //if (uv != null)
//           //	CheckOrFailF(uv.size / 2 == vertices.size / 3);
//           //if (mTriangleGroups != null)
//           //	CheckOrFailF(mTriangleGroups.size == triangles.size / 3);

//           for (int tID : TriangleIndices()) {

//               CheckOrFailF(IsTriangle(tID));
//               CheckOrFailF(triangles_refcount.refCount(tID) == 1);

//               // vertices must exist
//               Index3i tv = GetTriangle(tID);
//               for (int j = 0; j < 3; ++j) {
//                   CheckOrFailF(IsVertex(tv[j]));
//                   triToVtxRefs[tv[j]] += 1;
//               }

//               // edges must exist and reference this tri
//               Index3i e;
//               for (int j = 0; j < 3; ++j) {
//                   int a = tv[j], b = tv[(j + 1) % 3];
//                   e[j] = FindEdge(a, b);
//                   CheckOrFailF(e[j] != InvalidID);
//                   CheckOrFailF(edge_has_t(e[j], tID));
//                   CheckOrFailF(e[j] == FindEdgeFromTri(a, b, tID));
//               }
//               CheckOrFailF(e[0] != e[1] && e[0] != e[2] && e[1] != e[2]);

//               // tri nbrs must exist and reference this tri, or same edge must be boundary edge
//               Index3i te = GetTriEdges(tID);
//               for (int j = 0; j < 3; ++j) {
//                   int eid = te[j];
//                   CheckOrFailF(IsEdge(eid));
//                   int tOther = edge_other_t(eid, tID);
//                   if (tOther == InvalidID) {
//                       CheckOrFailF(IsBoundaryTriangle(tID));
//                       continue;
//                   }

//                   CheckOrFailF(tri_has_neighbour_t(tOther, tID) == true);

//                   // edge must have same two verts as tri for same index
//                   int a = tv[j], b = tv[(j + 1) % 3];
//                   Index2i ev = GetEdgeV(te[j]);
//                   CheckOrFailF(same_pair_unordered(a, b, ev[0], ev[1]));

//                   // also check that nbr edge has opposite orientation
//                   Index3i othertv = GetTriangle(tOther);
//                   int found = find_tri_ordered_edge(b, a, othertv);
//                   CheckOrFailF(found != InvalidID);
//               }
//           }


//           // edge verts/tris must exist
//           for (int eID : EdgeIndices()) {
//               CheckOrFailF(IsEdge(eID));
//               CheckOrFailF(edges_refcount.refCount(eID) == 1);
//               Index2i ev = GetEdgeV(eID);
//               Index2i et = GetEdgeT(eID);
//               CheckOrFailF(IsVertex(ev[0]));
//               CheckOrFailF(IsVertex(ev[1]));
//               CheckOrFailF(et[0] != InvalidID);
//               CheckOrFailF(ev[0] < ev[1]);
//               CheckOrFailF(IsTriangle(et[0]));
//               if (et[1] != InvalidID) {
//                   CheckOrFailF(IsTriangle(et[1]));
//               }
//           }

//           // verify compact check
//           bool is_compact = vertices_refcount.is_dense();
//           if (is_compact) {
//               for (int vid = 0; vid < vertices.length() / 3; ++vid) {
//                   CheckOrFailF(vertices_refcount.isValid(vid));
//               }
//           }

//           // vertex edges must exist and reference this vert
//           for (int vID : VertexIndices()) {
//               CheckOrFailF(IsVertex(vID));

//               Vector3 v = GetVertex(vID);
//               CheckOrFailF( is_nan(v.LengthSquare()) == false);
//               CheckOrFailF( isfinite(v.LengthSquare()) );

//               for (int edgeid : mVertexEdges.values(vID)) {
//                   CheckOrFailF(IsEdge(edgeid));
//                   CheckOrFailF(edge_has_v(edgeid, vID));

//                   int otherV = edge_other_v(edgeid, vID);
//                   int e2 = find_edge(vID, otherV);
//                   CheckOrFailF(e2 != InvalidID);
//                   CheckOrFailF(e2 == edgeid);
//                   e2 = find_edge(otherV, vID);
//                   CheckOrFailF(e2 != InvalidID);
//                   CheckOrFailF(e2 == edgeid);
//               }

//               for (int nbr_vid : VtxVerticesItr(vID)) {
//                   CheckOrFailF(IsVertex(nbr_vid));
//                   int edge = find_edge(vID, nbr_vid);
//                   CheckOrFailF(IsEdge(edge));
//               }

//               std::vector<int> vTris, vTris2;
//               GetVtxTriangles(vID, vTris, false);
//               GetVtxTriangles(vID, vTris2, true);
//               CheckOrFailF(vTris.size() == vTris2.size());
//               //System.Console.WriteLine(string.Format("{0} {1} {2}", vID, vTris.Count, GetVtxEdges(vID).Count));
//               if (bAllowNonManifoldVertices)
//                   CheckOrFailF(vTris.size() <= GetVtxEdgeCount(vID));
//               else
//                   CheckOrFailF(vTris.size() == GetVtxEdgeCount(vID) || vTris.size() == GetVtxEdgeCount(vID) - 1);
//               CheckOrFailF(vertices_refcount.refCount(vID) == vTris.size() + 1);
//               CheckOrFailF(triToVtxRefs[vID] == vTris.size());
//               for (int tID : vTris) {
//                   CheckOrFailF(tri_has_v(tID, vID));
//               }

//               // check that edges around vert only references tris above, and reference all of them!
//               std::vector<int> vRemoveTris(vTris);
//               for (int edgeid : mVertexEdges.values(vID)) {
//                   Index2i edget = GetEdgeT(edgeid);
//                   CheckOrFailF( Contains(vTris,edget[0]) );
//                   if (edget[1] != InvalidID)
//                       CheckOrFailF( Contains(vTris, edget[1]) );
//                   Remove(vRemoveTris, edget[0]);
//                   if (edget[1] != InvalidID)
//                       Remove(vRemoveTris, edget[1]);
//               }
//               CheckOrFailF(vRemoveTris.size() == 0);
//           }

//           return is_ok;
//       }




//       int textureID() const
//       {
//           return mTextureID;
//       }

//       void setTextureID(int textureID)
//       {
//           mTextureID = textureID;
//       }

       IMemory::dvector<int> triangles() const
       {
           return mTriangles;
       }
};


}

#endif // IOBJECTMESH_H


#endif // IMESH_H
