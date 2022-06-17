#include "IMeshLoader.h"
#include <fstream>
#include <iostream>

namespace IEngine
{

using namespace std;

IMeshLoader::IMeshLoader(const std::string _filename, bool localization)
    :mFilename(_filename)
{
   this->loader(localization);
}

bool IMeshLoader::loader(bool localization)
{
    unsigned short chunk_id;
    unsigned int   chunk_pos;
    unsigned int   chunk_temppos;
    unsigned int   chunk_len;

    unsigned short nVertexs = 0;

    ifstream ifs;
    ifs.open(mFilename, ios_base::in | ios_base::binary | ios_base::openmode(ios_base::beg));



    std::vector<Vector3> Points;
    std::vector<Vector2> UVs;

    if (!ifs.is_open())
    {
        cout << "not File:: <"<< mFilename <<">"<< endl;
        return false;
    }
    else
    {

        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);
        if (chunk_id != MAIN3DS) return false;
        ifs.seekg(ifs.tellg() - (std::streamoff) 6);

        chunk_pos = FindChunk(ifs, EDIT3DS);
        if (chunk_pos == 0)  return false;



        chunk_pos = FindChunk(ifs, EDIT_OBJECT);
        if (chunk_pos == 0)  return false;

        chunk_pos = FindChunk(ifs, OBJ_TRIMESH);
        if (chunk_pos == 0) return false;

        chunk_temppos = chunk_pos;

        chunk_pos = FindChunk(ifs, 0x4110);
        if (chunk_pos == 0) return false;
        ifs.ignore(6);


        ifs.read((char*) &nVertexs, 2);


        Points.resize(nVertexs);
        for (int i = 0; i < nVertexs; ++i)
        {
            float x,y,z;
            ifs.read((char*) &(x), 4);
            ifs.read((char*) &(y), 4);
            ifs.read((char*) &(z), 4);

            Points[i] = Vector3(x,z,y);//y � z
        }

    }


    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_MAPPINGCOORS);
    if (chunk_pos == 0) return false;
    ifs.ignore(6);

    unsigned short nTexCoords;
    ifs.read((char*) &nTexCoords, 2);


    UVs.resize(nTexCoords);
    for (int i = 0; i < nTexCoords; i++)
    {
        float x;
        float y;
        ifs.read((char*) &(x), 4);
        ifs.read((char*) &(y), 4);

        UVs[i] = Vector2(x,y);
    }



    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_FACELIST);
    if (chunk_pos == 0)  return false;
    ifs.ignore(6);

    unsigned short nTriangles;
    ifs.read((char*) &nTriangles, 2);


    std::vector<unsigned short> indices;

    for (int i = 0; i < nTriangles; i++)
    {
        unsigned short A, B, C;

        ifs.read((char*) &(A), 2);
        ifs.read((char*) &(B), 2);
        ifs.read((char*) &(C), 2);

        indices.push_back(A);
        indices.push_back(B);
        indices.push_back(C);

        ifs.ignore(2);
    }


    for(unsigned int i=0; i<Points.size();++i)
    {
       VertexInfo vertex(Points[i],UVs[i]);
       this->AppendVertex(vertex);
    }



    for(unsigned int i=0; i<indices.size(); i+=3)
    {
        unsigned short A = indices[i+0];
        unsigned short B = indices[i+1];
        unsigned short C = indices[i+2];

        this->AppendTriangle(A,B,C);
    }


    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_LOCAL);
    if (chunk_pos == 0) return false;
    ifs.ignore(6);


    float Local[12] = { 0.0f };
    ifs.read((char*) &Local, sizeof(float) * 12);


    Vector3   pos(Local[9], Local[10] , Local[11]);
    Matrix3   rot(Local[0], Local[2], Local[1],
                  Local[3], Local[5], Local[4],
                  Local[6], Local[8], Local[7]);


    if( localization )
    {
        Transform   transform( pos , rot );

//        Matrix4 matrixTransform;
//        matrixTransform.SetToIdentity();
//        matrixTransform.SetTranslation(pos);
//        matrixTransform.SetRotation(rot) ;

        for(unsigned int i=0; i<Points.size();++i)
        {
           Vector3 point_local = transform.GetInverse() * GetVertex(i);
           this->SetVertex(i,point_local);
        }

        RecalculateBounds();

    }
    else
    {
        //Transform  transform( pos , rot );
        //SetTransform(transform);
    }






    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_MATERIAL);
    if (chunk_pos == 0)  return false;
    ifs.ignore(6);


    return true;
}

unsigned int IMeshLoader::FindChunk(std::ifstream &ifs, unsigned short id, bool isParent)
{
    unsigned short chunk_id;
    unsigned int   chunk_len;

    if (isParent)
    {

        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);

        if (chunk_id == EDIT_OBJECT)
        {
            char ch;
            do
            {
                ifs.read((char*) &ch, 1);
            } while (ch != '\0' && !ifs.eof());
        }
    }
    do
    {
        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);

        if (chunk_id != id)
        {
            ifs.ignore(chunk_len - 6);
        }
        else
        {
            ifs.seekg(ifs.tellg() - (std::streamoff) 6);
            return (ifs.tellg());
        }
    }
    while (!ifs.eof());

    return 0;

}

}
