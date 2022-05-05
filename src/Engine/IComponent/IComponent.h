#ifndef ICOMPONENT_H
#define ICOMPONENT_H

#include "../imaths.hpp"
#include "../engine.hpp"

#include "../IHierarchy/IHierarchy/IHierarchyNode.h"

#include <map>
//#include <GL/freeglut.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

namespace IEngine
{

using namespace IMath;
using namespace IEngine;

class IComponent : public IHierarchyNode //: AffineTransform
{

   private:

     ICollisionBody                       *mCollid;
     std::map< IMesh* , IProxyShape* >     mMeshes;


   public:

      IComponent(ICollisionBody *_collid , unsigned int id = 0)
       : IHierarchyNode(id) ,
         mCollid(_collid)
      {}


      Matrix4   GetTransformMatrixHierarchy() const;
      Transform GetTransformHierarchy() const;


      void SetTransform( const IMatrix4x4<float>& m );


      IProxyShape* Add( ICollisionShape* collid_shape , IMesh* mesh , scalar massa = 1.f);





      //-=-=-=-=-=-=-=-=-=-=-=-=-=-=///
      void Update()
      {
//          for (auto it=mMeshes.begin(); it != mMeshes.end(); ++it)
//          {
//              auto  proxy =  it->second;
//              Vector3 point = mCollid->GetTransform().GetTransformMatrix().GetTranspose() *
//                              (proxy->GetLocalToBodyTransform().GetTransformMatrix().GetTranspose() * Vector3::ZERO);

//              glPushMatrix();
//              glPointSize(5);
//              glColor3f(1,0,0);
//              glBegin(GL_POINTS);
//              glVertex3fv( point );
//              glEnd();
//              glPopMatrix();


//              if((point - old).Length() > 0.1f) points.push_back(point);
//              old = point;

//          }
      }


      void Draw()
      {
          //---------------------------------------------//


//          std::cout <<  points.size() << std::endl;


//          for (int i = 0; i < points.size(); ++i)
//          {
//              glPushMatrix();
//              glPointSize(5);
//              glBegin(GL_POINTS);
//              glVertex3fv(points[i]);
//              glEnd();
//              glPopMatrix();
//          }



//          glPushMatrix();
//          glMultMatrixf(GetTransformMatrixHierarchy());
//          glLineWidth(4);
//          glBegin(GL_LINES);

//               glColor3f(1,0,0);
//              glVertex3f(0,0,0);
//              glVertex3f(1,0,0);

//               glColor3f(0,1,0);
//              glVertex3f(0,0,0);
//              glVertex3f(0,1,0);

//               glColor3f(0,0,1);
//              glVertex3f(0,0,0);
//              glVertex3f(0,0,1);
//          glEnd();
//          glLineWidth(2);
//          glPopMatrix();

          //---------------------------------------------//

//          for (auto it=mMeshes.begin(); it != mMeshes.end(); ++it)
//          {
//              auto  proxy =  it->second;
//              auto  ComponentMesh = it->first;
//              glPushMatrix();
//              glMultMatrixf(GetTransformMatrixHierarchy());
//              glMultMatrixf(proxy->GetLocalToBodyTransform().GetTransformMatrix());
//              glLineWidth(3);
//              for (Index4i t : ComponentMesh->Edges())
//              {
//                  glBegin(GL_LINES);
//                  glColor3f(1,1,1);
//                  glVertex3fv(ComponentMesh->GetVertex(t[0]));
//                  glVertex3fv(ComponentMesh->GetVertex(t[1]));
//                  glEnd();
//              }
//              glLineWidth(1);
//              glPopMatrix();
//          }

          //---------------------------------------------//
      }

      ICollisionBody *Collid() const
      {
          return mCollid;
      }

      std::map<IMesh*, IProxyShape*> meshes() const
      {
          return mMeshes;
      }
};


}


#endif // ICOMPONENT_H
