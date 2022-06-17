#ifndef IAABBOX3D_H
#define IAABBOX3D_H

#include "../imaths.hpp"
#include "Segments/IRay.h"

namespace IEngine
{


   class IAABBox3D
   {

     private:

       mutable Vector3 mMin;
       mutable Vector3 mMax;


     public:

       static const IAABBox3D Empty;
       static const IAABBox3D Zero;
       static const IAABBox3D UnitPositive;


       IAABBox3D();
       IAABBox3D(scalar xmin, scalar ymin, scalar zmin,
                 scalar xmax, scalar ymax, scalar zmax);

       /// <summary>
       /// init box [0,size] x [0,size] x [0,size]
       /// </summary>
       IAABBox3D(scalar fCubeSize);

       /// <summary>
       /// Init box [0,width] x [0,height] x [0,depth]
       /// </summary>
       IAABBox3D(scalar fWidth, scalar fHeight, scalar fDepth);
       IAABBox3D(Vector3& vMin, Vector3& vMax);
       IAABBox3D(const Vector3& vMin, const Vector3& vMax);
       IAABBox3D(const Vector3& vCenter, scalar fHalfWidth, scalar fHalfHeight, scalar fHalfDepth);
       IAABBox3D(Vector3& vCenter, scalar fHalfWidth, scalar fHalfHeight, scalar fHalfDepth);

       IAABBox3D(const Vector3& vCenter, scalar fHalfSize);
       IAABBox3D(const Vector3& vCenter);

       //------------------------------------------------------//


       //       // Merge the IAABB in parameter with the current one
       //       void MergeWithAABB(const  IAABBox3D& aabb)
       //       {

       //           mMin.x = IMin(mMin.x, aabb.mMin.x);
       //           mMin.y = IMin(mMin.y, aabb.mMin.y);
       //           mMin.z = IMin(mMin.z, aabb.mMin.z);

       //           mMax.x = IMax(mMax.x, aabb.mMax.x);
       //           mMax.y = IMax(mMax.y, aabb.mMax.y);
       //           mMax.z = IMax(mMax.z, aabb.mMax.z);
       //       }

              // Replace the current AABB with a new AABB that is the union of two AABBs in parameters
              void MergeTwoAABBs(const  IAABBox3D& aabb1, const  IAABBox3D& aabb2);

       //       // Return true if the current AABB contains the AABB given in parameter
       //       bool Contains(const  IAABBox3D& aabb) const
       //       {

       //           bool isInside = true;
       //           isInside = isInside && mMin.x <= aabb.mMin.x;
       //           isInside = isInside && mMin.y <= aabb.mMin.y;
       //           isInside = isInside && mMin.z <= aabb.mMin.z;

       //           isInside = isInside && mMax.x >= aabb.mMax.x;
       //           isInside = isInside && mMax.y >= aabb.mMax.y;
       //           isInside = isInside && mMax.z >= aabb.mMax.z;
       //           return isInside;
       //       }

              // Create and return an AABB for a triangle
              static IAABBox3D CreateAABBForTriangle(const Vector3* trianglePoints);

              // Return true if the ray intersects the AABB
              /// This method use the line vs AABB raycasting technique described in
              /// Real-time Collision Detection by Christer Ericson.
              bool TestRayIntersect(const IRay &ray, const Matrix4 &transform = Matrix4::IDENTITY, IRaycast *_raycast = nullptr) const;



              // Return true if the current IAABB is overlapping with the IAABB in argument.
              /// Two IAABBs overlap if they overlap in the three x, y and z axis at the same time
              bool TestCollision(const IAABBox3D& aabb) const;

              // Return the volume of the AABB
              scalar GetVolume() const;

       //------------------------------------------------------//
       Vector3 GetMax() const;
       Vector3 GetMin() const;

       void SetMin(const Vector3 &min);
       void SetMax(const Vector3 &max);

       Vector3 HalfSize() const;



       scalar Width()   const;
       scalar Height()  const;

       scalar Depth()   const;
       scalar Volume()  const;

       scalar  DiagonalLength()  const;
       scalar  MaxDim()          const;

       Vector3 Diagonal()  const;
       Vector3 Extents()   const;
       Vector3 Center()    const;

       bool operator ==(const IAABBox3D& b) const ;
       bool operator != (const IAABBox3D& b);

       bool Equals(IAABBox3D other);

//       int CompareTo(IAABBox3D other)
//       {
//           int c = this.Min.CompareTo(other.Min);
//           if (c == 0)
//               return this.Max.CompareTo(other.Max);
//           return c;
//       }

       int GetHashCode()  const;


       // See Box3.Corner for details on which corner is which
       Vector3 Corner(int i)  const;

       /// <summary>
       /// Returns point on face/edge/corner. For each coord value neg==min, 0==center, pos==max
       /// </summary>
       Vector3 Point(int xi, int yi, int zi)  const;


       // TODO
       ////! 0 == bottom-left, 1 = bottom-right, 2 == top-right, 3 == top-left
       //Vector3 GetCorner(int i) {
       //    return Vector3((i % 3 == 0) ? Min.x : Max.x, (i < 2) ? Min.y : Max.y);
       //}

       //! value is subtracted from min and added to max
       void Expand(scalar fRadius);

       //! return this box expanded by radius
       IAABBox3D Expanded(scalar fRadius) const;

       //! value is added to min and subtracted from max
       void Contract(scalar fRadius)  const;

       //! return this box expanded by radius
       IAABBox3D Contracted(scalar fRadius)  const;


       void Scale(scalar sx, scalar sy, scalar sz);

       void MergeWithAABB(Vector3& v);

       void MergeWithAABB(const IAABBox3D& box);


       // Replace the current AABB with a new AABB that is the union of two AABBs in parameters
       void ContainsTwoAABBs(const IAABBox3D& aabb1, const IAABBox3D& aabb2);

       IAABBox3D Intersect(const IAABBox3D& box)  const;

       // Return true if the IAABB of a triangle intersects the IAABB
       bool TestCollisionTriangleAABB(const Vector3* trianglePoints) const;


       bool Contains(Vector3& v)             const;
       bool Contains(const Vector3& v)       const;

       bool Contains(IAABBox3D box2)         const;
       bool Contains(const IAABBox3D& box2)  const;


       bool Containsss(const IAABBox3D& box2);


       bool Intersects(const IAABBox3D& box)  const;
       bool IntersectsRay( IRay &r );



       scalar DistanceSquared(const Vector3& v)  const;

       scalar Distance(const Vector3& v)  const;

       scalar SignedDistance(const Vector3& v)  const;

       scalar DistanceSquared(const IAABBox3D& box2)  const;


       // [TODO] we have handled corner cases, but not edge cases!
       //   those are 2D, so it would be like (dx > width && dy > height)
       //T Distance(const Vector3& v)
       //{
       //    T dx = (double)im::IAbs(v.x - Center.x);
       //    T dy = (double)im::IAbs(v.y - Center.y);
       //    T dz = (double)im::IAbs(v.z - Center.z);
       //    T fWidth = Width * 0.5;
       //    T fHeight = Height * 0.5;
       //    T fDepth = Depth * 0.5;
       //    if (dx < fWidth && dy < fHeight && dz < Depth)
       //        return 0.0f;
       //    else if (dx > fWidth && dy > fHeight && dz > fDepth)
       //        return (double)Math.Sqrt((dx - fWidth) * (dx - fWidth) + (dy - fHeight) * (dy - fHeight) + (dz - fDepth) * (dz - fDepth));
       //    else if (dx > fWidth)
       //        return dx - fWidth;
       //    else if (dy > fHeight)
       //        return dy - fHeight;
       //    else if (dz > fDepth)
       //        return dz - fDepth;
       //    return 0.0f;
       //}


       IAABBox3D GetAxisAlignedBoxTransform(const Matrix4 &_TransformMatrix = Matrix4::IDENTITY) const;



       //! relative translation
       void Translate(const Vector3& vTranslate);

       void MoveMin(const Vector3& vNewMin);

       void MoveMin(scalar fNewX, scalar fNewY,scalar fNewZ);


       void Insert(const Vector3& point);

       void Insert(const IAABBox3D& aabb);

       void Repair();

//        explicit operator IAABBox3D(IAABBox3D v)
//       {
//           return IAABBox3D(v.Min, v.Max);
//       }
//       static explicit operator AxisAlignedBox3f(IAABBox3D v)
//       {
//           return AxisAlignedBox3f((Vector3f)v.Min, (Vector3f)v.Max);
//       }




       //-------------[ output operator ]------------------------
       /**
            * Output to stream operator
            * @param lhs Left hand side argument of operator (commonly ostream instance).
            * @param rhs Right hand side argument of operator.
            * @return Left hand side argument - the ostream object passed to operator.
            */
       friend std::ostream& operator<<(std::ostream& lhs, const IAABBox3D& rhs)
       {
           lhs << "Min[" << rhs.mMin[0] << "," << rhs.mMin[1] << "," << rhs.mMin[2] << "] ";
           lhs << "Max[" << rhs.mMax[0] << "," << rhs.mMax[1] << "," << rhs.mMax[2] << "] ";
           return lhs;
       }

       /**
            * Gets string representation.
            */
       std::string ToString() const
       {
           std::ostringstream oss;
           oss << *this;
           return oss.str();
       }


   };




}



#endif // IAABBOX3D_H
