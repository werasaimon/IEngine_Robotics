#ifndef IGIZMOTRANSFORMMOVE_H
#define IGIZMOTRANSFORMMOVE_H

#include "IGizmoTransform.h"

namespace IuGizmo
{

    class CGizmoTransformMove : public CGizmoTransform
    {

        protected:

            enum MOVETYPE
            {
                MOVE_NONE,
                MOVE_X,
                MOVE_Y,
                MOVE_Z,
                MOVE_XY,
                MOVE_XZ,
                MOVE_YZ,
                MOVE_XYZ
            };

            MOVETYPE mMoveType;
            MOVETYPE mMoveTypePredict;
            IVector3 mMoveSnap;

        public:

                     CGizmoTransformMove();
            virtual ~CGizmoTransformMove();

            bool  isOnMouseDownSelected() const { return mMoveType != MOVETYPE::MOVE_NONE; }

            // return true if gizmo transform capture mouse
            virtual bool OnMouseDown(unsigned int x, unsigned int y);
            virtual void OnMouseMove(unsigned int x, unsigned int y);
            virtual void OnMouseUp(unsigned int x, unsigned int y);

            virtual void Draw();
            // snap

            virtual void SetSnap(float snapx, float snapy, float snapz)
            {
                mMoveSnap = IVector3(snapx, snapy, snapz);
            }

            virtual void SetSnap(const float snap)
            {
                mMoveSnap = IVector3(snap, snap, snap);
            }

            IVector3 GetMoveSnap()
            {
                return mMoveSnap;
            }

            virtual void ApplyTransform(IVector3& trans, bool bAbsolute);

        protected:

            bool GetOpType(MOVETYPE &type, unsigned int x, unsigned int y);

            int  RayTracePlaneOrtoAxis(const IVector3& RayOrigin , const IVector3& RayDir ,  const IVector3& Origin ,  const IVector3& Axis_A ,  const IVector3& Axis_B , IVector3 &inters);


    };
}

#endif // IGIZMOTRANSFORMMOVE_H
