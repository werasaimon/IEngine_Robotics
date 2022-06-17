#include "IGizmoManipulator.h"



IGizmoManipulator::IGizmoManipulator()
{

}

IuGizmo::IGizmo *IGizmoManipulator::GetGizmo() const
{
   return gizmo.get();
}

void IGizmoManipulator::SetGizmo_status(const IuGizmo::IGizmo::LOCATION &value)
{
    gizmo_status = value;
}
