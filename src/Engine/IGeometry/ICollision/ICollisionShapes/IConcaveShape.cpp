#include "IConcaveShape.h"

namespace IEngine
{

// Constructor
ConcaveShape::ConcaveShape()
 : ICollisionShape(CONCAVE_SHAPE),
   mRaycastTestType(TriangleRaycastSide::FRONT)
{

}

}
