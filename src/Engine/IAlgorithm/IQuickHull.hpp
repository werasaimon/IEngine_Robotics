#ifndef IQUICKCONVEXHULL_H
#define IQUICKCONVEXHULL_H

#include "QuickHull/ConvexHull.hpp"
#include "QuickHull/QuickHull.hpp"

namespace IEngine
{

namespace IAlgorithm
{

   //-------------------------------------------------------------//
    template<class T> using IQuickHull  = quickhull::QuickHull<T>;
    template<class T> using IConvexHull = quickhull::ConvexHull<T>;
   //-------------------------------------------------------------//

}

}

#endif // IQUICKCONVEXHULL_H
