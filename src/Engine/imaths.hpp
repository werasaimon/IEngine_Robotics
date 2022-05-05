#ifndef IMATHS_HPP
#define IMATHS_HPP

#include "IMath/IFunc.h"
#include "IMath/IVector2D.h"
#include "IMath/IVector3D.h"
#include "IMath/IVector4D.h"
#include "IMath/ILorentzVector.h"
#include "IMath/IMatrix.h"
#include "IMath/IMatrix2x2.h"
#include "IMath/IMatrix3x3.h"
#include "IMath/IMatrix4x4.h"
#include "IMath/IQuaternion.h"
#include "IMath/IOctonion.h"
#include "IMath/IComplex.h"
#include "IMath/ITransform.h"
#include "IMath/IAffineTransform.h"
#include "IMath/ISpherical.h"

#include "IMath/IVector.h"
#include "IMath/IScalarType.h"
#include "IMath/IAlgebra.h"


#include <limits>


namespace IEngine
{

typedef float scalar;


typedef IMath::IVector2D<scalar>        Vector2;
typedef IMath::IVector3D<scalar>        Vector3;
typedef IMath::IVector4D<scalar>        Vector4;
typedef IMath::ILorentzVector<scalar>   LorentzVector;
typedef IMath::IMatrix2x2<scalar>       Matrix2;
typedef IMath::IMatrix3x3<scalar>       Matrix3;
typedef IMath::IMatrix4x4<scalar>       Matrix4;
typedef IMath::IQuaternion<scalar>      Quaternion;
typedef IMath::ITransform<scalar>       Transform;
typedef IMath::IAffineTransform<scalar> AffineTransform;


typedef IMath::IVector2D<int> Index2i;
typedef IMath::IVector3D<int> Index3i;
typedef IMath::IVector4D<int> Index4i;

typedef IMath::IVector2D<int> Vector2i;
typedef IMath::IVector3D<int> Vector3i;
typedef IMath::IVector4D<int> Vector4i;

const scalar DECIMAL_SMALLEST = -std::numeric_limits<scalar>::max();
const scalar DECIMAL_LARGEST  =  std::numeric_limits<scalar>::max();

}




#endif // IMATHS_HPP
