#ifndef RPSETTINGS_H
#define RPSETTINGS_H

// Libraries
#include <assert.h>
#include <cstring>
#include <new>
#include <float.h>
#include <limits>

#include "../imaths.hpp"

namespace IEngine
{

typedef signed int   i32;
typedef signed short i16;
typedef signed char  i8;
typedef unsigned int   u32;
typedef unsigned long int lu32;
typedef unsigned short u16;
typedef unsigned char  u8;
typedef unsigned long long u64;

// Memory
//#define RP_NOT_USED(x) ((void)(x))
//#define RP_ASSERT(c) assert(c)
//#define RP_STATIC_ASSERT(c) static_assert(c)

// ------------------- Enumerations ------------------- //

/// Position correction technique used in the constraint solver (for joints).
/// BAUMGARTE_JOINTS : Faster but can be innacurate in some situations.
/// NON_LINEAR_GAUSS_SEIDEL : Slower but more precise. This is the option used by default.
enum JointsPositionCorrectionTechnique {BAUMGARTE_JOINTS, NON_LINEAR_GAUSS_SEIDEL};

/// Position correction technique used in the contact solver (for contacts)
/// BAUMGARTE_CONTACTS : Faster but can be innacurate and can lead to unexpected bounciness
///                      in some situations (due to error correction factor being added to
///                      the bodies momentum).
/// SPLIT_IMPULSES : A bit slower but the error correction factor is not added to the
///                 bodies momentum. This is the option used by default.
enum ContactsPositionCorrectionTechnique {BAUMGARTE_CONTACTS, SPLIT_IMPULSES};



/// Pi constant
const scalar PI = scalar(3.14159265);

/// 2*Pi constant
const scalar PI_TIMES_2 = scalar(6.28318530);



// The maximum number of manifolds that can be build
// for all contacts.
const u32 MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET = 3;   // Maximum number of contact manifolds in the set
const u32 CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS  = 3;   // N Number for the N x N subdivisions of the cubemap

// If this is equal to 4 then the contact generator
// will keep the hull-hull manifold clipped points up to 4 such that
// still creates a stable manifold to the solver. More points
// usually means better torque balance but can decrease
// the performance of the solver significantly.
// Therefore, keep this to 4 for greater performance.
const u32 MAX_CONTACT_POINTS_IN_MANIFOLD = 4;




/// Light Velocity c = 300000.kilometers / 1.second
//const scalar LIGHT_MAX_VELOCITY_C = scalar(300.0);

/// Smallest scalar value (negative)
const scalar SCALAR_SMALLEST = - std::numeric_limits<scalar>::max();

/// Maximum scalar value
const scalar SCALAR_LARGEST = std::numeric_limits<scalar>::max();




/// Default friction coefficient for a rigid body
const scalar DEFAULT_FRICTION_COEFFICIENT = scalar(0.3);

/// Default bounciness factor for a rigid body
const scalar DEFAULT_BOUNCINESS = scalar(0.0);

/// Default rolling resistance
const scalar DEFAULT_ROLLING_RESISTANCE = scalar(0.0);




/// Distance threshold for two contact points for a valid persistent contact (in meters)
const scalar PERSISTENT_CONTACT_DIST_THRESHOLD = scalar(0.03);

/// Velocity threshold for contact velocity restitution
const scalar RESTITUTION_VELOCITY_THRESHOLD = scalar(1.0);




/// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
const u32 DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;

/// Number of iterations when solving the position constraints of the Sequential Impulse technique
const u32 DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 10;




/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// inflated with a constant gap to allow the collision shape to move a little bit
/// without triggering a large modification of the tree which can be costly
const scalar DYNAMIC_TREE_AABB_GAP = scalar(0.1);

/// In the broad-phase collision detection (dynamic AABB tree), the AABBs are
/// also inflated in direction of the linear motion of the body by mutliplying the
/// followin constant with the linear velocity and the elapsed time between two frames.
const scalar DYNAMIC_TREE_AABB_LIN_GAP_MULTIPLIER = scalar(1.7);





/// Maximum number of contact manifolds in an overlapping pair that involves two
/// convex collision shapes.
const i32 NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE = 1;

/// Maximum number of contact manifolds in an overlapping pair that involves at
/// least one concave collision shape.
const i32 NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE = 3;


/// Maximum Collison Shape Type
const i32 NB_COLLISION_SHAPE_TYPES = 9;





///// Time (in seconds) that a body must stay still to be considered sleeping
const scalar DEFAULT_TIME_BEFORE_SLEEP = scalar(0.2);//1.0;

///// True if the spleeping technique is enabled
const bool  SLEEPING_ENABLED = false;





///// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
///// might enter sleeping mode.
const scalar DEFAULT_SLEEP_LINEAR_VELOCITY = scalar(0.01);

///// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
///// might enter sleeping mode
const scalar DEFAULT_SLEEP_ANGULAR_VELOCITY = scalar(3.0 * scalar(PI / 180.f));



/// Minimum for start damping
const scalar MINIMUM_FOR_DAPING = scalar(0.5);

///collision center point for the interpolation of (A+B) / 2.0
const bool   INTERPOLATION_CONTACT_POINTS = false;



const scalar LINEAR_SLOP = scalar(0.005); //5cm


#define RP_KiB(n) (1024 * n)
#define RP_MiB(n) (1024 * RP_KiB(n))
#define RP_GiB(n) (1024 * RP_MiB(n))


}


#endif // RPSETTINGS_H
