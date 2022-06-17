#include "IIntegrateUtil.h"


namespace IEngine
{

    IIntegrateUtil::IIntegrateUtil()
    {

    }

    Transform IIntegrateUtil::IntegrateTransform(const Transform &curTrans, const Vector3 &linvel, const Vector3 &angvel, scalar timeStep)
    {
        //Exponential map
        //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
        Vector3 axis;
        scalar fAngle = angvel.Length();

        if (fAngle < scalar(0.001))
        {
            // use Taylor's expansions of sync function
            axis = angvel * (scalar(0.5) * timeStep -
                    (timeStep * timeStep * timeStep) *
                    (scalar(0.020833333333)) * fAngle * fAngle);
        }
        else
        {
            axis = angvel * (sin(scalar(0.5) * fAngle * timeStep) / fAngle);
        }


        Quaternion dorn(axis, cos(fAngle * timeStep * scalar(0.5)));
        Quaternion predictedOrn = dorn * Quaternion(curTrans.GetBasis());
        predictedOrn.Normalize();

        return Transform( curTrans.GetPosition() + linvel * timeStep ,  predictedOrn.GetRotMatrix(), curTrans.GetTime() );

    }

    void IIntegrateUtil::Damping(Vector3 &Velocity, const scalar &min_damping, const scalar &damping)
    {
        if( Velocity.LengthSquare()  < min_damping )
        {
            if( Velocity.Length() > damping )
            {
                Velocity -= ( Velocity.GetUnit() * damping);
            }
            else
            {
                Velocity  = Vector3(0,0,0);
            }
        }
    }




}

