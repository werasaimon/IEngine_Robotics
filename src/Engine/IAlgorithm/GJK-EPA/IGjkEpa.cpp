#include "IGjkEpa.h"


namespace IEngine
{

namespace IAlgorithm
{


////----------------------- Api -----------------------------//

using namespace gjk_epa;

template<typename ConvexTemplate>
inline bool IGjkEpa::Distance(const ConvexTemplate &a, const ConvexTemplate &b, const Vector3 &guess, IGjkEpa::sResults &results)
{

    MinkowskiDiff<ConvexTemplate>	 shape(a,b);
    Initialize(a,b,results,shape);

    GJK<ConvexTemplate>				 gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,guess);

    if(gjk_status==eGjkValid)
    {
        Vector3	w0=Vector3(0,0,0);
        Vector3	w1=Vector3(0,0,0);
        for(U i=0;i<gjk.mSimplex->rank;++i)
        {
            const scalar	p=gjk.mSimplex->p[i];

            w0+=shape.Support( gjk.mSimplex->c[i]->d,0)*p;
            w1+=shape.Support(-gjk.mSimplex->c[i]->d,1)*p;
        }
        results.mWitnesses[0]	=  w0;
        results.mWitnesses[1]	=  w1;


        results.mNormal			=	w0-w1;
        results.mDistance		=	results.mNormal.Length();
        results.mNormal		   /=	results.mDistance>GJK_MIN_DISTANCE?results.mDistance:1;
        return(true);
    }
    else
    {
        results.status	=	gjk_status==eGjkInside?
                                        IGjkEpa::sResults::Penetrating	:
                                        IGjkEpa::sResults::GJK_Failed	;
        return(false);
    }
}




template<typename ConvexTemplate>
inline bool IGjkEpa::Penetration(const ConvexTemplate &a, const ConvexTemplate &b, const Vector3 &guess, IGjkEpa::sResults &results)
{
    MinkowskiDiff<ConvexTemplate> shape(a,b);
    Initialize(a,b,results,shape);

    GJK<ConvexTemplate>	gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,-guess);

    switch(gjk_status)
    {
    case	eGjkInside:
    {
        EPA<ConvexTemplate> epa;
        eEpaStatus	epa_status=epa.Evaluate(gjk,-guess);
        if(epa_status!=eEpaFailed)
        {
            Vector3	w0=Vector3(0,0,0);
            for(U i=0;i<epa.mResult.rank;++i)
            {
                w0+=shape.Support(epa.mResult.c[i]->d,0)*epa.mResult.p[i];
            }
            results.status = IGjkEpa::sResults::Penetrating;

            results.mWitnesses[0]	=  w0;
            results.mWitnesses[1]	= (w0-epa.mNormal*epa.mDepth);

            results.mNormal		=	-epa.mNormal;
            results.mDistance		=	-epa.mDepth;

            return(true);

        } else results.status=IGjkEpa::sResults::EPA_Failed;
    }
        break;

    case	eGjkFailed:

        results.status=IGjkEpa::sResults::GJK_Failed;

        break;

    default:
    {
    }
    }
    return(false);
}



template<typename ConvexTemplate, typename DistanceInfoTemplate>
inline int IGjkEpa::ComputeGjkDistance(const ConvexTemplate &a, const ConvexTemplate &b, const IGjkCollisionDescription &colDesc, DistanceInfoTemplate &distInfo)
{

    IGjkEpa::sResults results;
    Vector3 guess = colDesc.mFirstDir;

    bool isSeparated = GJKDistance(a , b , guess , results);
    if (isSeparated)
    {
        distInfo->mDistance   = results.mDistance;
        distInfo->mPointOnA  = results.mWitnesses[0];
        distInfo->mPointOnB  = results.mWitnesses[1];
        distInfo->mNormalBtoA = results.mNormal;
        return 0;
    }

    return -1;

}



template<typename ConvexTemplate>
inline bool IGjkEpa::SignedDistance(const ConvexTemplate &a, const ConvexTemplate &b, const Vector3 &guess, IGjkEpa::sResults &results)
{
    if(!Distance(a,b,guess,results))
    {
        return(Penetration(a,b,guess,results));
    }
    else
    {
        return(true);
    }
}


template <typename ConvexTemplate>
inline bool IGjkEpa::ComputeGjkEpaPenetrationDepth(const ConvexTemplate& a,
                                                         const ConvexTemplate& b,
                                                         const IGjkCollisionDescription& colDesc,
                                                         Vector3& v,
                                                         Vector3& wWitnessOnA,
                                                         Vector3& wWitnessOnB)
{


    Vector3	guessVector(b.GetWorldTransform().GetPosition()-
                        a.GetWorldTransform().GetPosition());//?? why not use the GJK input?

    if(colDesc.mFirstDir != Vector3(1.0,1.0,1.0)) guessVector = colDesc.mFirstDir;

    IGjkEpa::sResults	results;
    if(IGjkEpa::Penetration(a,b,guessVector,results))
    {
        //	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,Vector3(255,0,0));
        //resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
        wWitnessOnA = results.mWitnesses[0];
        wWitnessOnB = results.mWitnesses[1];

        v = results.mNormal;
        return true;
    }
    else
    {
        if(IGjkEpa::Distance(a,b,guessVector,results))
        {
            wWitnessOnA = results.mWitnesses[0];
            wWitnessOnB = results.mWitnesses[1];

            v = results.mNormal;

            Vector3 L =  wWitnessOnA - wWitnessOnB;
            return L.Length() < 0.02f;
        }
    }
    return false;
}




/* Symbols cleanup		*/

#undef GJK_MAX_ITERATIONS
#undef GJK_ACCURARY
#undef GJK_MIN_DISTANCE
#undef GJK_DUPLICATED_EPS
#undef GJK_SIMPLEX2_EPS
#undef GJK_SIMPLEX3_EPS
#undef GJK_SIMPLEX4_EPS

#undef EPA_MAX_VERTICES
#undef EPA_MAX_FACES
#undef EPA_MAX_ITERATIONS
#undef EPA_ACCURACY
#undef EPA_FALLBACK
#undef EPA_PLANE_EPS
#undef EPA_INSIDE_EPS

}

}
