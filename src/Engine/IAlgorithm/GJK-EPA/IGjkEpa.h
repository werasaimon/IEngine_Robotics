#ifndef IGJKEPA_H
#define IGJKEPA_H


#include "../../imaths.hpp"

namespace IEngine
{

namespace IAlgorithm
{


using namespace IMath;


// Config

/* GJK	*/
#define GJK_MAX_ITERATIONS	128
#define GJK_ACCURARY		(0.0001)
#define GJK_MIN_DISTANCE	(0.0001)
#define GJK_DUPLICATED_EPS	(0.0001)
#define GJK_SIMPLEX2_EPS	(0.0)
#define GJK_SIMPLEX3_EPS	(0.0)
#define GJK_SIMPLEX4_EPS	(0.0)

/* EPA	*/
#define EPA_MAX_VERTICES	64
#define EPA_MAX_FACES		(EPA_MAX_VERTICES*2)
#define EPA_MAX_ITERATIONS	255
#define EPA_ACCURACY		(0.0001)
#define EPA_FALLBACK		(10*EPA_ACCURACY)
#define EPA_PLANE_EPS		(0.0005)
#define EPA_INSIDE_EPS	    (0.004)



struct IGjkCollisionDescription
{
    Vector3	    mFirstDir;
    int	        mMaxGjkIterations;
    scalar	    mMaximumDistanceSquared;
    scalar	    mGjkRelError2;


    IGjkCollisionDescription()
    :mFirstDir(0,1,0),
     mMaxGjkIterations(100),
     mMaximumDistanceSquared(1e30f),
     mGjkRelError2(1.0e-6)
    {
    }


    virtual ~IGjkCollisionDescription()
    {
    }
};



struct	IGjkEpa
{
    struct	sResults
    {
        enum eStatus
        {
            Separated,		/* Shapes doesnt penetrate												*/
            Penetrating,	/* Shapes are penetrating												*/
            GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
            EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/

        } status;


        Vector3	mWitnesses[2];
        Vector3	mNormal;
        scalar	mDistance;
    };


    template <typename ConvexTemplate>
    static bool Distance(const ConvexTemplate& a,
                         const ConvexTemplate& b,
                         const Vector3& guess, IGjkEpa::sResults& results);



    template <typename ConvexTemplate>
    static bool  Penetration(const ConvexTemplate& a,
                             const ConvexTemplate& b,
                             const Vector3& guess, IGjkEpa::sResults& results);


    template <typename ConvexTemplate , typename DistanceInfoTemplate>
    static int	 ComputeGjkDistance(const ConvexTemplate& a,
                                    const ConvexTemplate& b,
                                    const IGjkCollisionDescription& colDesc,
                                          DistanceInfoTemplate& distInfo);



    template <typename  ConvexTemplate>
    static bool SignedDistance(const ConvexTemplate& a,
                               const ConvexTemplate& b,
                               const Vector3& guess, IGjkEpa::sResults& results);


    template <typename ConvexTemplate>
    static bool ComputeGjkEpaPenetrationDepth(const ConvexTemplate& a,
                                              const ConvexTemplate& b,
                                              const IGjkCollisionDescription& colDesc,
                                              Vector3& v,
                                              Vector3& wWitnessOnA,
                                              Vector3& wWitnessOnB);



};



namespace gjk_epa
{

// Shorthands
typedef unsigned int	U;
typedef unsigned char	U1;


//// MinkowskiDiff
template <typename ConvexTemplate>
struct	MinkowskiDiff
{
    const ConvexTemplate* mConvexAPtr;
    const ConvexTemplate* mConvexBPtr;

    bool				m_enableMargin;

    Transform				mWorld0;
    Transform				mWorld1;


    MinkowskiDiff(const ConvexTemplate& a, const ConvexTemplate& b)
     :mConvexAPtr(&a),
      mConvexBPtr(&b)
    {
    }

    void EnableMargin(bool enable)
    {
        m_enableMargin = enable;
    }

    SIMD_INLINE Vector3  Support0( const Vector3& dir ) const
    {
        Vector3 p  = mWorld0 * (mConvexAPtr->GetLocalSupportPointWithMargin( mWorld0.GetBasis().GetTranspose() * dir));
        //Vector3 p  = (mConvexAPtr->GetWorldSupportPointWithMargin(dir));
        return  p;
    }

    SIMD_INLINE Vector3	 Support1( const Vector3& dir ) const
    {
        Vector3 p  = mWorld1 * (mConvexBPtr->GetLocalSupportPointWithMargin( mWorld1.GetBasis().GetTranspose() * dir));
       //Vector3 p  = (mConvexBPtr->GetWorldSupportPointWithMargin(dir));
        return p;
    }

    SIMD_INLINE  Vector3	Support(const Vector3& d) const
    {
        return(Support0(d)-Support1(-d));
    }

    SIMD_INLINE  Vector3	Support(const Vector3& d,U index) const
    {
        if(index)
            return(Support1(d));
        else
            return(Support0(d));
    }
};


enum	eGjkStatus
{
    eGjkValid,
    eGjkInside,
    eGjkFailed
};

// GJK
template <typename ConvexTemplate>
struct	GJK
{
    /* Types		*/
    struct	sSV
    {
        Vector3	d,w;
    };
    struct	sSimplex
    {
        sSV*	    c[4];
        scalar	    p[4];
        U	    rank;
    };

    /* Fields		*/
    MinkowskiDiff<ConvexTemplate> mMinkowskiShape;

    Vector3	    mRay;
    scalar		    mDistance;
    sSimplex		mSimplices[2];
    sSV			    mStore[4];
    sSV*		    mFree[4];
    U			    mNfree;
    U			    mCurrent;
    sSimplex*		mSimplex;
    eGjkStatus      mStatus;
    /* Methods		*/

    GJK(const ConvexTemplate& a, const ConvexTemplate& b)
        :mMinkowskiShape(a,b)
    {
        Initialize();
    }
    void Initialize()
    {
        mRay		=	Vector3	(0,0,0);
        mNfree		=	0;
        mStatus	        =	eGjkFailed;
        mCurrent	=	0;
        mDistance	=	0;
    }
    eGjkStatus	Evaluate(const MinkowskiDiff<ConvexTemplate>& shapearg,const Vector3 &guess)
    {
        U			iterations=0;
        scalar	sqdist=0;
        scalar	alpha=0;
        Vector3	lastw[4];
        U			clastw=0;
        /* Initialize solver		*/
        mFree[0]			=	&mStore[0];
        mFree[1]			=	&mStore[1];
        mFree[2]			=	&mStore[2];
        mFree[3]			=	&mStore[3];
        mNfree				=	4;
        mCurrent			=	0;
        mStatus			=	eGjkValid;
        mMinkowskiShape				=	shapearg;
        mDistance			=	0;
        /* Initialize simplex		*/
        mSimplices[0].rank	=	0;
        mRay				=	guess;
        const scalar	sqrl=	mRay.LengthSquare();
        appendvertice(mSimplices[0],sqrl>0?-mRay:Vector3	(1,0,0));
        mSimplices[0].p[0]	=	1;
        mRay				=	mSimplices[0].c[0]->w;
        sqdist				=	sqrl;
        lastw[0]			=
                lastw[1]			=
                lastw[2]			=
                lastw[3]			=	mRay;
        /* Loop						*/
        do	{
            const U		next=1-mCurrent;
            sSimplex&	cs=mSimplices[mCurrent];
            sSimplex&	ns=mSimplices[next];
            /* Check zero							*/
            const scalar	rl=mRay.Length();
            if(rl<GJK_MIN_DISTANCE)
            {/* Touching or inside				*/
                mStatus=eGjkInside;
                break;
            }
            /* Append new vertice in -'v' direction	*/
            appendvertice(cs,-mRay);
            const Vector3	&	w=cs.c[cs.rank-1]->w;
            bool				found=false;
            for(U i=0;i<4;++i)
            {
                if((w-lastw[i]).LengthSquare()<GJK_DUPLICATED_EPS)
                { found=true;break; }
            }
            if(found)
            {/* Return old simplex				*/
                removevertice(mSimplices[mCurrent]);
                break;
            }
            else
            {/* Update lastw					*/
                lastw[clastw=(clastw+1)&3]=w;
            }
                        /* Check for termination				*/
                        const scalar	omega=Dot(mRay,w)/rl;
                        alpha=IMax(omega,alpha);
                        if(((rl-alpha)-(GJK_ACCURARY*rl))<=0)
                {/* Return old simplex				*/
                    removevertice(mSimplices[mCurrent]);
                    break;
                }
                /* Reduce simplex						*/
                scalar	weights[4];
                U			mask=0;
                switch(cs.rank)
                {
                case	2:	sqdist=projectorigin(	cs.c[0]->w,
                            cs.c[1]->w,
                            weights,mask);break;
                case	3:	sqdist=projectorigin(	cs.c[0]->w,
                            cs.c[1]->w,
                            cs.c[2]->w,
                            weights,mask);break;
                case	4:	sqdist=projectorigin(	cs.c[0]->w,
                            cs.c[1]->w,
                            cs.c[2]->w,
                            cs.c[3]->w,
                            weights,mask);break;
                }
                if(sqdist>=0)
                {/* Valid	*/
                    ns.rank		=	0;
                    mRay		=	Vector3	(0,0,0);
                    mCurrent	=	next;
                    for(U i=0,ni=cs.rank;i<ni;++i)
                    {
                        if(mask&(1<<i))
                        {
                            ns.c[ns.rank]		=	cs.c[i];
                            ns.p[ns.rank++]		=	weights[i];
                            mRay		       +=	cs.c[i]->w*weights[i];
                        }
                        else
                        {
                            mFree[mNfree++]	=	cs.c[i];
                        }
                    }
                    if(mask==15) mStatus=eGjkInside;
                }
                else
                {/* Return old simplex				*/
                    removevertice(mSimplices[mCurrent]);
                    break;
                }
                mStatus=((++iterations)<GJK_MAX_ITERATIONS)?mStatus:eGjkFailed;
            } while(mStatus==eGjkValid);
            mSimplex=&mSimplices[mCurrent];
            switch(mStatus)
            {
            case	eGjkValid:	mDistance=mRay.Length();break;
            case	eGjkInside:	mDistance=0;break;
            default:
            {
            }
            }
            return(mStatus);
        }
        bool	EncloseOrigin()
        {
            switch(mSimplex->rank)
            {
            case	1:
            {
                for(U i=0;i<3;++i)
                {
                    Vector3		axis=Vector3	(0,0,0);
                    axis[i]=1;
                    appendvertice(*mSimplex, axis);
                    if(EncloseOrigin())	return(true);
                    removevertice(*mSimplex);
                    appendvertice(*mSimplex,-axis);
                    if(EncloseOrigin())	return(true);
                    removevertice(*mSimplex);
                }
            }
                break;
            case	2:
            {
                const Vector3	d=mSimplex->c[1]->w-mSimplex->c[0]->w;
                for(U i=0;i<3;++i)
                {
                    Vector3		axis=Vector3	(0,0,0);
                    axis[i]=1;
                    const Vector3	p=Cross(d,axis);
                    if(p.LengthSquare()>0)
                    {
                        appendvertice(*mSimplex, p);
                        if(EncloseOrigin())	return(true);
                        removevertice(*mSimplex);
                        appendvertice(*mSimplex,-p);
                        if(EncloseOrigin())	return(true);
                        removevertice(*mSimplex);
                    }
                }
            }
                break;
            case	3:
            {
                const Vector3	n=Cross(mSimplex->c[1]->w-mSimplex->c[0]->w,
                                        mSimplex->c[2]->w-mSimplex->c[0]->w);
                if(n.LengthSquare()>0)
                {
                    appendvertice(*mSimplex,n);
                    if(EncloseOrigin())	return(true);
                    removevertice(*mSimplex);
                    appendvertice(*mSimplex,-n);
                    if(EncloseOrigin())	return(true);
                    removevertice(*mSimplex);
                }
            }
                break;
            case	4:
            {
                if(IAbs(det( mSimplex->c[0]->w-mSimplex->c[3]->w,
                             mSimplex->c[1]->w-mSimplex->c[3]->w,
                             mSimplex->c[2]->w-mSimplex->c[3]->w))>0)
                    return(true);
            }
                break;
            }
            return(false);
        }
        /* Internals	*/
        void	getsupport(const Vector3	& d,sSV& sv) const
        {
            sv.d	=	d/d.Length();
            sv.w	=	mMinkowskiShape.Support(sv.d);
        }
        void				removevertice(sSimplex& simplex)
        {
            mFree[mNfree++]=simplex.c[--simplex.rank];
        }
        void				appendvertice(sSimplex& simplex,const Vector3	& v)
        {
            simplex.p[simplex.rank]=0;
            simplex.c[simplex.rank]=mFree[--mNfree];
            getsupport(v,*simplex.c[simplex.rank++]);
        }
        static scalar	det(const Vector3	& a,const Vector3	& b,const Vector3	& c)
        {
            return( a.y*b.z*c.x+a.z*b.x*c.y-
                    a.x*b.z*c.y-a.y*b.x*c.z+
                    a.x*b.y*c.z-a.z*b.y*c.x);
        }
        static scalar	projectorigin(	const Vector3	& a,
                                        const Vector3	& b,
                                         scalar* w,U& m)
        {
            const Vector3	d=b-a;
            const scalar	l=d.LengthSquare();
            if(l>GJK_SIMPLEX2_EPS)
            {
                const scalar	t(l>0?-Dot(a,d)/l:0);
                if(t>=1)		{ w[0]=0;w[1]=1;m=2;return(b.LengthSquare()); }
                else if(t<=0)	{ w[0]=1;w[1]=0;m=1;return(a.LengthSquare()); }
                else			{ w[0]=1-(w[1]=t);m=3;return((a+d*t).LengthSquare()); }
            }
            return(-1);
        }
        static scalar	projectorigin(	const Vector3	& a,
                                        const Vector3	& b,
                                        const Vector3	& c,
                                         scalar* w,U& m)
        {
            static const U		imd3[]={1,2,0};
            const Vector3*	    vt[]={&a,&b,&c};
            const Vector3		dl[]={a-b,b-c,c-a};
            const Vector3		n=Cross(dl[0],dl[1]);
            const scalar		l=n.LengthSquare();
            if(l>GJK_SIMPLEX3_EPS)
            {
                scalar	mindist=-1;
                scalar	subw[2]={0.f,0.f};
                U			subm(0);
                for(U i=0;i<3;++i)
                {
                    if(Dot(*vt[i],Cross(dl[i],n))>0)
                    {
                        const U			j=imd3[i];
                        const scalar	subd(projectorigin(*vt[i],*vt[j],subw,subm));
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist		=	subd;
                            m			=	static_cast<U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
                            w[i]		=	subw[0];
                            w[j]		=	subw[1];
                            w[imd3[j]]	=	0;
                        }
                    }
                }
                if(mindist<0)
                {
                    const scalar	d=Dot(a,n);
                    const scalar	s=ISqrt(l);
                    const Vector3	p=n*(d/l);
                    mindist	=	p.LengthSquare();
                    m		=	7;
                    w[0]	=	(Cross(dl[1],b-p)).Length()/s;
                    w[1]	=	(Cross(dl[2],c-p)).Length()/s;
                    w[2]	=	1-(w[0]+w[1]);
                }
                return(mindist);
            }
            return(-1);
        }
        static scalar	 projectorigin(	const Vector3	& a,
                                        const Vector3	& b,
                                        const Vector3	& c,
                                        const Vector3	& d,
                                        scalar* w,U& m)
        {
            static const U		imd3[]={1,2,0};
            const Vector3	*	vt[]={&a,&b,&c,&d};
            const Vector3		dl[]={a-d,b-d,c-d};
            const scalar		vl=det(dl[0],dl[1],dl[2]);
            const bool			ng=(vl*Dot(a,Cross(b-c,a-b)))<=0;
            if(ng&&(IAbs(vl)>GJK_SIMPLEX4_EPS))
            {
                scalar	mindist=-1;
                scalar	subw[3]={0.f,0.f,0.f};
                U			subm(0);
                for(U i=0;i<3;++i)
                {
                    const U			j=imd3[i];
                    const scalar	s=vl*Dot(d,Cross(dl[i],dl[j]));
                    if(s>0)
                    {
                        const scalar	subd=projectorigin(*vt[i],*vt[j],d,subw,subm);
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist		=	subd;
                            m			=	static_cast<U>((subm&1?1<<i:0)+
                                                           (subm&2?1<<j:0)+
                                                           (subm&4?8:0));
                            w[i]		=	subw[0];
                            w[j]		=	subw[1];
                            w[imd3[j]]	=	0;
                            w[3]		=	subw[2];
                        }
                    }
                }
                if(mindist<0)
                {
                    mindist	=	0;
                    m		=	15;
                    w[0]	=	det(c,b,d)/vl;
                    w[1]	=	det(a,c,d)/vl;
                    w[2]	=	det(b,a,d)/vl;
                    w[3]	=	1-(w[0]+w[1]+w[2]);
                }
                return(mindist);
            }
            return(-1);
        }
    };



    enum	eEpaStatus
    {
        eEpaValid,
        eEpaTouching,
        eEpaDegenerated,
        eEpaNonConvex,
        eEpaInvalidHull,
        eEpaOutOfFaces,
        eEpaOutOfVertices,
        eEpaAccuraryReached,
        eEpaFallBack,
        eEpaFailed
    };


    // EPA
    template <typename ConvexTemplate>
    struct	EPA
    {
        /* Types		*/

        struct	sFace
        {
            Vector3	n;
            scalar	d;

            typename GJK<ConvexTemplate>::sSV*	c[3];

            sFace*		f[3];
            sFace*		l[2];
            U1			e[3];
            U1			pass;
        };

        struct	sList
        {
            sFace*		root;
            U			count;
            sList() : root(0),count(0)	{}
        };

        struct	sHorizon
        {
            sFace*		cf;
            sFace*		ff;
            U			nf;
            sHorizon() : cf(0),ff(0),nf(0)	{}
        };

        /* Fields		*/

        typename GJK<ConvexTemplate>::sSimplex	mResult;

        eEpaStatus	mStatus;
        Vector3		mNormal;
        scalar		mDepth;

        typename GJK<ConvexTemplate>::sSV	m_sv_store[EPA_MAX_VERTICES];
        sFace		                    	m_fc_store[EPA_MAX_FACES];

        U			mNextsv;
        sList			mHull;
        sList			mStock;
        /* Methods		*/
        EPA()
        {
            Initialize();
        }


        static inline void		bind(sFace* fa,U ea,sFace* fb,U eb)
        {
            fa->e[ea]=(U1)eb;fa->f[ea]=fb;
            fb->e[eb]=(U1)ea;fb->f[eb]=fa;
        }
        static inline void		append(sList& list,sFace* face)
        {
            face->l[0]	=	0;
            face->l[1]	=	list.root;
            if(list.root) list.root->l[0]=face;
            list.root	=	face;
            ++list.count;
        }
        static inline void		remove(sList& list,sFace* face)
        {
            if(face->l[1]) face->l[1]->l[0]=face->l[0];
            if(face->l[0]) face->l[0]->l[1]=face->l[1];
            if(face==list.root) list.root=face->l[1];
            --list.count;
        }


        void	Initialize()
        {
            mStatus	=	eEpaFailed;
            mNormal	=	Vector3(0,0,0);
            mDepth		=	0;
            mNextsv	=	0;
            for(U i=0;i<EPA_MAX_FACES;++i)
            {
                append(mStock,&m_fc_store[EPA_MAX_FACES-i-1]);
            }
        }


        eEpaStatus	Evaluate(GJK<ConvexTemplate>& gjk,const Vector3& guess)
        {
            typename GJK<ConvexTemplate>::sSimplex&	simplex=*gjk.mSimplex;
            if((simplex.rank>1)&&gjk.EncloseOrigin())
            {

                /* Clean up				*/
                while(mHull.root)
                {
                    sFace*	f = mHull.root;
                    remove(mHull,f);
                    append(mStock,f);
                }
                mStatus	=	eEpaValid;
                mNextsv	=	0;
                /* Orient simplex		*/
                if(gjk.det(	simplex.c[0]->w-simplex.c[3]->w,
                            simplex.c[1]->w-simplex.c[3]->w,
                            simplex.c[2]->w-simplex.c[3]->w)<0)
                {
                    ISwap(simplex.c[0],simplex.c[1]);
                    ISwap(simplex.p[0],simplex.p[1]);
                }
                /* Build initial hull	*/
                sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
                                 newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
                                 newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
                                 newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};


                if(mHull.count==4)
                {
                    sFace*		best=findbest();
                    sFace		outer=*best;
                    U			pass=0;
                    U			iterations=0;
                    bind(tetra[0],0,tetra[1],0);
                    bind(tetra[0],1,tetra[2],0);
                    bind(tetra[0],2,tetra[3],0);
                    bind(tetra[1],1,tetra[3],2);
                    bind(tetra[1],2,tetra[2],1);
                    bind(tetra[2],2,tetra[3],1);
                    mStatus=eEpaValid;
                    for(;iterations<EPA_MAX_ITERATIONS;++iterations)
                    {
                        if(mNextsv<EPA_MAX_VERTICES)
                        {
                            sHorizon		horizon;
                            typename GJK<ConvexTemplate>::sSV*			w=&m_sv_store[mNextsv++];
                            bool			valid=true;
                            best->pass	=	(U1)(++pass);
                            gjk.getsupport(best->n,*w);
                            const scalar	wdist=Dot(best->n,w->w)-best->d;
                            if(wdist>EPA_ACCURACY)
                            {
                                for(U j=0;(j<3)&&valid;++j)
                                {
                                    valid&=expand(	pass,w,
                                                    best->f[j],best->e[j],
                                                    horizon);
                                }
                                if(valid&&(horizon.nf>=3))
                                {
                                    bind(horizon.cf,1,horizon.ff,2);
                                    remove(mHull,best);
                                    append(mStock,best);
                                    best=findbest();
                                    outer=*best;
                                } else { mStatus=eEpaInvalidHull;break; }
                            } else { mStatus=eEpaAccuraryReached;break; }
                        } else { mStatus=eEpaOutOfVertices;break; }
                    }
                    const Vector3	projection=outer.n*outer.d;
                    mNormal	        =	outer.n;
                    mDepth		    =	outer.d;
                    mResult.rank	=	3;
                    mResult.c[0]	=	outer.c[0];
                    mResult.c[1]	=	outer.c[1];
                    mResult.c[2]	=	outer.c[2];
                    mResult.p[0]	=	Cross(	outer.c[1]->w-projection, outer.c[2]->w-projection).Length();
                    mResult.p[1]	=	Cross(	outer.c[2]->w-projection, outer.c[0]->w-projection).Length();
                    mResult.p[2]	=	Cross(	outer.c[0]->w-projection, outer.c[1]->w-projection).Length();
                    const scalar	sum=mResult.p[0]+mResult.p[1]+mResult.p[2];
                    mResult.p[0]	/=	sum;
                    mResult.p[1]	/=	sum;
                    mResult.p[2]	/=	sum;
                    return(mStatus);
                }
            }
            /* Fallback		*/
            mStatus	=	eEpaFallBack;
            mNormal	=	-guess;
            const scalar	nl=mNormal.Length();
            if(nl>0)
                mNormal	=	mNormal/nl;
            else
                mNormal	=	Vector3(1,0,0);
            mDepth	=	0;
            mResult.rank=1;
            mResult.c[0]=simplex.c[0];
            mResult.p[0]=1;
            return(mStatus);
        }


        bool getedgedist(sFace* face, typename GJK<ConvexTemplate>::sSV* a, typename GJK<ConvexTemplate>::sSV* b, scalar& dist)
        {
            const Vector3 ba = b->w - a->w;
            const Vector3 n_ab = Cross(ba, face->n); // Outward facing edge normal direction, on triangle plane
            const scalar a_dot_nab = Dot(a->w, n_ab); // Only care about the sign to determine inside/outside, so not normalization required

            if(a_dot_nab < 0)
            {
                // Outside of edge a->b
                const scalar ba_l2 = ba.LengthSquare();
                const scalar a_dot_ba = Dot(a->w, ba);
                const scalar b_dot_ba = Dot(b->w, ba);

                if(a_dot_ba > 0)
                {
                    // Pick distance vertex a
                    dist = a->w.Length();
                }
                else if(b_dot_ba < 0)
                {
                    // Pick distance vertex b
                    dist = b->w.Length();
                }
                else
                {
                    // Pick distance to edge a->b
                    const scalar a_dot_b = Dot(a->w, b->w);
                    dist = ISqrt(IMax((a->w.LengthSquare() * b->w.LengthSquare() - a_dot_b * a_dot_b) / ba_l2, (scalar)0));
                }

                return true;
            }

            return false;
        }



        sFace*	newface(typename GJK<ConvexTemplate>::sSV* a,typename GJK<ConvexTemplate>::sSV* b,typename GJK<ConvexTemplate>::sSV* c,bool forced)
        {
            if(mStock.root)
            {
                sFace*	face=mStock.root;
                remove(mStock,face);
                append(mHull,face);
                face->pass	=	0;
                face->c[0]	=	a;
                face->c[1]	=	b;
                face->c[2]	=	c;
                face->n		=	Cross(b->w-a->w,c->w-a->w);
                const scalar	l=face->n.Length();
                const bool		v=l>EPA_ACCURACY;

                if(v)
                {
                    if(!(getedgedist(face, a, b, face->d) ||
                         getedgedist(face, b, c, face->d) ||
                         getedgedist(face, c, a, face->d)))
                    {
                        // Origin projects to the interior of the triangle
                        // Use distance to triangle plane
                        face->d = Dot(a->w, face->n) / l;
                    }

                    face->n /= l;
                    if(forced || (face->d >= -EPA_PLANE_EPS))
                    {
                        return face;
                    }
                    else
                        mStatus=eEpaNonConvex;
                }
                else
                    mStatus=eEpaDegenerated;

                remove(mHull, face);
                append(mStock, face);
                return 0;

            }
            mStatus = mStock.root ? eEpaOutOfVertices : eEpaOutOfFaces;
            return 0;
        }



        sFace*	findbest()
        {
            sFace*		minf=mHull.root;
            scalar	    mind=minf->d*minf->d;
            for(sFace* f=minf->l[1];f;f=f->l[1])
            {
                const scalar	sqd=f->d*f->d;
                if(sqd<mind)
                {
                    minf=f;
                    mind=sqd;
                }
            }
            return(minf);
        }



        bool  expand(U pass,typename GJK<ConvexTemplate>::sSV* w,sFace* f,U e,sHorizon& horizon)
        {
            static const U	i1m3[]={1,2,0};
            static const U	i2m3[]={2,0,1};
            if(f->pass!=pass)
            {
                const U	e1=i1m3[e];
                if((Dot(f->n,w->w)-f->d)<-EPA_PLANE_EPS)
                {
                    sFace*	nf=newface(f->c[e1],f->c[e],w,false);
                    if(nf)
                    {
                        bind(nf,0,f,e);
                        if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
                        horizon.cf=nf;
                        ++horizon.nf;
                        return(true);
                    }
                }
                else
                {
                    const U	e2=i2m3[e];
                    f->pass		=	(U1)pass;
                    if(	expand(pass,w,f->f[e1],f->e[e1],horizon)&&
                        expand(pass,w,f->f[e2],f->e[e2],horizon))
                    {
                        remove(mHull,f);
                        append(mStock,f);
                        return(true);
                    }
                }
            }
            return(false);
        }

    };



    template <typename ConvexTemplate>
    static void	Initialize( const ConvexTemplate& a,
                            const ConvexTemplate& b,
                            IGjkEpa::sResults& results,
                            MinkowskiDiff<ConvexTemplate>& shape)
    {
        /* Results		*/
        results.mWitnesses[0]	=
        results.mWitnesses[1]	=	Vector3(0,0,0);
        results.status		=	IGjkEpa::sResults::Separated;
        /* Shape		*/


        shape.mWorld1 =	b.GetWorldTransform();
        shape.mWorld0 =	a.GetWorldTransform();



    }

    //===========================================================//

}




}

}
#endif // IGJKEPA_H
