#ifndef IGIZMOMANIPULATOR_H
#define IGIZMOMANIPULATOR_H


#include "SceneMain.h"
#include "Engine/engine.hpp"

#include "Engine/ICommon/IMemory/IList.h"
#include "Engine/ICommon/IMemory/IMem.h"
#include "Engine/ICommon/IMemory/IStack.h"

#include "Engine/IComponent/IComponent.h"

#include "IGizmo/IUGizmo.h"



using namespace IEngine;


class IGizmoManipulator : public IRaycastCallback
{

private:

    IuGizmo::IGizmo::LOCATION      gizmo_status;
    std::auto_ptr<IuGizmo::IGizmo> gizmo;

    float mScaleSize;
    float mWidth;
    float mHeight;


public:

    Matrix4                       mTransformInitilization;
    std::map<int,Matrix4>         mSelectedIndexIDs;

    IProxyShape                  *mSelectedShape;
    int                           mSelectedIndexID;


private:


    // -------------------- Methods -------------------- //

    /// Private copy-constructor
    IGizmoManipulator(const IGizmoManipulator& world);

    /// Private assignment operator
    IGizmoManipulator& operator=(const IGizmoManipulator& world);


public:

    IGizmoManipulator();



    ~IGizmoManipulator()
    {
        gizmo.release();
    }

    void InitilizationDefault()
    {
        mSelectedIndexID = -1;
        gizmo = std::auto_ptr<IuGizmo::IGizmo>(IuGizmo::CreateMoveGizmo());
    }


    //========================================================//

    void realase()
    {
        if(gizmo.get())
        {
            mSelectedIndexID = -1;
            mSelectedShape = NULL;
        }
    }


    //========================================================//

    virtual scalar notifyRaycastHit(const IRaycastInfo& IRaycastInfo)
    {
       mTransformInitilization =
       IRaycastInfo.body->GetTransform().GetTransformMatrix();
       gizmo->SetEditMatrix( mTransformInitilization );
       mSelectedShape = IRaycastInfo.proxyShape;
       return IRaycastInfo.hitFraction;
    }



    void DisplayScale( float scale_size )
    {
        gizmo->SetDisplayScale(mScaleSize = scale_size);
    }

    void Resize(float width, float height)
    {
        mWidth  = width;
        mHeight = height;
        if (gizmo.get())
        {
            gizmo->SetScreenDimension( mWidth, mHeight );
        }
    }

    //========================================================//

    IRay BuildRayMouse(float mouse_x , float mouse_y) //const
    {
        Vector3 rayOrigin;
        Vector3 rayDir;
        gizmo->BuildRay( mouse_x , mouse_y , rayOrigin , rayDir );

        return IRay( rayOrigin , rayDir );
    }

    //========================================================//

    bool OnMouseDown( float mouse_x , float mouse_y ) const
    {
        return  gizmo->OnMouseDown( mouse_x, mouse_y ) && mSelectedIndexID >= 0;
    }


    void OnMouseMove( float mouse_x , float mouse_y ) const
    {
        if (gizmo.get())
        {
            gizmo->OnMouseMove( mouse_x , mouse_y );
        }
    }


    bool isSelectedIndex() const
    {
        return mSelectedIndexID >= 0;
    }

    //========================================================//

    void CheckMove()
    {
        gizmo = std::auto_ptr<IuGizmo::IGizmo>(IuGizmo::CreateMoveGizmo());
        gizmo->SetLocation( gizmo_status );
        gizmo->SetEditMatrix( mTransformInitilization );
        gizmo->SetScreenDimension( mWidth, mHeight );
        gizmo->SetDisplayScale(mScaleSize);

    }

    void CheckRotate()
    {
        gizmo = std::auto_ptr<IuGizmo::IGizmo>(IuGizmo::CreateRotateGizmo());
        gizmo->SetLocation( gizmo_status );
        gizmo->SetEditMatrix( mTransformInitilization );
        gizmo->SetScreenDimension( mWidth, mHeight );
        gizmo->SetDisplayScale(mScaleSize);
    }

    void CheckScale()
    {
        gizmo = std::auto_ptr<IuGizmo::IGizmo>(IuGizmo::CreateScaleGizmo());
        gizmo->SetLocation( gizmo_status );
        gizmo->SetEditMatrix( mTransformInitilization );
        gizmo->SetScreenDimension( mWidth, mHeight );
        gizmo->SetDisplayScale(mScaleSize);
    }

    void CheckLocal()
    {
        gizmo->SetLocation( gizmo_status = IuGizmo::IGizmo::LOCATE_LOCAL );
    }

    void CheckWorld()
    {
        gizmo->SetLocation( gizmo_status = IuGizmo::IGizmo::LOCATE_WORLD );
    }

    std::auto_ptr<IuGizmo::IGizmo> getGizmo()
    {
        return gizmo;
    }


    IuGizmo::IGizmo *GetGizmo() const;
    void SetGizmo_status(const IuGizmo::IGizmo::LOCATION &value);
};

#endif // IGIZMOMANIPULATOR_H
