#pragma once

#include "RigidBodyDyn_base.h"
#include "btBulletDynamicsCommon.h"

class CRigidBodyDyn : public CRigidBodyDyn_base
{
public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    void init(CXShape* shape,bool forceStatic,bool forceNonRespondable);


    btRigidBody* getBtRigidBody();

    C7Vector getInertiaFrameTransformation();
    C7Vector getShapeFrameTransformation();
    void reportVelocityToShape(float simulationTime);
    void handleAdditionalForcesAndTorques();
    void handleKinematicBody_step(float t,float cumulatedTimeStep);
    void handleKinematicBody_end();

protected:    
    btRigidBody* _rigidBody;
};
