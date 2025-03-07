#pragma once

#include <RigidBodyDyn_base.h>
#include <ode/ode.h>

class CRigidBodyDyn : public CRigidBodyDyn_base
{
  public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    void init(CXShape* shape, bool forceStatic, bool forceNonRespondable);

    dBodyID getOdeRigidBody();

    C7Vector getInertiaFrameTransformation();
    C7Vector getShapeFrameTransformation();
    void reportVelocityToShape(double simulationTime);
    void handleAdditionalForcesAndTorques();
    void handleKinematicBody_step(double t, double cumulatedTimeStep);
    void handleKinematicBody_end();

  protected:
    dBodyID _odeRigidBody;
};
