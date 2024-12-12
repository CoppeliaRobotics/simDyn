#pragma once

#include <RigidBodyDyn_base.h>
namespace Vx
{
class VxPart;
class VxUniverse;
class VxIntersectFilter;
} // namespace Vx

class CRigidBodyDyn : public CRigidBodyDyn_base
{
  public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    void init(CXShape* shape, bool forceStatic, bool forceNonRespondable);

    Vx::VxPart* getVortexRigidBody();

    // Following values accessed by callbacks:
    double vortex_skinThickness;
    bool vortex_autoSlip;
    double vortex_angularVelocityDamping;
    double vortex_autoAngularDampingTensionRatio;
    static void setVortexFilter(Vx::VxIntersectFilter* filter)
    {
        _sVortexIntersectFilter = filter;
    }

    C7Vector getInertiaFrameTransformation();
    C7Vector getShapeFrameTransformation();
    void reportVelocityToShape(double simulationTime);
    void handleAdditionalForcesAndTorques();
    void handleKinematicBody_step(double t, double cumulatedTimeStep);
    void handleKinematicBody_end();

  protected:
    Vx::VxPart* _vortexRigidBody;
    static Vx::VxIntersectFilter* _sVortexIntersectFilter;
};
