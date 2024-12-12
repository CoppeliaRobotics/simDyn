#pragma once

#include <utils.h>
#include <CollShapeDyn.h>
#include <simMath/7Vector.h>

class CRigidBodyDyn_base
{
  public:
    CRigidBodyDyn_base();
    virtual ~CRigidBodyDyn_base();

    virtual void init(CXShape* shape, bool forceStatic, bool forceNonRespondable);

    virtual C7Vector getInertiaFrameTransformation();
    virtual C7Vector getShapeFrameTransformation();
    virtual void reportVelocityToShape(sReal simulationTime);
    virtual void handleAdditionalForcesAndTorques();
    virtual void handleKinematicBody_step(sReal t, sReal cumulatedTimeStep);
    virtual void handleKinematicBody_end();

    int getShapeHandle() const;
    CXShape* getShape() const;
    bool isStatic() const;
    bool isNeverRespondable() const;
    CCollShapeDyn* getCollisionShapeDyn() const;
    void reportConfigurationToShape(sReal simulationTime);
    void handleKinematicBody_init(sReal dt);

  protected:
    int _shapeHandle; // rigid body id is the same
    CXShape* _shape;
    CCollShapeDyn* _collisionShapeDyn;
    bool _isStatic;
    bool _isNeverRespondable; // if true, will never be respondable. If false, can be dyn. enabled/disabled
    bool _applyBodyToShapeTransf_kinematicBody;

    // Unscaled:
    C7Vector _bodyStart_kinematicBody;
    C7Vector _bodyEnd_kinematicBody;

    // Scaled:
    C7Vector _localInertiaFrame_scaled;
    C7Vector _inverseLocalInertiaFrame_scaled;
    C3Vector _diagonalInertia_scaled; // includes mass!
    sReal _mass_scaled;

    C7Vector _localTransformation_old; // for old, "parent follows" functionality
};
