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

    virtual CPose getInertiaFrameTransformation();
    virtual CPose getShapeFrameTransformation();
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
    CPose _bodyStart_kinematicBody;
    CPose _bodyEnd_kinematicBody;

    // Scaled:
    CPose _localInertiaFrame_scaled;
    CPose _inverseLocalInertiaFrame_scaled;
    C3Vector _diagonalInertia_scaled; // includes mass!
    sReal _mass_scaled;

    CPose _localTransformation_old; // for old, "parent follows" functionality
};
