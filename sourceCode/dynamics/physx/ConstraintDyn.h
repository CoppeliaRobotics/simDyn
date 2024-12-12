#pragma once

#include <ConstraintDyn_base.h>

class CConstraintDyn : public CConstraintDyn_base
{
  public:
    CConstraintDyn();
    virtual ~CConstraintDyn();
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint);
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint, CXDummy* dummyA, CXDummy* dummyB);
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXDummy* dummyA, CXDummy* dummyB);
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor);
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor, CXDummy* dummyA, CXDummy* dummyB);

    void reportStateToCoppeliaSim(double simulationTime, int currentPass, int totalPasses);
    double getPrismaticJointPosition() const;
    double getRevoluteJointAngle();
    double getRevoluteJointAngle_forCoppeliaSim();

  protected:
    void _updateJointLimits(CXJoint* joint);
    void _handleJoint(CXJoint* joint, int passCnt, int totalPasses);
};
