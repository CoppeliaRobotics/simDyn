#pragma once

#include <ConstraintDyn_base.h>
#include <ode/ode.h>

class CConstraintDyn : public CConstraintDyn_base
{
  public:
    CConstraintDyn();
    virtual ~CConstraintDyn();
    // ShapeA -> joint -> shapeB
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint);
    // ShapeA -> joint -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint, CXDummy* dummyA, CXDummy* dummyB);
    // ShapeA -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXDummy* dummyA, CXDummy* dummyB);
    // ShapeA -> force sensor -> shapeB
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor);
    // ShapeA -> force sensor -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor, CXDummy* dummyA, CXDummy* dummyB);

    void reportStateToCoppeliaSim(double simulationTime, int currentPass, int totalPasses);
    double getPrismaticJointPosition() const; // important! The slider pos is not initialized when added (Bullet)!
    double getRevoluteJointAngle();
    double getRevoluteJointAngle_forCoppeliaSim();

  protected:
    void _updateJointLimits(CXJoint* joint);
    void _handleJoint(CXJoint* joint, int passCnt, int totalPasses);

    dJointID _odeConstraint;
    dJointFeedback* _odeJointFeedbackStructure;
};
