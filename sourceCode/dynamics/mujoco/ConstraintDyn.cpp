#include "ConstraintDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CConstraintDyn::CConstraintDyn()
{
}
CConstraintDyn::~CConstraintDyn()
{
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint)
{
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB)
{
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB)
{
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor)
{
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB)
{
}

void CConstraintDyn::_updateJointLimits(CXJoint* joint)
{
}

void CConstraintDyn::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
}

dynReal CConstraintDyn::getPrismaticJointPosition() const
{
    return(0.0);
}

dynReal CConstraintDyn::getRevoluteJointAngle()
{
    return(0.0);
}

dynReal CConstraintDyn::getRevoluteJointAngle_forCoppeliaSim()
{
    return(0.0);
}

void CConstraintDyn::reportStateToCoppeliaSim(float simulationTime,int currentPass,int totalPasses)
{
}
