#include "RigidBodyDyn.h"
#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "simLib.h"

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
}

void CRigidBodyDyn::init(CXShape* shape,bool forceStatic,bool forceNonRespondable)
{
}

C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{
    C7Vector tr;
    return(tr);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{
    C7Vector tr;
    return(tr);
}

void CRigidBodyDyn::reportVelocityToShape(double simulationTime)
{
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques()
{
}

void CRigidBodyDyn::handleKinematicBody_step(double t,double cumulatedTimeStep)
{
}

void CRigidBodyDyn::handleKinematicBody_end()
{
}

