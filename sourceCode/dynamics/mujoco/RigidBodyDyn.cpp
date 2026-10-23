#include <RigidBodyDyn.h>
#include <RigidBodyContainerDyn.h>
#include <CollShapeDyn.h>
#include <simLib/simLib.h>

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
}

void CRigidBodyDyn::init(CXShape* shape, bool forceStatic, bool forceNonRespondable)
{
}

CPose CRigidBodyDyn::getInertiaFrameTransformation()
{
    CPose tr;
    return (tr);
}

CPose CRigidBodyDyn::getShapeFrameTransformation()
{
    CPose tr;
    return (tr);
}

void CRigidBodyDyn::reportVelocityToShape(double simulationTime)
{
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques()
{
}

void CRigidBodyDyn::handleKinematicBody_step(double t, double cumulatedTimeStep)
{
}

void CRigidBodyDyn::handleKinematicBody_end()
{
}
