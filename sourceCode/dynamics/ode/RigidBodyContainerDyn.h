#pragma once

#include <RigidBodyContainerDyn_base.h>
#include <ode/ode.h>

struct SOdeContactData
{
    dJointID jointID;
    int objectID1;
    int objectID2;
    C3Vector positionScaled;
    C3Vector normalVector;
};

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
  public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const double floatParams[20], const int intParams[20]);

    std::string getEngineInfo() const;
    void serializeDynamicContent(const std::string& filenameAndPath, int maxSerializeBufferSize);

    dWorldID getWorld() const;

    dSpaceID getOdeSpace();

  protected:
    void _applyGravity();
    void _stepDynamics(double dt, int pass);
    void _createDependenciesBetweenJoints();
    void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

    static void _odeCollisionCallbackStatic(void* data, dGeomID o1, dGeomID o2);
    void _odeCollisionCallback(void* data, dGeomID o1, dGeomID o2);
    dWorldID _dynamicsWorld;
    dSpaceID _odeSpace;
    dJointGroupID _odeContactGroup;
    std::vector<SOdeContactData> _odeContactsRegisteredForFeedback;
};
