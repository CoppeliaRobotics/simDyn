#pragma once

#include <RigidBodyContainerDyn_base.h>
#include <Vx/VxGearRatio.h>
class VortexIntersectSubscriber;
class VortexIntersectFilter;
namespace Vx
{
class VxUniverse;
}
struct SVortexJointDependency
{
    Vx::VxGearRatio* gear;
    CConstraintDyn* constr1;
    CConstraintDyn* constr2;
};

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
  public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const double floatParams[20], const int intParams[20]);

    std::string getEngineInfo() const;
    void serializeDynamicContent(const std::string& filenameAndPath, int maxSerializeBufferSize);

    Vx::VxUniverse* getWorld() const;

    static bool _checkingLicense;
    static double gravityVectorLength; // updated when handleDynamics is called
    static void _vortexCollisionCallbackStatic(void* data, Vx::VxCollisionGeometry* o1, Vx::VxCollisionGeometry* o2);
    void _vortexCollisionCallback(void* data, Vx::VxCollisionGeometry* o1, Vx::VxCollisionGeometry* o2);

  protected:
    void _applyGravity();
    void _licenseCheck();
    void _stepDynamics(double dt, int pass);
    void _createDependenciesBetweenJoints();
    void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

    void _addVortexContactPoints(int dynamicPassNumber);
    VortexIntersectSubscriber* vortexIntersectSubscriber;
    Vx::VxUniverse* _dynamicsWorld;
    VortexIntersectFilter* vortexIntersectFilter;
    std::vector<SVortexJointDependency> _gears;
};
