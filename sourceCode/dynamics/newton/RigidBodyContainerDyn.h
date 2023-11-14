#pragma once

#include <RigidBodyContainerDyn_base.h>
#include <Newton.h>
#include <dMatrix.h>
#include <CustomJoint.h>
#include <CustomHinge.h>
#include <CustomSlider.h>
#include <CustomBallAndSocket.h>
#include <CustomHingeActuator.h>
#include <CustomSliderActuator.h>

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
public:

    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const sReal floatParams[20],const int intParams[20]);

    std::string getEngineInfo() const;
    void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

    NewtonWorld* getWorld() const;

    void _notifySekeletonRebuild ();
    void _rebuildSkeletonList();
    static void* NewtonAllocMemory (int sizeInBytes);
    static void NewtonFreeMemory (void* const ptr, int sizeInBytes);

    bool _rebuildSkeletons;

protected:
    void _applyGravity();
    void _stepDynamics(sReal dt,int pass);
    void _createDependenciesBetweenJoints();
    void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);
    bool _updateWorldFromCoppeliaSim();

    void _addNewtonContactPoints(int dynamicPassNumber);
    typedef void (*NewtonSkeletontDestructor) (const NewtonSkeletonContainer* const me);
    static void NewtonOnUserContacts (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
    static int NewtonOnAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);

    NewtonWorld* _dynamicsWorld;
};
