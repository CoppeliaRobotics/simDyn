#pragma once

#include "RigidBodyContainerDyn_base.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedObjectArray.h"

typedef bool (*ContactAddedCallback)(
    btManifoldPoint& cp,
    const btCollisionObject* colObj0,
    int partId0,
    int index0,
    const btCollisionObject* colObj1,
    int partId1,
    int index1);

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const double floatParams[20],const int intParams[20]);

    std::string getEngineInfo() const;
    void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

    btDiscreteDynamicsWorld* getWorld() const;

    void addBulletContactPoints(int dynamicPassNumber);

protected:
    void _applyGravity();
    void _stepDynamics(double dt,int pass);
    void _createDependenciesBetweenJoints();
    void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);

    static bool _bulletContactCallback(btManifoldPoint& cp,const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);

    btDiscreteDynamicsWorld* _dynamicsWorld;
    btBroadphaseInterface* _broadphase;
    btCollisionDispatcher* _dispatcher;
    btConstraintSolver* _solver;
    btDefaultCollisionConfiguration* _collisionConfiguration;
    btOverlapFilterCallback* _filterCallback;
    static bool _bulletContactCallback_useCustom;
    static double _bulletContactCallback_combinedFriction;
    static double _bulletContactCallback_combinedRestitution;
};
