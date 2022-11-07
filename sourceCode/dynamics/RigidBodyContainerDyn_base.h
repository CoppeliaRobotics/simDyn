#pragma once

#include "RigidBodyDyn.h"
#include "CollShapeDyn.h"
#include "ConstraintDyn_base.h"
#include "ParticleObjectContainer_base.h"
#include "dummyClasses.h"
#include "3Vector.h"
#include <string>
#include <map>
#include <set>

struct SContactInfo
{
    int subPassNumber;
    int objectID1;
    int objectID2;
    C3Vector position;
    C3Vector surfaceNormal;
    C3Vector directionAndAmplitude;
};

class CConstraintDyn;
class CRigidBodyContainerDyn;

class CRigidBodyContainerDyn_base
{
public:
    CRigidBodyContainerDyn_base();
    virtual ~CRigidBodyContainerDyn_base();

    virtual std::string init(const float floatParams[20],const int intParams[20]);

    virtual std::string getEngineInfo() const;
    virtual void serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize);

    virtual void handleDynamics(float dt,float simulationTime);
    virtual bool isDynamicContentAvailable();
    bool getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float* contactInfo);
    int getDynamicsCalculationPasses() const;
    float getSimulationTime() const;
    float* getContactPoints(int* cnt);

    static void setDynWorld(CRigidBodyContainerDyn* dynWorld);
    static CRigidBodyContainerDyn* getDynWorld();
    CParticleObjectContainer_base* getParticleCont();

    float getPositionScalingFactorDyn() const;
    float getLinearVelocityScalingFactorDyn() const;
    float getMassScalingFactorDyn() const;
    float getMasslessInertiaScalingFactorDyn() const;
    float getForceScalingFactorDyn() const;
    float getTorqueScalingFactorDyn() const;
    float getGravityScalingFactorDyn() const;
    float getDynamicsInternalTimeStep() const;
    int getDynamicParticlesIdStart() const;

    static bool isJointInDynamicMode(CXSceneObject* joint);

protected:
    virtual void _applyGravity();
    virtual void _stepDynamics(float dt,int pass);
    virtual void _createDependenciesBetweenJoints();
    virtual void _removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint);
    virtual bool _updateWorldFromCoppeliaSim();

    void _reportRigidBodyStatesToCoppeliaSim(float simulationTime);
    void _reportConstraintStatesToCoppeliaSim(float simulationTime,int currentPass,int totalPasses);
    virtual void _reportWorldToCoppeliaSim(float simulationTime,int currentPass,int totalPasses);
    void _handleAdditionalForcesAndTorques();
    void _clearAdditionalForcesAndTorques();
    CRigidBodyDyn* _getRigidBodyFromObjectHandle(int shapeHandle);
    void _getAllRigidBodies(std::vector<CRigidBodyDyn*>& allBodies);
    CConstraintDyn* _getConstraintFromObjectHandle(int object1Handle,int object2Handle);
    void _getAllConstraints(std::vector<CConstraintDyn*>& allConstraints);
    void _disableShapesOutOfDynamicActivityRange();
    void _getShapesToConsiderAsRigidBodies(std::set<CXShape*>& shapesToConsiderAsRigidBodies);
    void _updateRigidBodiesFromShapes(const std::set<CXShape*>& additionalShapesToConsiderAsRigidBodies);
    void _updateConstraintsFromJointsAndForceSensors();
    void _updateConstraintsFromDummies();

    bool _addRigidBodyFromShape(CXShape* shape,bool forceStatic,bool forceNonRespondable);
    bool _addConstraintFromJointOrForceSensor(CXSceneObject* object,bool isJoint);
    bool _addConstraintFromLinkedDummies(CXDummy* dummyA,CXDummy* dummyB);
    void _handleMotorControls(int passCnt,int totalPasses);

    void _addRigidBody(CRigidBodyDyn* body);
    void _removeRigidBody(int shapeHandle);
    void _addConstraint(CConstraintDyn* constr);
    void _removeConstraint(int object1Handle,int object2Handle);

    void _updateHybridJointTargetPositions_old();

    // Following 3 go in pair:
    void _handleKinematicBodies_init(float dt);
    void _handleKinematicBodies_step(float t,float cumulatedTimeStep);
    void _handleKinematicBodies_end();

    static CRigidBodyContainerDyn* _dynWorld;

    std::map<int,CRigidBodyDyn*> _allRigidBodies;
    std::map<int,CConstraintDyn*> _allConstraints;
    CParticleObjectContainer_base* _particleCont;

    std::vector<float> _contactPoints;
    std::vector<SContactInfo> _contactInfo; // Not same as above!

    int _dynamicsCalculationPasses;
    float _simulationTime;
    int _engine;
    int _engineVersion;
    float _positionScalingFactorDyn;
    float _linearVelocityScalingFactorDyn;
    float _massScalingFactorDyn;
    float _masslessInertiaScalingFactorDyn;
    float _forceScalingFactorDyn;
    float _torqueScalingFactorDyn;
    float _gravityScalingFactorDyn;
    float _dynamicActivityRange;
    float _dynamicsInternalStepSize;
    int _dynamicParticlesIdStart;
};
