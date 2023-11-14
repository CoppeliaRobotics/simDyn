#pragma once

#include <RigidBodyContainerDyn_base.h>
#include <PxPhysicsAPI.h>
#include <foundation/Px.h>
#include <foundation/PxSimpleTypes.h>

using namespace physx;

struct SInfo
{
    std::vector<CXSceneObject*> moreToExplore;
    std::vector<std::string> meshFiles;
//    std::vector<SHfield> heightfieldFiles;
    std::vector<CXSceneObject*> loopClosures;
    std::vector<CXSceneObject*> tendons;
    std::vector<CXSceneObject*> staticWelds;
    std::map<CXSceneObject*,int> massDividers;
    std::string folder;
    bool inertiaCalcRobust; // e.g. for meshes without volume, so that inertia can be computed
    bool isTreeDynamic; // one a branch is dynamic, a static item indicates an error
};

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const double floatParams[20],const int intParams[20]);
    void handleDynamics(double dt,double simulationTime);

    std::string getEngineInfo() const;
    bool isDynamicContentAvailable();

    void particlesAdded();

protected:
    static std::string _getObjectName(CXSceneObject* object);
    int _hasContentChanged();
    std::string _buildPhysxWorld(double timeStep,double simTime,bool rebuild);
    /*
    bool _addObjectBranch(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info);
    static bool _addMeshes(CXSceneObject* object,CXmlSer* xmlDoc,SInfo* info,std::vector<SMjGeom>* geoms,bool shapeIsStatic);
    void _addShape(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info);
*/
    double _getAngleMinusAlpha(double angle,double alpha);

    void _stepDynamics(double dt,int pass);
    void _reportWorldToCoppeliaSim(double simulationTime,int currentPass,int totalPasses);
    bool _updateWorldFromCoppeliaSim();
    void _handleKinematicBodies_step(double t,double cumulatedTimeStep);

    int _objectCreationCounter;
    int _objectDestructionCounter;
    int _hierarchyChangeCounter;
 //   static bool _simulationHalted;

    PxDefaultAllocator gAllocator;
    PxDefaultErrorCallback gErrorCallback;
    PxFoundation* gFoundation;
    PxPhysics* gPhysics;
    PxDefaultCpuDispatcher*	gDispatcher;
    PxScene* gScene;
    PxMaterial* gMaterial;
    //*
    PxPvd* gPvd;
    //*/
};
