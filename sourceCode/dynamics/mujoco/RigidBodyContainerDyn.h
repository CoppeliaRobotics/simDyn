#pragma once

#include "RigidBodyContainerDyn_base.h"
#include "xmlser.h"
#include <mujoco/mujoco.h>

enum shapeModes{staticMode=0,kinematicMode=1,freeMode=2,attachedMode=3};

struct SMjShape
{
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId; // body
    int mjId2; // body (static counterpart, if applies), or freejoint (if shape in free mode)
    shapeModes shapeMode;
    C7Vector shapeComTr; // Shape's com transf rel to shape frame
    C7Vector staticShapeStart;
    C7Vector staticShapeGoal;
};

struct SMjGeom
{
    int objectHandle; // shape or particle handle
    std::string name; // shape or particle name
    int particleShapeRespondableMask; // for particles only
    bool particleParticleRespondable; // for particles only

    int mjId; // geom
};

struct SMjJoint
{
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId; // joint
    int mjId2; // actuator
    int actMode; // 0=free, 1=force/torque, 2=mixed(force/vel, using inverse dyn.)
    int jointType;
    float jointCtrlDv;
    float jointCtrlForceToApply;
    C4Vector initialBallQuat;
    int dependencyJointHandle;
    double polycoef[5];
};

struct SMjForceSensor
{
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId; // forceSensor(force)
    int mjId2; // forceSensor(torque)
};

struct SHfield
{
    std::string file;
    int nrow;
    int ncol;
    double size[4];
};

struct SInfo
{
    std::vector<CXSceneObject*> moreToExplore;
    std::vector<std::string> meshFiles;
    std::vector<SHfield> heightfieldFiles;
    std::vector<CXSceneObject*> loopClosures;
    std::vector<CXSceneObject*> staticWelds;
    std::map<CXSceneObject*,int> massDividers;
    std::string folder;
    bool inertiaCalcRobust; // e.g. for meshes without volume, so that inertia can be computed
};

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const float floatParams[20],const int intParams[20]);
    void handleDynamics(float dt,float simulationTime);

    std::string getEngineInfo() const;
    bool isDynamicContentAvailable();

    static float computeInertia(int shapeHandle,C7Vector& tr,C3Vector& diagI,bool addRobustness=false);

protected:
    static std::string _getObjectName(CXSceneObject* object);
    static bool _addMeshes(CXSceneObject* object,CXmlSer* xmlDoc,SInfo* info,std::vector<SMjGeom>* geoms);
    std::string _buildMujocoWorld(float timeStep);
    bool _addObjectBranch(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info);
    void _addShape(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info);
    void _addInertiaElement(CXmlSer* xmlDoc,float mass,const C7Vector& tr,const C3Vector diagI);

    int _handleContact(const mjModel* m,mjData* d,int geom1,int geom2);
    void _handleControl(const mjModel* m,mjData* d);
    void _handleMotorControl(SMjJoint* mujocoItem,int passCnt,int totalPasses);
    void _handleContactPoints(int dynPass);
    dynReal _getAngleMinusAlpha(dynReal angle,dynReal alpha);

    static int _contactCallback(const mjModel* m,mjData* d,int geom1,int geom2);
    static void _controlCallback(const mjModel* m,mjData* d);
    static void _errorCallback(const char* err);
    static void _warningCallback(const char* warn);

    void _stepDynamics(float dt,int pass);
    void _reportWorldToCoppeliaSim(float simulationTime,int currentPass,int totalPasses);
    bool _updateWorldFromCoppeliaSim();
    void _handleKinematicBodies_step(float t,float cumulatedTimeStep);

    static bool _simulationHalted;

    mjModel* _mjModel;
    mjData* _mjData;
    mjData* _mjDataCopy;

    std::vector<SMjGeom> _allGeoms; // shape and particles
    std::vector<SMjJoint> _allJoints; // not freejoints
    std::vector<SMjForceSensor> _allForceSensors;
    std::vector<SMjShape> _allShapes;
    std::vector<int> _geomIdIndex;

    bool _firstDynPass;
    bool _firstCtrlPass;
    int _currentPass;
};
