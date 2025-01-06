#pragma once

#include <RigidBodyContainerDyn_base.h>
#include <xmlser.h>
#include <mujoco/mujoco.h>
#include <simStack/stackArray.h>
#include <simStack/stackMap.h>

enum shapeModes
{
    staticMode = 0,
    kinematicMode = 1,
    freeMode = 2,
    attachedMode = 3
};

enum itemTypes
{
    shapeItem = 0,
    dummyShapeItem = 3
};

struct SMjShape
{ // only the shapes that have a counterpart in CoppeliaSim.
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId;         // body
    int mjIdStatic;   // body (static counterpart, if applies)
    int mjIdJoint;    // freejoint (if shape in free mode)
    shapeModes shapeMode;
    C7Vector staticShapeStart;
    C7Vector staticShapeGoal;
    itemTypes itemType;
    bool adhesion;
    std::vector<int> geomIndices; // pointing into _allGeoms
};

struct SMjGeom
{                        // for CoppeliaSim shape geoms
    int objectHandle;    // shape
    std::string name;    // shape
    int respondableMask;
    bool belongsToStaticItem;
    itemTypes itemType;

    int mjId; // geom
};

struct SMjJoint
{ // only the joints that exist as joints in CoppeliaSim
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId;         // joint
    int mjIdActuator; // actuator
    int actMode;      // 0=free, 1=force/torque, 2=mixed(force/vel, using inverse dyn.), 3=general mujoco actuator
    int jointType;
    double jointCtrlDv;
    double jointCtrlToApply;
    C4Vector initialBallQuat;
    C4Vector initialBallQuat2;
    int dependencyJointHandle;
    double polycoef[5];
};

struct SMjFreejoint
{ // only the freejoints that are linked to free shapes in CoppeliaSim
    int objectHandle;
    std::string name;      // name includes the "freejoint" suffix
    CXSceneObject* object; // shape

    int mjId;
};

struct SMjForceSensor
{
    int objectHandle;
    std::string name;
    CXSceneObject* object;

    int mjId;  // forceSensor(force)
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
    std::map<CXSceneObject*, int> massDividers;
    std::string folder;
    bool inertiaCalcRobust; // e.g. for meshes without volume, so that inertia can be computed
    bool isTreeDynamic;     // one a branch is dynamic, a static item indicates an error
};

class CRigidBodyContainerDyn : public CRigidBodyContainerDyn_base
{
  public:
    CRigidBodyContainerDyn();
    virtual ~CRigidBodyContainerDyn();

    std::string init(const double floatParams[20], const int intParams[20]);
    void handleDynamics(double dt, double simulationTime);

    std::string getEngineInfo() const;
    bool isDynamicContentAvailable() const;
    bool hasSimulationHalted() const;

  protected:
    static void _displayWarningAboutCPUCompatibility();
    static std::string _getObjectName(CXSceneObject* object);
    static bool _addMeshes(CXSceneObject* object, CXmlSer* xmlDoc, SInfo* info, std::vector<SMjGeom>* geoms, bool shapeIsStatic);
    int _hasContentChanged();

    std::string _buildMujocoWorld(double timeStep, double simTime, bool rebuild);
    bool _addObjectBranch(CXSceneObject* object, CXSceneObject* parent, CXmlSer* xmlDoc, SInfo* info);
    void _addShape(CXSceneObject* object, CXSceneObject* parent, CXmlSer* xmlDoc, SInfo* info);
    void _addInertiaElement(CXmlSer* xmlDoc, double mass, const C7Vector& tr, const C3Vector diagI);

    int _handleContact(const mjModel* m, mjData* d, int geom1, int geom2);
    void _handleControl(const mjModel* m, mjData* d);
    void _handleMotorControl(SMjJoint* mujocoItem);
    void _handleContactPoints(int dynPass);
    double _getAngleMinusAlpha(double angle, double alpha);

    static int _contactCallback(const mjModel* m, mjData* d, int geom1, int geom2);
    static void _controlCallback(const mjModel* m, mjData* d);
    static void _errorCallback(const char* err);
    static void _warningCallback(const char* warn);

    void _stepDynamics(double dt, int pass);
    void _reportWorldToCoppeliaSim(double simulationTime, int currentPass, int totalPasses);
    bool _updateWorldFromCoppeliaSim();
    void _handleKinematicBodies_step(double t, double cumulatedTimeStep);

    static bool _simulationHalted;

    mjModel* _mjModel;
    mjData* _mjData;
    mjData* _mjDataCopy;

    std::vector<SMjGeom> _allGeoms;              // shape geoms, particles and composites
    std::vector<SMjJoint> _allJoints;            // only joints that also exist in CoppeliaSim
    std::vector<SMjFreejoint> _allFreejoints;    // only freejoints linked to free shapes in CoppeliaSim
    std::vector<std::string> _allfreeJointNames; // only freejoints from CoppeliaSim free shapes
    std::vector<SMjForceSensor> _allForceSensors;
    std::vector<SMjShape> _allShapes; // only shapes that also exist in CoppeliaSim
    std::vector<int> _geomIdIndex;

    int _objectCreationCounter;
    int _objectDestructionCounter;
    int _hierarchyChangeCounter;
    bool _firstCtrlPass;
    int _currentPass;
    int _overrideKinematicFlag;
    int _rebuildTrigger;
    int _restartCount;
    bool _restartWarning;
    std::map<std::string, bool> _dynamicallyResetObjects;
};
