#pragma once

#include "RigidBodyContainerDyn_base.h"
#include "xmlser.h"
#include <mujoco/mujoco.h>

enum shapeModes{staticMode=0,kinematicMode=1,freeMode=2,attachedMode=3};

enum itemTypes{shapeItem=0,particleItem=1,compositeItem=2,dummyShapeItem=3};

struct SMjShape
{
    int objectHandle; // shape. All particles are -2, all composites are -3
    std::string name;
    CXSceneObject* object;

    int mjId; // body
    int mjId2; // body (static counterpart, if applies), or freejoint (if shape in free mode)
    shapeModes shapeMode;
    C7Vector shapeComTr; // Shape's com transf rel to shape frame
    C7Vector staticShapeStart;
    C7Vector staticShapeGoal;
    itemTypes itemType;
};

struct SMjGeom
{
    int objectHandle; // shape. All particles are -2, all composites are -3
    std::string name; // shape, particle or composite name
    std::string prefix; // composite prefix
    int respondableMask; // for particles and composites only
    itemTypes itemType;

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

    int type; // 0=CoppeliaSim joint, 2=composite
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
    std::vector<CXSceneObject*> tendons;
    std::vector<CXSceneObject*> staticWelds;
    std::map<CXSceneObject*,int> massDividers;
    std::string folder;
    bool inertiaCalcRobust; // e.g. for meshes without volume, so that inertia can be computed
};

struct SInject
{
    std::string xml;
    std::string element;
    int objectHandle;
    std::string xmlDummyString;
};

struct SCompositeInject
{
    std::string xml;
    int shapeHandle;
    std::string element;
    std::string type;
    int respondableMask; // global
    double grow;
    std::string prefix;
    std::string xmlDummyString;
    size_t count[3];
    std::vector<int> mjIds;
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

    std::string getCompositeInfo(const char* prefix,int what,std::vector<double>& info,int count[3]) const;
    static float computeInertia(int shapeHandle,C7Vector& tr,C3Vector& diagI,bool addRobustness=false);
    static void injectXml(const char* xml,const char* element,int objectHandle);
    static void injectCompositeXml(const char* xml,int shapeHandle,const char* element,const char* prefix,const size_t* count,const char* type,int respondableMask,double grow);
    static int getCompositeIndexFromPrefix(const char* prefix);

protected:
    static std::string _getObjectName(CXSceneObject* object);
    static bool _addMeshes(CXSceneObject* object,CXmlSer* xmlDoc,SInfo* info,std::vector<SMjGeom>* geoms);
    void _addInjections(CXmlSer* xmlDoc,int objectHandle,const char* currentElement);
    void _addComposites(CXmlSer* xmlDoc,int shapeHandle,const char* currentElement);
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
    static std::vector<SInject> _xmlInjections;
    static std::vector<SCompositeInject> _xmlCompositeInjections;

    mjModel* _mjModel;
    mjData* _mjData;
    mjData* _mjDataCopy;

    std::vector<SMjGeom> _allGeoms; // shape, particles and composites
    std::vector<SMjJoint> _allJoints; // not freejoints
    std::vector<SMjForceSensor> _allForceSensors;
    std::vector<SMjShape> _allShapes;
    std::vector<int> _geomIdIndex;

    bool _firstDynPass;
    bool _firstCtrlPass;
    int _currentPass;
    int _overrideKinematicFlag;
};
