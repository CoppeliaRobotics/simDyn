#include <simLib/simLib.h>
#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "RigidBodyDyn.h"
#include "ConstraintDyn.h"

CRigidBodyContainerDyn* CRigidBodyContainerDyn_base::_dynWorld=nullptr;

CRigidBodyContainerDyn_base::CRigidBodyContainerDyn_base()
{
    _dynamicsCalculationPasses=0;
    _particleCont=new CParticleObjectContainer_base();
}

CRigidBodyContainerDyn_base::~CRigidBodyContainerDyn_base()
{
    delete _particleCont;
}

std::string CRigidBodyContainerDyn_base::init(const sReal floatParams[20],const int intParams[20])
{
    _positionScalingFactorDyn=floatParams[0];
    _linearVelocityScalingFactorDyn=floatParams[1];
    _massScalingFactorDyn=floatParams[2];
    _masslessInertiaScalingFactorDyn=floatParams[3];
    _forceScalingFactorDyn=floatParams[4];
    _torqueScalingFactorDyn=floatParams[5];
    _gravityScalingFactorDyn=floatParams[6];
    _dynamicActivityRange=floatParams[7];
    _dynamicParticlesIdStart=intParams[0];

    _allRigidBodies.clear();
    _allConstraints.clear();
    return("");
}

std::string CRigidBodyContainerDyn_base::getEngineInfo() const
{
    return("");
}

void CRigidBodyContainerDyn_base::_applyGravity()
{
}

void CRigidBodyContainerDyn_base::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
}

void CRigidBodyContainerDyn_base::_stepDynamics(sReal dt,int pass)
{
}

void CRigidBodyContainerDyn_base::_createDependenciesBetweenJoints()
{
}

void CRigidBodyContainerDyn_base::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
}

void CRigidBodyContainerDyn_base::setDynWorld(CRigidBodyContainerDyn* dynWorld)
{
    _dynWorld=dynWorld;
}

CRigidBodyContainerDyn* CRigidBodyContainerDyn_base::getDynWorld()
{
    return(_dynWorld);
}

CParticleObjectContainer_base* CRigidBodyContainerDyn_base::getParticleCont()
{
    return(_particleCont);
}

sReal CRigidBodyContainerDyn_base::getPositionScalingFactorDyn() const
{
    return(_positionScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getLinearVelocityScalingFactorDyn() const
{
    return(_linearVelocityScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getMassScalingFactorDyn() const
{
    return(_massScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getMasslessInertiaScalingFactorDyn() const
{
    return(_masslessInertiaScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getForceScalingFactorDyn() const
{
    return(_forceScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getTorqueScalingFactorDyn() const
{
    return(_torqueScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getGravityScalingFactorDyn() const
{
    return(_gravityScalingFactorDyn);
}

sReal CRigidBodyContainerDyn_base::getDynamicsInternalTimeStep() const
{
    return(_dynamicsInternalStepSize);
}

int CRigidBodyContainerDyn_base::getDynamicParticlesIdStart() const
{
    return(_dynamicParticlesIdStart);
}

int CRigidBodyContainerDyn_base::getDynamicsCalculationPasses() const
{
    return(_dynamicsCalculationPasses);
}

sReal CRigidBodyContainerDyn_base::getSimulationTime() const
{
    return(_simulationTime);
}

bool CRigidBodyContainerDyn_base::isJointInDynamicMode(CXSceneObject* joint)
{
    int m=_simGetJointMode(joint);
    while (m==sim_jointmode_dependent)
    {
        int masterJ;
        sReal off,mult;
        simGetJointDependency(_simGetObjectID(joint),&masterJ,&off,&mult);
        if (masterJ==-1)
            break;
        joint=(CXSceneObject*)_simGetObject(masterJ);
        m=_simGetJointMode(joint);
    }
    return(m==sim_jointmode_dynamic);
}

bool CRigidBodyContainerDyn_base::_addRigidBodyFromShape(CXShape* shape,bool forceStatic,bool forceNonRespondable)
{
    bool retVal=false;
    CRigidBodyDyn* body=_getRigidBodyFromObjectHandle(_simGetObjectID(shape));
    if (body==nullptr)
    { // We have to add that shape!
        CXGeomProxy* geom=(CXGeomProxy*)_simGetGeomProxyFromShape(shape); // even non respondable shapes have a geom
        CXGeomWrap* geomInfoFromShape=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geom);

        // Following to display a warning with static respondable shapes build on a non-static construction:
        if ((_simIsShapeDynamicallyStatic(shape)||forceStatic)&&(_simIsShapeDynamicallyRespondable(shape)&&(!forceNonRespondable)))
        { // explore its parents:
            CXSceneObject* parent=(CXSceneObject*)_simGetParentObject(shape);
            while (parent!=nullptr)
            {
                if (_simGetObjectType(parent)==sim_object_shape_type)
                {
                    CXShape* s((CXShape*)parent);
                    if ((_simIsShapeDynamicallyStatic(s)==0)&&(_simGetTreeDynamicProperty(s)&sim_objdynprop_dynamic))
                    {
                        _simMakeDynamicAnnouncement(sim_announce_containsstaticshapesondynamicconstruction);
                        break;
                    }
                }
                // We don't wanna check for dynamic joints or force sensors, since we might have an intended force sensor or joint at the base (e.g. for later assembly with another model)
                parent=(CXSceneObject*)_simGetParentObject(parent);
            }
        }

        // Following to display a warning if using non-pure non-convex shapes:
        if ((_simGetPurePrimitiveType(geomInfoFromShape)==sim_primitiveshape_none)&&(geom!=nullptr)&&(_simIsGeomWrapConvex(geomInfoFromShape)==0)&&_simIsShapeDynamicallyRespondable(shape)&&(_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable))
            _simMakeDynamicAnnouncement(sim_announce_containsnonpurenonconvexshapes);

        // Now create the rigid body
        body=new CRigidBodyDyn();
        body->init(shape,forceStatic,forceNonRespondable);

        _addRigidBody(body);
        retVal=true;
    }
    return(retVal);
}

CRigidBodyDyn* CRigidBodyContainerDyn_base::_getRigidBodyFromObjectHandle(int shapeHandle)
{
    CRigidBodyDyn* retVal=nullptr;
    auto it=_allRigidBodies.find(shapeHandle);
    if (it!=_allRigidBodies.end())
        retVal=it->second;
    return(retVal);
}

void CRigidBodyContainerDyn_base::_getAllRigidBodies(std::vector<CRigidBodyDyn*>& allBodies)
{
    allBodies.clear();
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        allBodies.push_back(it->second);
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_getAllConstraints(std::vector<CConstraintDyn*>& allConstraints)
{
    allConstraints.clear();
    auto it=_allConstraints.begin();
    while (it!=_allConstraints.end())
    {
        allConstraints.push_back(it->second);
        ++it;
    }
}

CConstraintDyn* CRigidBodyContainerDyn_base::_getConstraintFromObjectHandle(int object1Handle,int object2Handle)
{
    CConstraintDyn* retVal=nullptr;
    if ( (object1Handle>=0)||(object2Handle>=0) )
    { // normally only one handle in case of joints or force sensors, but dummy-dummy pairs will need 2 handles
        if (object1Handle<0)
            object1Handle=999999999;
        if (object2Handle<0)
            object2Handle=999999999;
        int h=std::min<int>(object1Handle,object2Handle);
        if (_simGetObject(h)!=nullptr) // not really needed
        {
            auto it=_allConstraints.find(h);
            if (it!=_allConstraints.end())
                retVal=it->second;
        }
    }
    return(retVal);
}

void CRigidBodyContainerDyn_base::_addRigidBody(CRigidBodyDyn* body)
{
    _allRigidBodies[body->getShapeHandle()]=body;
}

void CRigidBodyContainerDyn_base::_removeRigidBody(int shapeHandle)
{
    auto it=_allRigidBodies.find(shapeHandle);
    if (it!=_allRigidBodies.end())
    {
        // First remove dependent constraint objects:
        std::vector<CConstraintDyn*> allConstraints;
        _getAllConstraints(allConstraints);
        for (size_t i=0;i<allConstraints.size();i++)
        {
            CConstraintDyn* constr=allConstraints[i];
            if (constr->announceBodyWillBeDestroyed(shapeHandle))
                _removeConstraint(constr->getIdentifyingHandle(),-1);
        }
        // Now the rigid body itself:
        delete it->second;
        _allRigidBodies.erase(it);
    }
}

void CRigidBodyContainerDyn_base::_addConstraint(CConstraintDyn* constr)
{
    _allConstraints[constr->getIdentifyingHandle()]=constr;
}

void CRigidBodyContainerDyn_base::_removeConstraint(int object1Handle,int object2Handle)
{
    if ( (object1Handle>=0)||(object2Handle>=0) )
    { // normally only one handle in case of joints or force sensors, but dummy-dummy pairs will need 2 handles
        if (object1Handle<0)
            object1Handle=999999999;
        if (object2Handle<0)
            object2Handle=999999999;
        int h=std::min<int>(object1Handle,object2Handle);
        auto it=_allConstraints.find(h);
        if (it!=_allConstraints.end())
        {
            _removeDependenciesBetweenJoints(it->second);
            delete it->second;
            _allConstraints.erase(it);
        }
    }
}

void CRigidBodyContainerDyn_base::_updateRigidBodiesFromShapes(const std::set<CXShape*>& shapesToConsiderAsRigidBodies)
{
    // 1. Check which rigid bodies have to be removed:
    std::vector<CRigidBodyDyn*> allBodies;
    _getAllRigidBodies(allBodies);
    for (size_t i=0;i<allBodies.size();i++)
    {
        CRigidBodyDyn* body=allBodies[i];
        CXShape* shape=(CXShape*)_simGetObject(body->getShapeHandle());
        bool remove=true;
        if ( (shape!=nullptr)&&(_simGetDynamicsFullRefreshFlag(shape)==0) )
        { // we have to check if it is still valid:
            int dp=_simGetTreeDynamicProperty(shape);
            bool isStatic=( ((dp&sim_objdynprop_dynamic)==0)||_simIsShapeDynamicallyStatic(shape) );
            bool isNeverRespondable=((dp&sim_objdynprop_respondable)==0);
            remove=(body->isStatic()!=isStatic)||(body->isNeverRespondable()!=isNeverRespondable);
        }
        if (remove)
            _removeRigidBody(body->getShapeHandle());
    }

    // 2. we have to add new shapes
    auto it=shapesToConsiderAsRigidBodies.begin();
    while (it!=shapesToConsiderAsRigidBodies.end())
    {
        CXShape* shape=*it;
        int dp=_simGetTreeDynamicProperty(shape);
        if (dp&(sim_objdynprop_dynamic|sim_objdynprop_respondable))
        {
            if (_addRigidBodyFromShape(shape,(dp&sim_objdynprop_dynamic)==0,(dp&sim_objdynprop_respondable)==0))
                _simSetDynamicSimulationIconCode(shape,sim_dynamicsimicon_objectisdynamicallysimulated);
        }
        else
            _simSetDynamicSimulationIconCode(shape,sim_dynamicsimicon_none);
        _simSetDynamicsFullRefreshFlag(shape,false);
        ++it;
    }
}

bool CRigidBodyContainerDyn_base::isDynamicContentAvailable()
{
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        if (!it->second->isStatic())
            return(true);
        ++it;
    }
    return(false);
}

void CRigidBodyContainerDyn_base::handleDynamics(sReal dt,sReal simulationTime)
{
    sReal maxDynStep;
    simGetFloatParam(sim_floatparam_physicstimestep,&maxDynStep);

    _dynamicsCalculationPasses=int((dt/maxDynStep)+0.5);
    if (_dynamicsCalculationPasses<1)
        _dynamicsCalculationPasses=1;
    _dynamicsInternalStepSize=dt/sReal(_dynamicsCalculationPasses);
    _simulationTime=simulationTime;
    bool particlesPresent=_updateWorldFromCoppeliaSim();
    _createDependenciesBetweenJoints();
    _contactInfo.clear();
    _contactPoints.clear();

    if (isDynamicContentAvailable()||particlesPresent)
    {
        _applyGravity();
        _handleKinematicBodies_init(dt);
        for (int i=0;i<_dynamicsCalculationPasses;i++)
        {
            int integers[4]={0,i+1,_dynamicsCalculationPasses,0};
            sReal floats[1]={_dynamicsInternalStepSize};
            _simDynCallback(integers,floats);
            _handleKinematicBodies_step(sReal(i+1)/sReal(_dynamicsCalculationPasses),dt);
            _handleMotorControls(i+1,_dynamicsCalculationPasses); // to enable/disable motors, to update target velocities, target positions and position control
            _handleAdditionalForcesAndTorques(); // for shapes but also for "anti-gravity" particles or particels with fluid friction force!
            _contactPoints.clear();
            _stepDynamics(_dynamicsInternalStepSize,i);
            _simulationTime+=_dynamicsInternalStepSize;
            _reportWorldToCoppeliaSim(_simulationTime,i,_dynamicsCalculationPasses);
            integers[3]=1;
            _simDynCallback(integers,floats);
        }
        _handleKinematicBodies_end();
    }

    _clearAdditionalForcesAndTorques();

    // 1=respondable, 2=dynamic, 4=free, 8=motor, 16=pos control,32=force sensor, 64=loop closure dummy?
    // Do following always, also when displaying the normal scene (so that when we switch during a pause, it looks correct)
    int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
    for (int i=0;i<shapeListSize;i++)
    {
        CXShape* it=(CXShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
        CRigidBodyDyn* b=_getRigidBodyFromObjectHandle(_simGetObjectID(it));
        int flag=0;
        if (b!=nullptr)
        {
            if (!b->isStatic())
                flag|=2;
            if ( _simIsShapeDynamicallyRespondable(it)&&((_simGetTreeDynamicProperty(it)&sim_objdynprop_respondable)!=0) )
                flag|=1;
        }
        _simSetDynamicObjectFlagForVisualization(it,flag);
    }
    int jointListSize=_simGetObjectListSize(sim_object_joint_type);
    for (int i=0;i<jointListSize;i++)
    {
        CXJoint* it=(CXJoint*)_simGetObjectFromIndex(sim_object_joint_type,i);
        CConstraintDyn* c=_getConstraintFromObjectHandle(_simGetObjectID(it),-1);
        int flag=0;
        if (c!=nullptr)
            flag=4;
        _simSetDynamicObjectFlagForVisualization(it,flag);
    }
    int forceSensorListSize=_simGetObjectListSize(sim_object_forcesensor_type);
    for (int i=0;i<forceSensorListSize;i++)
    {
        CXForceSensor* it=(CXForceSensor*)_simGetObjectFromIndex(sim_object_forcesensor_type,i);
        CConstraintDyn* c=_getConstraintFromObjectHandle(_simGetObjectID(it),-1);
        int flag=0;
        if (c!=nullptr)
            flag=32;
        _simSetDynamicObjectFlagForVisualization(it,flag);
    }
    int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
    for (int i=0;i<dummyListSize;i++)
    {
        CXDummy* it=(CXDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
        int linkedDummyHandle=-1;
        int linkType=_simGetDummyLinkType(it,&linkedDummyHandle);
        int flag=0;
        if ( (linkType==sim_dummylink_dynloopclosure)&&(linkedDummyHandle!=-1) )
        {
            CConstraintDyn* c=_getConstraintFromObjectHandle(_simGetObjectID(it),linkedDummyHandle);
            if (c!=nullptr)
                flag=64;
        }
        _simSetDynamicObjectFlagForVisualization(it,flag);
    }
}

void CRigidBodyContainerDyn_base::_reportWorldToCoppeliaSim(sReal simulationTime,int currentPass,int totalPasses)
{
    _reportConstraintStatesToCoppeliaSim(simulationTime,currentPass,totalPasses);
    _reportRigidBodyStatesToCoppeliaSim(simulationTime);
    _particleCont->updateParticlesPosition(simulationTime);
}

void CRigidBodyContainerDyn_base::_reportRigidBodyStatesToCoppeliaSim(sReal simulationTime)
{
    // It is important that we update shapes from base to tip, otherwise we get wrong positions!
    std::vector<CXSceneObject*> toExplore;
    int orphanListSize=_simGetObjectListSize(-1);
    for (int i=0;i<orphanListSize;i++)
        toExplore.push_back((CXSceneObject*)_simGetObjectFromIndex(-1,i));
    for (size_t i=0;i<toExplore.size();i++)
    {
        CXSceneObject* it=toExplore[i];
        if (_simGetObjectType(it)==sim_object_shape_type)
        {
            CXShape* shape=(CXShape*)it;
            int shapeHandle=_simGetObjectID(shape);
            CRigidBodyDyn* body=_getRigidBodyFromObjectHandle(shapeHandle);
            if (body!=nullptr)
            {
                body->reportConfigurationToShape(simulationTime);
                body->reportVelocityToShape(simulationTime);
            }
            else
                _simSetShapeDynamicVelocity(shape,C3Vector::zeroVector.data,C3Vector::zeroVector.data,simulationTime);
        }
        // Now its own children:
        int childrenCount;
        CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(it,&childrenCount);
        if (childrenCount>0)
            toExplore.insert(toExplore.end(),childrenPointer,childrenPointer+childrenCount);
    }
}

void CRigidBodyContainerDyn_base::_reportConstraintStatesToCoppeliaSim(sReal simulationTime,int currentPass,int totalPasses)
{
    auto it=_allConstraints.begin();
    while (it!=_allConstraints.end())
    {
        CConstraintDyn* constraint=it->second;
        constraint->reportStateToCoppeliaSim(simulationTime,currentPass,totalPasses);
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_handleKinematicBodies_init(sReal dt)
{
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        CRigidBodyDyn* body=it->second;
        if (body->isStatic())
            body->handleKinematicBody_init(dt);
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_handleKinematicBodies_step(sReal t,sReal cumulatedTimeStep)
{
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        CRigidBodyDyn* body=it->second;
        if (body->isStatic())
            body->handleKinematicBody_step(t,cumulatedTimeStep);
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_handleKinematicBodies_end()
{
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        CRigidBodyDyn* body=it->second;
        if (body->isStatic())
        {
            CXShape* shape=body->getShape();
            if (shape!=nullptr)
                body->handleKinematicBody_end();
        }
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_updateConstraintsFromJointsAndForceSensors()
{
    // 1. We have to remove all constraints that are not valid anymore
    std::vector<CConstraintDyn*> allConstraints;
    _getAllConstraints(allConstraints);
    for (size_t i=0;i<allConstraints.size();i++)
    {
        CConstraintDyn* constraint=allConstraints[i];
        int sceneObjectHandle=constraint->getJointOrForceSensorHandle();
        if (sceneObjectHandle!=-1)
        { // we have a joint or force sensor constraint
            CXSceneObject* sceneObject=constraint->getJointOrForceSensor();
            bool remove=true;
            if (sceneObject!=nullptr)
            {
                if ( (_simGetDynamicsFullRefreshFlag(sceneObject)==0)&&((_simGetTreeDynamicProperty(sceneObject)&sim_objdynprop_dynamic)!=0) )
                { // We have to make sure the bodies are still there! (also with the same hierarchy relationship!)
                    CXSceneObject* parent=(CXSceneObject*)_simGetParentObject(sceneObject);
                    int childrenCount;
                    CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(sceneObject,&childrenCount);
                    if ( (parent!=nullptr)&&(childrenCount==1) )
                    {
                        CXSceneObject* child=childrenPointer[0];
                        if (_simGetObjectID(parent)==constraint->getShapeAHandle())
                        { // the parent is still there
                            CXShape* shapeA=(CXShape*)parent;
                            // 2 possibilities: child is a shape (regular case), or it is a dummy
                            int dummyAHandle=constraint->getDummyAHandle();
                            if (dummyAHandle==-1)
                            { // we have the regular case: shape --> joint/fsensor --> shape
                                if (_simGetObjectID(child)==constraint->getShapeBHandle())
                                { // we have the same child
                                    CXShape* shapeB=(CXShape*)child;
                                    if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
                                        remove=false;
                                }
                            }
                            else
                            { // we have a complex case: shape --> joint/fsensor --> dummy -- dummy <-- shape
                                int childChildListSize;
                                _simGetObjectChildren(child,&childChildListSize);
                                if ( (_simGetObjectID(child)==dummyAHandle)&&(childChildListSize==0) )
                                {
                                    CXDummy* dummyA=(CXDummy*)child;
                                    int dummyALinkedDummyID;
                                    int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyID);
                                    CXDummy* dummyB=(CXDummy*)_simGetObject(dummyALinkedDummyID);
                                    if (dummyB!=nullptr)
                                    {
                                        int dummyBChildListSize;
                                        _simGetObjectChildren(dummyB,&dummyBChildListSize);
                                        if ((dummyALinkType==sim_dummylink_dynloopclosure)&&(dummyBChildListSize==0))
                                        {
                                            CXSceneObject* dummyBParent=(CXSceneObject*)_simGetParentObject(dummyB);
                                            int childDummyBID=constraint->getDummyBHandle();
                                            if ( (childDummyBID==_simGetObjectID(dummyB))&&(dummyBParent!=nullptr) )
                                            {
                                                if (_simGetObjectID(dummyBParent)==constraint->getShapeBHandle())
                                                {
                                                    CXShape* shapeB=(CXShape*)dummyBParent;
                                                    if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
                                                        remove=false;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (remove)
                _removeConstraint(constraint->getIdentifyingHandle(),-1);
        }
    }

    // 2. we have to add new objects
    int obT[]={sim_object_joint_type,sim_object_forcesensor_type};
    for (size_t j=0;j<2;j++)
    {
        int objListSize=_simGetObjectListSize(obT[j]);
        for (int i=0;i<objListSize;i++)
        {
            CXSceneObject* object=(CXSceneObject*)_simGetObjectFromIndex(obT[j],i);
            if ( ( (obT[j]!=sim_object_joint_type)||isJointInDynamicMode(object)||_simIsJointInHybridOperation(object) )&&(_simGetTreeDynamicProperty(object)&sim_objdynprop_dynamic) )
                _addConstraintFromJointOrForceSensor(object,obT[j]==sim_object_joint_type);
            else
                _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_none);
            _simSetDynamicsFullRefreshFlag(object,false);
        }
    }
}

void CRigidBodyContainerDyn_base::_updateConstraintsFromDummies()
{
    // 1. We have to remove all constraints that are not valid anymore
    std::vector<CConstraintDyn*> allConstraints;
    _getAllConstraints(allConstraints);
    for (size_t i=0;i<allConstraints.size();i++)
    {
        CConstraintDyn* constraint=allConstraints[i];
        int dummyAHandle=constraint->getDummyAHandle();
        if ( (constraint->getJointOrForceSensorHandle()==-1)&&(dummyAHandle!=-1) )
        { // constraint of type shape --> dummyA -- dummyB <-- shape
            CXDummy* dummyA=constraint->getDummyA();
            bool remove=true;
            if (dummyA!=nullptr)
            {
                if ( (_simGetDynamicsFullRefreshFlag(dummyA)==0)&&((_simGetTreeDynamicProperty(dummyA)&sim_objdynprop_dynamic)!=0) )
                { // We have to make sure the bodies are still there! (also with the same hierarchy relationship!)
                    CXShape* shapeA=(CXShape*)_simGetParentObject(dummyA);
                    int dummyBHandle;
                    int dummyLinkType=_simGetDummyLinkType(dummyA,&dummyBHandle);
                    CXDummy* dummyB=(CXDummy*)_simGetObject(dummyBHandle);
                    if ( (shapeA!=nullptr)&&(dummyB!=nullptr)&&(_simGetDynamicsFullRefreshFlag(dummyB)==0)&&((_simGetTreeDynamicProperty(dummyB)&sim_objdynprop_dynamic)!=0) )
                    {
                        int dummyAChildListSize;
                        _simGetObjectChildren(dummyA,&dummyAChildListSize);
                        int dummyBChildListSize;
                        _simGetObjectChildren(dummyB,&dummyBChildListSize);
                        if ((dummyLinkType==sim_dummylink_dynloopclosure)&&(dummyAChildListSize==0)&&(dummyBChildListSize==0) )
                        {
                            CXShape* shapeB=(CXShape*)_simGetParentObject(dummyB);
                            if (shapeB!=nullptr)
                            {
                                if ((_simGetObjectID(shapeA)==constraint->getShapeAHandle())&&(_simGetObjectID(shapeB)==constraint->getShapeBHandle()))
                                { // now make sure that at least one of the shape is dynamic (a rigid joint between two static shapes is not YET supported)
                                    if ( (_simIsShapeDynamicallyStatic(shapeA)==0)||(_simIsShapeDynamicallyStatic(shapeB)==0) )
                                        remove=false;
                                }
                            }
                        }
                    }
                }
            }
            if (remove)
                _removeConstraint(constraint->getIdentifyingHandle(),-1);
        }
    }

    // 2. we have to add new linked dummies
    int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
    for (int i=0;i<dummyListSize;i++)
    {
        CXDummy* dummy=(CXDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
        int dummyHandle=_simGetObjectID(dummy);
        int linkedDummyHandle;
        int linkType=_simGetDummyLinkType(dummy,&linkedDummyHandle);
        if ( (linkType==sim_dummylink_dynloopclosure)&&(dummyHandle<linkedDummyHandle) )
            _addConstraintFromLinkedDummies(dummy,(CXDummy*)_simGetObject(linkedDummyHandle));
        _simSetDynamicsFullRefreshFlag(dummy,false);
    }
}

bool CRigidBodyContainerDyn_base::_addConstraintFromJointOrForceSensor(CXSceneObject* object,bool isJoint)
{
    bool retVal=false;
    CConstraintDyn* constraint=_getConstraintFromObjectHandle(_simGetObjectID(object),-1);
    if (constraint==nullptr)
    { // We have to add that constraint (maybe)!
        CXSceneObject* parentObj=(CXSceneObject*)_simGetParentObject(object);
        int jointChildListSize;
        CXSceneObject** jointChildrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&jointChildListSize);
        if ( (jointChildListSize==1)&&(parentObj!=nullptr)&&(_simGetObjectType(parentObj)==sim_object_shape_type) )
        {
            CXSceneObject* childObj=jointChildrenPointer[0];
            if (_simGetObjectType(childObj)==sim_object_shape_type)
            { // regular case: the link is shape --> joint/fsensor --> shape
                CRigidBodyDyn* bodyA(_getRigidBodyFromObjectHandle(_simGetObjectID(parentObj)));
                CRigidBodyDyn* bodyB(_getRigidBodyFromObjectHandle(_simGetObjectID(childObj)));
                if ( (bodyA!=nullptr)&&(bodyB!=nullptr) )
                { // Now make sure we the child is dynamic:
                    // following line new since 2010/03/17:
                    if (!bodyB->isStatic())
                    {

                        constraint=new CConstraintDyn();
                        if (isJoint)
                            constraint->init(bodyA,bodyB,(CXJoint*)object);
                        else
                            constraint->init(bodyA,bodyB,(CXForceSensor*)object);
                        _addConstraint(constraint);
                        retVal=true;
                    }
                }
            }
            else
            { // we might have a link: shape --> object/fsensor --> dummy -- dummy <-- shape
                if (_simGetObjectType(childObj)==sim_object_dummy_type)
                {
                    CXDummy* dummyA=(CXDummy*)childObj;
                    int dummyALinkedDummyId;
                    int dummyALinkType=_simGetDummyLinkType(dummyA,&dummyALinkedDummyId);
                    CXDummy* dummyB=(CXDummy*)_simGetObject(dummyALinkedDummyId);
                    if ( (dummyB!=nullptr)&&(dummyALinkType==sim_dummylink_dynloopclosure) )
                    {
                        int dummyAChildListSize,dummyBChildListSize;
                        _simGetObjectChildren(dummyA,&dummyAChildListSize);
                        _simGetObjectChildren(dummyB,&dummyBChildListSize);
                        if ((dummyAChildListSize==0)&&(dummyBChildListSize==0))
                        {
                            CXSceneObject* dummyBParent=(CXSceneObject*)_simGetParentObject(dummyB);
                            if (dummyBParent!=nullptr)
                            {
                                CRigidBodyDyn* bodyA(_getRigidBodyFromObjectHandle(_simGetObjectID(parentObj)));
                                CRigidBodyDyn* bodyB(_getRigidBodyFromObjectHandle(_simGetObjectID(dummyBParent)));
                                if ( (bodyA!=nullptr)&&(bodyB!=nullptr) )
                                { // Now make sure we have at least one dynamic body:
                                    if ( (!bodyA->isStatic())||(!bodyB->isStatic()) )
                                    {
                                        constraint=new CConstraintDyn();
                                        if (isJoint)
                                            constraint->init(bodyA,bodyB,(CXJoint*)object,dummyA,dummyB);
                                        else
                                            constraint->init(bodyA,bodyB,(CXForceSensor*)object,dummyA,dummyB);
                                        _addConstraint(constraint);
                                        retVal=true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (retVal)
            _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_objectisdynamicallysimulated);
        else
            _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_objectisnotdynamicallyenabled);
    }
    return(retVal);
}

bool CRigidBodyContainerDyn_base::_addConstraintFromLinkedDummies(CXDummy* dummyA,CXDummy* dummyB)
{ // dummyA is normally the one with the lower handle
    bool retVal=false;
    CConstraintDyn* constraint=_getConstraintFromObjectHandle(_simGetObjectID(dummyA),-1);
    if ( (constraint==nullptr)&&(_simGetTreeDynamicProperty(dummyA)&sim_objdynprop_dynamic)&&(_simGetTreeDynamicProperty(dummyB)&sim_objdynprop_dynamic) )
    { // We have to add that constraint! (maybe)
        CXSceneObject* parentObj=(CXSceneObject*)_simGetParentObject(dummyA);
        if ( (parentObj!=nullptr)&&(_simGetObjectType(parentObj)==sim_object_shape_type) )
        {
            int dummyAchLs,dummyBchLs;
            _simGetObjectChildren(dummyA,&dummyAchLs);
            _simGetObjectChildren(dummyB,&dummyBchLs);
            if ( (dummyAchLs==0)&&(dummyBchLs==0) )
            {
                CXSceneObject* secondParentObj=(CXSceneObject*)_simGetParentObject(dummyB);
                if ( (secondParentObj!=nullptr)&&(_simGetObjectType(secondParentObj)==sim_object_shape_type) )
                {
                    CRigidBodyDyn* bodyA(_getRigidBodyFromObjectHandle(_simGetObjectID(parentObj)));
                    CRigidBodyDyn* bodyB(_getRigidBodyFromObjectHandle(_simGetObjectID(secondParentObj)));
                    if ( (bodyA!=nullptr)&&(bodyB!=nullptr) )
                    { // Now make sure that at least one body is dynamic:
                        if ( (!bodyA->isStatic())||(!bodyB->isStatic()) )
                        {
                            constraint=new CConstraintDyn();
                            constraint->init(bodyA,bodyB,dummyA,dummyB);
                            _addConstraint(constraint);
                            retVal=true;
                        }
                    }
                }
            }
        }
    }
    return(retVal);
}

bool CRigidBodyContainerDyn_base::_updateWorldFromCoppeliaSim()
{
    std::set<CXShape*> shapesToConsiderAsRigidBodies;
    _getShapesToConsiderAsRigidBodies(shapesToConsiderAsRigidBodies);
    _updateRigidBodiesFromShapes(shapesToConsiderAsRigidBodies);
    _updateConstraintsFromJointsAndForceSensors();
    _updateConstraintsFromDummies();
    _updateHybridJointTargetPositions_old();
    _disableShapesOutOfDynamicActivityRange();

    bool particlesPresent=_particleCont->addParticlesIfNeeded();
    _particleCont->removeKilledParticles();
    return(particlesPresent);
}

void CRigidBodyContainerDyn_base::_disableShapesOutOfDynamicActivityRange()
{ // So that falling objects stop falling eventually. In all directions
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        CRigidBodyDyn* body=it->second;
        CXShape* shape=body->getShape();
        if (_simGetParentObject(shape)==nullptr)
        {
            C7Vector tr;
            _simGetObjectLocalTransformation(shape,tr.X.data,tr.Q.data,true);
            if (tr.X.getLength()>_dynamicActivityRange)
                _simDisableDynamicTreeForManipulation(shape,true);
        }
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_getShapesToConsiderAsRigidBodies(std::set<CXShape*>& shapesToConsiderAsRigidBodies)
{
    // We look for shapes that are
    // a) non-static or respondable.
    // Additionally, we look for static and non-respondable shapes in following situations:
    // b) staticAndNotRespondableShape --> dynJoint/fsensor --> dynShape
    // or:
    // c) staticAndNotRespondableShape --> dynJoint/fsensor --> dummyA -- dummyB <-- dynShape
    // or:
    // d) dynShape --> dynJoint/fsensor --> dummyA -- dummyB <-- staticAndNotRespondableShape
    // or:
    // e) staticAndNotRespondableShape --> dummyA -- dummyB <-- dynShape
    // or:
    // f) dynShape --> dummyA -- dummyB <-- staticAndNotRespondableShape

    shapesToConsiderAsRigidBodies.clear();

    // a)
    int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
    for (int j=0;j<shapeListSize;j++)
    {
        CXShape* shape=(CXShape*)_simGetObjectFromIndex(sim_object_shape_type,j);
        if ( _simIsShapeDynamicallyRespondable(shape)||(_simIsShapeDynamicallyStatic(shape)==0) )
            shapesToConsiderAsRigidBodies.insert(shape);
    }

    // b) and c)
    int objT[]={sim_object_joint_type,sim_object_forcesensor_type};
    for (size_t objTi=0;objTi<2;objTi++)
    {
        int objectListSize=_simGetObjectListSize(objT[objTi]);
        for (int i=0;i<objectListSize;i++)
        {
            CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(objT[objTi],i);
            if ( (objT[objTi]==sim_object_forcesensor_type)||isJointInDynamicMode(it)||_simIsJointInHybridOperation(it) )
            { // we have a dynamic joint or fsensor here.
                CXSceneObject* parentShape=(CXSceneObject*)_simGetParentObject(it);
                if ( (parentShape!=nullptr)&&(_simGetObjectType(parentShape)==sim_object_shape_type) )
                {
                    if ( shapesToConsiderAsRigidBodies.find((CXShape*)parentShape)==shapesToConsiderAsRigidBodies.end() )
                    { // b) and c) We just check for staticAndNotRespondableShape --> dynJoint/fsensor --> uniqueChild
                        int itChildListSize;
                        _simGetObjectChildren(it,&itChildListSize);
                        if (itChildListSize==1)
                            shapesToConsiderAsRigidBodies.insert((CXShape*)parentShape);
                    }
                }
            }
        }
    }

    // d), e) and f)
    int dummyListSize=_simGetObjectListSize(sim_object_dummy_type);
    for (int i=0;i<dummyListSize;i++)
    {
        CXDummy* dummyA=(CXDummy*)_simGetObjectFromIndex(sim_object_dummy_type,i);
        CXSceneObject* parentShape=(CXSceneObject*)_simGetParentObject(dummyA);
        if ( (parentShape!=nullptr)&&(_simGetObjectType(parentShape)==sim_object_shape_type) )
        {
            if ( shapesToConsiderAsRigidBodies.find((CXShape*)parentShape)==shapesToConsiderAsRigidBodies.end() )
            { // We just check for staticAndNotRespondableShape --> dummyA(noChild) -- dummyB(noChild)
                int itChildListSize;
                _simGetObjectChildren(dummyA,&itChildListSize);
                if (itChildListSize==0)
                {
                    int linkedDummyHandle;
                    int dummyLinkType=_simGetDummyLinkType(dummyA,&linkedDummyHandle);
                    CXDummy* dummyB=(CXDummy*)_simGetObject(linkedDummyHandle);
                    if ( (dummyB!=nullptr)&&(dummyLinkType==sim_dummylink_dynloopclosure) )
                    {
                        _simGetObjectChildren(dummyB,&itChildListSize);
                        if (itChildListSize==0)
                            shapesToConsiderAsRigidBodies.insert((CXShape*)parentShape);
                    }
                }
            }
        }
    }
}

void CRigidBodyContainerDyn_base::_handleMotorControls(int passCnt,int totalPasses)
{
    auto it=_allConstraints.begin();
    while (it!=_allConstraints.end())
    {
        CConstraintDyn* constraint=it->second;
        CXJoint* joint=(CXJoint*)constraint->getJoint();
        if (joint!=nullptr)
            constraint->handleMotorControl(joint,passCnt,totalPasses);
        ++it;
    }
}

void CRigidBodyContainerDyn_base::_updateHybridJointTargetPositions_old()
{
    std::vector<CConstraintDyn*> allConstraints;
    _getAllConstraints(allConstraints);
    for (size_t i=0;i<allConstraints.size();i++)
    {
        CConstraintDyn* constraint=allConstraints[i];
        CXJoint* joint=constraint->getJoint();
        if ( (joint!=nullptr)&&_simIsJointInHybridOperation(joint) ) // we could have a dummy constraint!
        { 
//            if (_simGetJointMode(joint)==sim_jointmode_dependent)
//                _simSetJointPosition(joint,0.0); // value doesn't matter. We just wanna refresh the value that is dependent!
            _simSetDynamicMotorPositionControlTargetPosition(joint,_simGetJointPosition(joint));
        }
    }
}

sReal* CRigidBodyContainerDyn_base::getContactPoints(int* cnt)
{
    cnt[0]=_contactPoints.size()/3;
    if (cnt[0]==0)
        return(nullptr);
    return(&_contactPoints[0]);
}

bool CRigidBodyContainerDyn_base::getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],sReal* contactInfo)
{
    int scanIndex=0;
    int ind=(index|sim_handleflag_extended)-sim_handleflag_extended;
    bool extended=((index&sim_handleflag_extended)!=0);
    for (size_t i=0;i<_contactInfo.size();i++)
    {
        if ( (_contactInfo[i].subPassNumber==dynamicPass)||(dynamicPass==sim_handle_all) )
        {
            if ( (_contactInfo[i].objectID1==objectHandle)||(_contactInfo[i].objectID2==objectHandle)||(sim_handle_all==objectHandle) )
            {
                if (ind==scanIndex)
                {
                    contactInfo[0]=_contactInfo[i].position(0);
                    contactInfo[1]=_contactInfo[i].position(1);
                    contactInfo[2]=_contactInfo[i].position(2);
                    if (_contactInfo[i].objectID2==objectHandle)
                    {
                        objectHandles[0]=_contactInfo[i].objectID2;
                        objectHandles[1]=_contactInfo[i].objectID1;
                        contactInfo[3]=-_contactInfo[i].directionAndAmplitude(0);
                        contactInfo[4]=-_contactInfo[i].directionAndAmplitude(1);
                        contactInfo[5]=-_contactInfo[i].directionAndAmplitude(2);
                        if (extended)
                        {
                            contactInfo[6]=-_contactInfo[i].surfaceNormal(0);
                            contactInfo[7]=-_contactInfo[i].surfaceNormal(1);
                            contactInfo[8]=-_contactInfo[i].surfaceNormal(2);
                        }
                    }
                    else
                    {
                        objectHandles[0]=_contactInfo[i].objectID1;
                        objectHandles[1]=_contactInfo[i].objectID2;
                        contactInfo[3]=_contactInfo[i].directionAndAmplitude(0);
                        contactInfo[4]=_contactInfo[i].directionAndAmplitude(1);
                        contactInfo[5]=_contactInfo[i].directionAndAmplitude(2);
                        if (extended)
                        {
                            contactInfo[6]=_contactInfo[i].surfaceNormal(0);
                            contactInfo[7]=_contactInfo[i].surfaceNormal(1);
                            contactInfo[8]=_contactInfo[i].surfaceNormal(2);
                        }
                    }
                    return(true);
                }
                scanIndex++;
            }
        }
    }
    return(false);
}

void CRigidBodyContainerDyn_base::_clearAdditionalForcesAndTorques()
{
    int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
    for (int i=0;i<shapeListSize;i++)
    {
        CXShape* it=(CXShape*)_simGetObjectFromIndex(sim_object_shape_type,i);
        _simClearAdditionalForceAndTorque(it);
    }
}

void CRigidBodyContainerDyn_base::_handleAdditionalForcesAndTorques()
{
    auto it=_allRigidBodies.begin();
    while (it!=_allRigidBodies.end())
    {
        CRigidBodyDyn* body=it->second;
        body->handleAdditionalForcesAndTorques();
        ++it;
    }
    C3Vector gravity;
    _simGetGravity(gravity.data);
    _particleCont->handleAntiGravityForces_andFluidFrictionForces(gravity);
}
