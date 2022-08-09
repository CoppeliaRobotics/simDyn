#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "RigidBodyDyn.h"
#include "ConstraintDyn.h"
#include "simLib.h"
#include "4X4Matrix.h"
#include <filesystem>
#include<iostream>
#include<fstream>

bool CRigidBodyContainerDyn::_simulationHalted=false;

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine=sim_physics_mujoco;
    _engineVersion=0;
    _mjModel=nullptr;
    _mjData=nullptr;
    _mjDataCopy=nullptr;
    _firstCtrlPass=true;
    _firstDynPass=true;
    _simulationHalted=false;
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
    mju_user_warning=nullptr;
    mju_user_error=nullptr;
    mjcb_contactfilter=nullptr;
    mjcb_control=nullptr;
    mj_deleteData(_mjDataCopy);
    mj_deleteData(_mjData);
    mj_deleteModel(_mjModel);
}

std::string CRigidBodyContainerDyn::init(const float floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams,intParams);
    return("");
}

std::string CRigidBodyContainerDyn::_buildMujocoWorld(float timeStep)
{
    char* _dir=simGetStringParam(sim_stringparam_mujocodir);
    std::string mjFile(_dir);
    std::string dir(_dir);
    simReleaseBuffer(_dir);
    std::filesystem::remove_all(mjFile.c_str());
    std::filesystem::create_directory(mjFile.c_str());
    mjFile+="/coppeliaSim.xml";
    CXmlSer* xmlDoc=new CXmlSer(mjFile.c_str());
    xmlDoc->pushNewNode("compiler");
    xmlDoc->setAttr("coordinate","global");
    xmlDoc->setAttr("angle","radian");
    xmlDoc->setAttr("balanceinertia",true); // TODO
    xmlDoc->setAttr("boundmass",0.0); // TODO
    xmlDoc->setAttr("boundinertia",0.000001); // TODO
    xmlDoc->popNode();

    xmlDoc->pushNewNode("size");
    xmlDoc->setAttr("njmax",1000); // TODO
    xmlDoc->setAttr("nconmax",500); // TODO
    xmlDoc->popNode();

    xmlDoc->pushNewNode("default");
    xmlDoc->pushNewNode("geom");
    xmlDoc->setAttr("rgba",0.8,0.6,0.4,1.0);
    xmlDoc->popNode();
    xmlDoc->popNode();

    xmlDoc->pushNewNode("option");
    xmlDoc->setAttr("timestep",timeStep);
    // TODO: option to compute all inertias from geoms and mass
    xmlDoc->setAttr("impratio",1.0); // TODO
    xmlDoc->setAttr("wind",0.0,0.0,0.0); // TODO
    xmlDoc->setAttr("density",0.0); // TODO
    xmlDoc->setAttr("viscosity",0.0); // TODO
  //  xmlDoc->setAttr("o_solref",0.0); // TODO
    //  xmlDoc->setAttr("o_solimp",0.0); // TODO
    xmlDoc->setAttr("integrator","Euler"); // TODO
    xmlDoc->setAttr("cone","pyramidal"); // TODO
    xmlDoc->setAttr("solver","Newton"); // TODO
    xmlDoc->setAttr("iterations",100); // TODO
    C3Vector gravity;
    _simGetGravity(gravity.data);
    xmlDoc->setAttr("gravity",gravity(0),gravity(1),gravity(2));
    xmlDoc->pushNewNode("flag");
    xmlDoc->setAttr("filterparent","disable");
    xmlDoc->setAttr("fwdinv","disable");
    xmlDoc->setAttr("multiccd","disable"); //TODO
    xmlDoc->popNode();
    xmlDoc->popNode();

    xmlDoc->pushNewNode("worldbody");
    xmlDoc->pushNewNode("light");
    xmlDoc->setAttr("pos",0.0,0.0,2.0);
    xmlDoc->setAttr("dir",0.0,-1.0,-1.0);
    xmlDoc->setAttr("diffuse",1,1,1);
    xmlDoc->popNode();
    xmlDoc->pushNewNode("light");
    xmlDoc->setAttr("pos",1.0,1.0,2.0);
    xmlDoc->setAttr("dir",-1.0,-1.0,-1.0);
    xmlDoc->setAttr("diffuse",1,1,1);
    xmlDoc->popNode();

    SInfo info;
    info.folder=dir;
    info.inertiaCalcRobust=false;

    // We need to balance the mass of certain shapes across other shapes, when involved in loop closure with a joint/forceSensor, e.g.:
    // shape1 --> joint --> dummy1 -- dummy2 <-- shape2
    // In Mujoco we can only implement this if we place an aux. body where dummy1 is, and equality constrain it to be 'glued' to shape2
    // In order to not bias the mass/inertia with that aux. body, we split the mass/inertia of shape2 onto all aux. bodies that might
    // be 'glued' to it. This requires 2 exploration passes, where the first pass only adjusts the mass dividers
    std::map<CXSceneObject*,int> massDividers; // shape,divider
    int shapeListSize=_simGetObjectListSize(sim_object_shape_type);
    for (int i=0;i<shapeListSize;i++)
    {
        CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(sim_object_shape_type,i);
        info.massDividers[it]=1;
    }

    for (size_t pass=0;pass<2;pass++)
    { // pass0 is to get the mass dividers only. Pass1 is the actual pass
        CXmlSer* ser=nullptr;
        if (pass==1)
            ser=xmlDoc;
        std::vector<CXSceneObject*> toExplore;
        info.meshFiles.clear();
        info.loopClosures.clear();
        info.staticWelds.clear();
        int orphanListSize=_simGetObjectListSize(-1);
        for (int i=0;i<orphanListSize;i++)
        {
            CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(-1,i);
            toExplore.push_back(it);
        }
        for (size_t i=0;i<toExplore.size();i++)
        {
            info.moreToExplore.clear();
            if (_addObjectBranch(toExplore[i],nullptr,ser,&info))
                toExplore.insert(toExplore.end(),info.moreToExplore.begin(),info.moreToExplore.end());
        }

        if (ser!=nullptr)
        {
            xmlDoc->popNode();

            xmlDoc->pushNewNode("asset");

            xmlDoc->pushNewNode("texture");
            xmlDoc->setAttr("type","skybox");
            xmlDoc->setAttr("builtin","gradient");
            xmlDoc->setAttr("rgb1",1,1,1);
            xmlDoc->setAttr("rgb2",0.6,0.8,1.0);
            xmlDoc->setAttr("width",256);
            xmlDoc->setAttr("height",256);
            xmlDoc->popNode();

            for (size_t i=0;i<info.meshFiles.size();i++)
            {
                xmlDoc->pushNewNode("mesh");
                xmlDoc->setAttr("file",info.meshFiles[i].c_str());
                xmlDoc->popNode();
            }
            for (size_t i=0;i<info.heightfieldFiles.size();i++)
            {
                xmlDoc->pushNewNode("hfield");
                xmlDoc->setAttr("file",info.heightfieldFiles[i].file.c_str());
                xmlDoc->setAttr("size",info.heightfieldFiles[i].size,4);
                xmlDoc->popNode();
            }

            xmlDoc->popNode();

            xmlDoc->pushNewNode("equality");
            for (size_t i=0;i<info.loopClosures.size();i++)
            {
                int dummy1Handle=_simGetObjectID(info.loopClosures[i]);
                int dummy2Handle=-1;
                _simGetDummyLinkType(info.loopClosures[i],&dummy2Handle);
                CXSceneObject* dummy2=(CXSceneObject*)_simGetObject(dummy2Handle);
                CXSceneObject* shape2=(CXSceneObject*)_simGetParentObject(dummy2);
                CXSceneObject* dummy1Parent=(CXSceneObject*)_simGetParentObject(info.loopClosures[i]);
                if (_simGetObjectType(dummy1Parent)==sim_object_shape_type)
                { // we have shape --> dummy -- dummy <-- shape
                    if (dummy1Handle<dummy2Handle)
                    { // make sure to not have 2 weld constraints for the same loop closure!
                        xmlDoc->pushNewNode("weld");
                        std::string nm(_getObjectName(dummy1Parent));
                        xmlDoc->setAttr("body1",nm.c_str());
                        nm=_getObjectName(shape2);
                        xmlDoc->setAttr("body2",nm.c_str());
                        C7Vector tr1,tr2;
                        _simGetObjectCumulativeTransformation(dummy1Parent,tr1.X.data,tr1.Q.data,1);
                        _simGetObjectCumulativeTransformation(info.loopClosures[i],tr2.X.data,tr2.Q.data,1);
                        C7Vector tra(tr1.getInverse()*tr2);
                        _simGetObjectCumulativeTransformation(shape2,tr1.X.data,tr1.Q.data,1);
                        _simGetObjectCumulativeTransformation(dummy2,tr2.X.data,tr2.Q.data,1);
                        C7Vector trb(tr2.getInverse()*tr1);
                        C7Vector tr(tra*trb);
                        double v[7]={tr.X(0),tr.X(1),tr.X(2),tr.Q(0),tr.Q(1),tr.Q(2),tr.Q(3)};
                        xmlDoc->setAttr("relpose",v,7);
                        xmlDoc->popNode();
                    }
                }
                else
                { // we have shape --> joint/forceSensor --> dummy -- dummy <-- shape
                    xmlDoc->pushNewNode("weld");
                    std::string nm(_getObjectName(info.loopClosures[i])+"loop");
                    xmlDoc->setAttr("body1",nm.c_str());
                    xmlDoc->setAttr("body2",_getObjectName(shape2).c_str());
                    C7Vector tr1,tr2;
                    _simGetObjectCumulativeTransformation(dummy2,tr1.X.data,tr1.Q.data,1);
                    _simGetObjectCumulativeTransformation(shape2,tr2.X.data,tr2.Q.data,1);
                    C7Vector tr(tr1.getInverse()*tr2);
                    double v[7]={tr.X(0),tr.X(1),tr.X(2),tr.Q(0),tr.Q(1),tr.Q(2),tr.Q(3)};
                    xmlDoc->setAttr("relpose",v,7);
                    xmlDoc->popNode();
                }
            }
            for (size_t i=0;i<info.staticWelds.size();i++)
            {
                xmlDoc->pushNewNode("weld");
                std::string nm(_getObjectName(info.staticWelds[i]));
                xmlDoc->setAttr("body1",(nm+"staticCounterpart").c_str());
                xmlDoc->setAttr("body2",nm.c_str());
                double v[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
                xmlDoc->setAttr("relpose",v,7);
                xmlDoc->popNode();
            }
            xmlDoc->popNode();

            xmlDoc->pushNewNode("sensor");
            for (size_t i=0;i<_allForceSensors.size();i++)
            {
                CXSceneObject* fsensor=(CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
                std::string nm(_getObjectName(fsensor));
                xmlDoc->pushNewNode("force");
                xmlDoc->setAttr("name",(nm+"force").c_str());
                xmlDoc->setAttr("site",nm.c_str());
                xmlDoc->popNode();
                xmlDoc->pushNewNode("torque");
                xmlDoc->setAttr("name",(nm+"torque").c_str());
                xmlDoc->setAttr("site",nm.c_str());
                xmlDoc->popNode();
            }
            xmlDoc->popNode(); // sensor

            xmlDoc->pushNewNode("actuator");
            for (size_t i=0;i<_allJoints.size();i++)
            {
                CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
                int m;
                simGetObjectInt32Param(_allJoints[i].objectHandle,sim_jointintparam_dynctrlmode,&m);
                xmlDoc->pushNewNode("motor");
                std::string nm(_getObjectName(joint));
                xmlDoc->setAttr("name",(nm+"act").c_str());
                xmlDoc->setAttr("joint",nm.c_str());
                //xmlDoc->setAttr("actlimited",limited);
                //xmlDoc->setAttr("actrange",low,high);
                if (m==sim_jointdynctrl_free)
                    _allJoints[i].actMode=0; // not actuated
                else if ( (m==sim_jointdynctrl_force)||(m==sim_jointdynctrl_spring) )
                    _allJoints[i].actMode=1; // pure force/torque
                else
                    _allJoints[i].actMode=2; // mixed
                xmlDoc->popNode();
            }
            xmlDoc->popNode(); // actuator

            xmlDoc->popNode();
        }
    }
    delete xmlDoc; // saves the file

    std::string retVal;

    char error[1000] = "could not load binary model";
    _mjModel=mj_loadXML(mjFile.c_str(),0,error,1000);
    if (_mjModel!=nullptr)
    {
        _mjData=mj_makeData(_mjModel);
        _mjDataCopy=mj_makeData(_mjModel);


        _geomIdIndex.resize(_mjModel->ngeom,-1);
        for (size_t i=0;i<_allGeoms.size();i++)
        {
            int mjId=mj_name2id(_mjModel,mjOBJ_GEOM,_allGeoms[i].name.c_str());
            _allGeoms[i].mjId=mjId;
            _geomIdIndex[mjId]=i;
        }
        _actuatorIdIndex.resize(_mjModel->nu,-1);
        for (size_t i=0;i<_allJoints.size();i++)
        {
            _allJoints[i].object=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
            int mjId=mj_name2id(_mjModel,mjOBJ_JOINT,_allJoints[i].name.c_str());
            _allJoints[i].mjId=mjId;
            mjId=mj_name2id(_mjModel,mjOBJ_ACTUATOR,(_allJoints[i].name+"act").c_str());
            _allJoints[i].mjId2=mjId;
            _actuatorIdIndex[mjId]=i;
        }
        for (size_t i=0;i<_allForceSensors.size();i++)
        {
            _allForceSensors[i].object=(CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
            int mjId=mj_name2id(_mjModel,mjOBJ_SENSOR,(_allForceSensors[i].name+"force").c_str());
            _allForceSensors[i].mjId=mjId;
            mjId=mj_name2id(_mjModel,mjOBJ_SENSOR,(_allForceSensors[i].name+"torque").c_str());
            _allForceSensors[i].mjId2=mjId;
        }
        for (size_t i=0;i<_allShapes.size();i++)
        {
            _allShapes[i].object=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
            int mjId=mj_name2id(_mjModel,mjOBJ_BODY,_allShapes[i].name.c_str());
            _allShapes[i].mjId=mjId;
            _allShapes[i].mjId2=mjId;
            if (_allShapes[i].shapeMode==shapeModes::kinematicMode)
                _allShapes[i].mjId2=mj_name2id(_mjModel,mjOBJ_BODY,(_allShapes[i].name+"staticCounterpart").c_str());
            if (_allShapes[i].shapeMode==shapeModes::freeMode)
            { // handle initial velocity for free bodies:
                mjId=mj_name2id(_mjModel,mjOBJ_JOINT,(_allShapes[i].name+"freejoint").c_str());
                _allShapes[i].mjId2=mjId;
                int nvadr=_mjModel->body_dofadr[_allShapes[i].mjId];
                C3Vector v;
                _simGetInitialDynamicVelocity(_allShapes[i].object,v.data);
                if (v.getLength()>0.0f)
                {
                    _mjData->qvel[nvadr+0]=v(0);
                    _mjData->qvel[nvadr+1]=v(1);
                    _mjData->qvel[nvadr+2]=v(2);
                    _simSetInitialDynamicVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                }
                _simGetInitialDynamicAngVelocity(_allShapes[i].object,v.data);
                if (v.getLength()>0.0f)
                {
                    _mjData->qvel[nvadr+3]=v(0);
                    _mjData->qvel[nvadr+4]=v(1);
                    _mjData->qvel[nvadr+5]=v(2);
                    _simSetInitialDynamicAngVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                }
            }
        }
        mjcb_contactfilter=_contactCallback;
        mjcb_control=_controlCallback;
        mju_user_error=_errorCallback;
        mju_user_warning=_warningCallback;
    }
    else
        retVal=error;
    return(retVal);
}

std::string CRigidBodyContainerDyn::_getObjectName(CXSceneObject* object)
{
    int objectHandle=_simGetObjectID(object);
    char* f=simGetObjectAlias(objectHandle,4);
    std::string retVal(f);
    simReleaseBuffer(f);
    return(retVal);
}

bool CRigidBodyContainerDyn::_addObjectBranch(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info)
{
    size_t moreToExploreInitSize=info->moreToExplore.size();
    int dynProp=_simGetTreeDynamicProperty(object);
    if (dynProp&(sim_objdynprop_dynamic|sim_objdynprop_respondable))
    { // elements in tree could be dynamic and/or respondable
        int objType=_simGetObjectType(object);
        bool objectWasAdded=false;
        bool ignoreChildren=false;
        if (objType==sim_object_shape_type)
        {
            bool isStatic=((dynProp&sim_objdynprop_dynamic)==0)||(_simIsShapeDynamicallyStatic(object)!=0);
            if ( isStatic&&(parent!=nullptr) )
            { // static shapes should always be added to "worldbody"!
                info->moreToExplore.push_back(object);
                ignoreChildren=true;
            }
            else
            {
                int parentType=-1;
                if (parent!=nullptr)
                    parentType=_simGetObjectType(parent);
                if (parentType==sim_object_shape_type)
                { // parent(shape) --> this(shape): those have to be added to "worldbody"
                    info->moreToExplore.push_back(object);
                    ignoreChildren=true;
                }
                else
                {
                    bool isNonRespondable=((dynProp&sim_objdynprop_respondable)==0)||(_simIsShapeDynamicallyRespondable(object)==0);
                    bool addIt=(!isNonRespondable)||(!isStatic);
                    if (!addIt)
                    { // check if we want to add this static, non-respondable shape
                        int childrenCount;
                        CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
                        for (int i=0;i<childrenCount;i++)
                        {
                            CXSceneObject* child=childrenPointer[i];
                            if (_simGetObjectType(child)==sim_object_forcesensor_type)
                            {
                                addIt=true;
                                break;
                            }
                            if ( (_simGetObjectType(child)==sim_object_joint_type)&&((_simGetJointMode(child)==sim_jointmode_dynamic)&&((_simGetTreeDynamicProperty(child)&sim_objdynprop_dynamic)!=0)) )
                            {
                                addIt=true;
                                break;
                            }
                        }
                    }
                    if (addIt)
                    {
                        _addShape(object,parent,xmlDoc,info);
                        objectWasAdded=true;
                    }
                }
            }
        }
        CXSceneObject* child=nullptr;
        if (objType==sim_object_joint_type)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&((_simGetObjectType(parent)==sim_object_shape_type)||(_simGetObjectType(parent)==sim_object_joint_type))&&(childrenCount==1)&&((_simGetJointMode(object)==sim_jointmode_dynamic)||_simIsJointInHybridOperation(object))&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape (or joint, consecutive joints are allowed!), joint is in correct mode, and has only one child
                if ( ( (_simGetObjectType(childrenPointer[0])==sim_object_shape_type)&&(_simIsShapeDynamicallyStatic(childrenPointer[0])==0) ) ||
                     ( (_simGetObjectType(childrenPointer[0])==sim_object_joint_type)&&(_simGetJointMode(childrenPointer[0])==sim_jointmode_dynamic) ) )
                { // child is a dyn. shape (or a dyn. joint)
                    _addObjectBranch(childrenPointer[0],object,xmlDoc,info);
                    ignoreChildren=true;
                }
                else
                    child=childrenPointer[0]; // check further down if that joint is involved in a loop closure
            }
        }
        if (objType==sim_object_forcesensor_type)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_object_shape_type)&&(childrenCount==1)&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape, and force sensor has only one child
                if ( (_simGetObjectType(childrenPointer[0])==sim_object_shape_type)&&(_simIsShapeDynamicallyStatic(childrenPointer[0])==0) )
                { // child is a dyn. shape
                    _addObjectBranch(childrenPointer[0],object,xmlDoc,info);
                    ignoreChildren=true;
                }
                else
                    child=childrenPointer[0]; // check further down if that force sensor is involved in a loop closure
            }
        }
        if (objType==sim_object_dummy_type)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_object_shape_type)&&(childrenCount==0)&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape, and dummy has no child
                int linkedDummyHandle=-1;
                int linkType=_simGetDummyLinkType(object,&linkedDummyHandle);
                if ( (linkType==sim_dummy_linktype_dynamics_loop_closure)&&(linkedDummyHandle!=-1) )
                { // the dummy is linked to another dummy via a dyn. overlap constr.
                    CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
                    if ( (linkedDummyParent!=nullptr)&&(_simGetObjectType(linkedDummyParent)==sim_object_shape_type) )
                    { // the linked dummy's parent is a shape
                        if ( _simIsShapeDynamicallyRespondable(linkedDummyParent)||(_simIsShapeDynamicallyStatic(linkedDummyParent)==0) )
                        { // the linked dummy's parent is dyn. or respondable
                            if ( _simGetTreeDynamicProperty(linkedDummyParent)&(sim_objdynprop_dynamic|sim_objdynprop_respondable) )
                            {
                                info->loopClosures.push_back(object); // this means a shape --> dummy -- dummy <-- shape loop closure
                                if (xmlDoc!=nullptr)
                                {
                                    _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(object,64);
                                    _simSetDynamicSimulationIconCode(linkedDummy,sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(linkedDummy,64);
                                }
                            }
                        }
                    }
                }
            }
        }
        if (child!=nullptr)
        { // joint or force sensor possibly involved in a loop closure
            if (_simGetObjectType(child)==sim_object_dummy_type)
            { // child is a dummy
                int linkedDummyHandle=-1;
                int linkType=_simGetDummyLinkType(child,&linkedDummyHandle);
                if ( (linkType==sim_dummy_linktype_dynamics_loop_closure)&&(linkedDummyHandle!=-1) )
                { // the dummy is linked to another dummy via a dyn. overlap constr.
                    CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
                    if ( (linkedDummyParent!=nullptr)&&(_simGetObjectType(linkedDummyParent)==sim_object_shape_type) )
                    { // the linked dummy's parent is a shape
                        if ( _simIsShapeDynamicallyRespondable(linkedDummyParent)||(_simIsShapeDynamicallyStatic(linkedDummyParent)==0) )
                        { // the linked dummy's parent is dyn. or respondable
                            if ( _simGetTreeDynamicProperty(linkedDummyParent)&(sim_objdynprop_dynamic|sim_objdynprop_respondable) )
                            {
                                _addShape(child,object,xmlDoc,info);
                                objectWasAdded=true;
                                ignoreChildren=true;
                                if (xmlDoc!=nullptr)
                                {
                                    _simSetDynamicSimulationIconCode(child,sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(child,64);
                                    _simSetDynamicSimulationIconCode(linkedDummy,sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(linkedDummy,64);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (!ignoreChildren)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            for (int i=0;i<childrenCount;i++)
            {
                if (objectWasAdded)
                    _addObjectBranch(childrenPointer[i],object,xmlDoc,info);
                else
                    info->moreToExplore.push_back(childrenPointer[i]);
            }
        }
        if ( objectWasAdded&&(xmlDoc!=nullptr) )
            xmlDoc->popNode();
    }
    return(moreToExploreInitSize!=info->moreToExplore.size());
}

void CRigidBodyContainerDyn::_addShape(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info)
{ // object can also be a loop closure dummy! (as in shape --> joint/fsensor --> dummy1 -- dummy2 <-- shape) In that case we add a dummy body at location of dummy1
    int objectHandle=_simGetObjectID(object);
    int dynProp=_simGetTreeDynamicProperty(object);
    bool forceStatic=((dynProp&sim_objdynprop_dynamic)==0);
    bool forceNonRespondable=((dynProp&sim_objdynprop_respondable)==0);
    int parentType=-1;
    if (parent!=nullptr)
        parentType=_simGetObjectType(parent);
    int flag=0;
    std::string objectName(_getObjectName(object));

    SMjShape g;
    g.objectHandle=objectHandle;
    g.name=objectName;
    if (parent==nullptr)
        g.shapeMode=shapeModes::freeMode; // might also be staticMode or kinematicMode. Decided further down
    else
        g.shapeMode=shapeModes::attachedMode;

    if (_simGetObjectType(object)==sim_object_dummy_type)
    { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
        if (xmlDoc==nullptr)
        { // We already know that dummy2 has a shape as parent, and the link type is overlap constr.
            int linkedDummyHandle=-1;
            _simGetDummyLinkType(object,&linkedDummyHandle);
            CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
            CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
            info->massDividers[linkedDummyParent]=info->massDividers[linkedDummyParent]+1;
        }
    }
    else
    {
        if (parent==nullptr)
        {
            if ( forceStatic||(_simIsShapeDynamicallyStatic(object)!=0) )
            { // we have a static shape.
                int kin;
                simGetObjectInt32Param(objectHandle,sim_shapeintparam_kinematic,&kin);
                if (kin==0)
                    g.shapeMode=shapeModes::staticMode;
                else
                { // The object is static AND kinematic (specific to Mujoco plugin): we need to add 2 bodies welded together: mocap and non-mocap
                    g.shapeMode=shapeModes::kinematicMode;
                    if (xmlDoc!=nullptr)
                    {
                        info->staticWelds.push_back(object);
                        xmlDoc->pushNewNode("body");
                        xmlDoc->setAttr("name",(objectName+"staticCounterpart").c_str());
                        float pos[3];
                        float quat[4];
                        _simGetObjectCumulativeTransformation(object,pos,quat,1);
                        xmlDoc->setPosAttr("pos",pos);
                        xmlDoc->setQuatAttr("quat",quat);
                        xmlDoc->setAttr("mocap",true);
                        xmlDoc->popNode();
                    }
                }
            }
        }
        if (g.shapeMode>=shapeModes::freeMode)
            flag=flag|2; // free or attached shapes
        if ( (!forceNonRespondable)&&_simIsShapeDynamicallyRespondable(object) )
            flag=flag|1;
    }

    if (xmlDoc!=nullptr)
        xmlDoc->pushNewNode("body");
    if (_simGetObjectType(object)==sim_object_dummy_type)
    { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
        forceNonRespondable=true;
        info->loopClosures.push_back(object);
        if (xmlDoc!=nullptr)
        {
            xmlDoc->setAttr("name",(objectName+"loop").c_str());
            xmlDoc->setAttr("mocap",forceStatic);
        }
    }
    else
    { // we have a shape
        if (xmlDoc!=nullptr)
        {
            xmlDoc->setAttr("name",objectName.c_str());
            CXGeomProxy* geom=(CXGeomProxy*)_simGetGeomProxyFromShape(object);
            CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geom);
            _simGetLocalInertiaFrame(geomInfo,g.shapeComTr.X.data,g.shapeComTr.Q.data);

            _allShapes.push_back(g);
            _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(object,flag);
        }
    }

    if (xmlDoc!=nullptr)
    {
        float pos[3];
        float quat[4];
        _simGetObjectCumulativeTransformation(object,pos,quat,1);
        xmlDoc->setPosAttr("pos",pos);
        xmlDoc->setQuatAttr("quat",quat);

        if (parent==nullptr)
        { // static or free shapes.
            if (g.shapeMode==shapeModes::staticMode)
                xmlDoc->setAttr("mocap",true);
            else
            {
                xmlDoc->pushNewNode("freejoint");
                xmlDoc->setAttr("name",(objectName+"freejoint").c_str());
                xmlDoc->popNode();
            }
        }
    }

    if (xmlDoc!=nullptr)
    {
        if (parentType==sim_object_joint_type)
        { // dyn. shape attached to joint (and consecutive joints are allowed!)
            std::vector<CXSceneObject*> parentJoints;
            while (_simGetObjectType(parent)==sim_object_joint_type)
            {
                parentJoints.push_back(parent);
                parent=(CXSceneObject*)_simGetParentObject(parent);
                if (parent==nullptr) // should not happen
                    break;
            }
            for (int i=int(parentJoints.size())-1;i>=0;i--)
            {
                parent=parentJoints[i];
                std::string parentName(_getObjectName(parent));
                int jt=_simGetJointType(parent);

                C7Vector jointPose;
                _simGetObjectCumulativeTransformation(parent,jointPose.X.data,jointPose.Q.data,1);
                xmlDoc->pushNewNode("joint");
                xmlDoc->setAttr("name",parentName.c_str());

                SMjJoint gjoint;

                if (jt==sim_joint_spherical_subtype)
                {
                    xmlDoc->setAttr("type","ball");
                    float p[7];
                    simGetObjectChildPose(_simGetObjectID(parent),p);
                    gjoint.initialBallQuat=C4Vector(p+3,true);
                }
                else
                {
                    if (jt==sim_joint_revolute_subtype)
                        xmlDoc->setAttr("type","hinge");
                    if (jt==sim_joint_prismatic_subtype)
                        xmlDoc->setAttr("type","slide");
                    float minp,rangep;
                    bool limited=_simGetJointPositionInterval(parent,&minp,&rangep);
                    xmlDoc->setAttr("limited",limited);
                    if (limited)
                        xmlDoc->setAttr("range",minp,minp+rangep);
                    xmlDoc->setAttr("ref",_simGetJointPosition(parent));
                }
                xmlDoc->setPosAttr("pos",jointPose.X.data);
                C4X4Matrix m(jointPose);
                xmlDoc->setPosAttr("axis",m.M.axis[2].data);
                xmlDoc->popNode();

                gjoint.objectHandle=_simGetObjectID(parent);
                gjoint.name=parentName;
                gjoint.jointType=jt;
                _allJoints.push_back(gjoint);

                _simSetDynamicSimulationIconCode(parent,sim_dynamicsimicon_objectisdynamicallysimulated);
                _simSetDynamicObjectFlagForVisualization(parent,4);
            }
        }
        if (parentType==sim_object_forcesensor_type)
        { // dyn. shape attached to force sensor
            std::string parentName(_getObjectName(parent));

            SMjForceSensor gfsensor;
            gfsensor.objectHandle=_simGetObjectID(parent);
            gfsensor.name=parentName;
            _allForceSensors.push_back(gfsensor);

            C7Vector sensorPose;
            _simGetObjectCumulativeTransformation(parent,sensorPose.X.data,sensorPose.Q.data,1);
            xmlDoc->pushNewNode("site");
            xmlDoc->setAttr("name",parentName.c_str());
            xmlDoc->setPosAttr("pos",sensorPose.X.data);
            xmlDoc->setQuatAttr("quat",sensorPose.Q.data);
            xmlDoc->popNode();

            _simSetDynamicSimulationIconCode(parent,sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(parent,32);
        }
    }

    if (xmlDoc!=nullptr)
    {
        if (_simGetObjectType(object)==sim_object_dummy_type)
        { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
            // We already know that dummy2 has a shape as parent, and the link type is overlap constr.
            int linkedDummyHandle=-1;
            _simGetDummyLinkType(object,&linkedDummyHandle);
            CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
            CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
            CXGeomProxy* geom=(CXGeomProxy*)_simGetGeomProxyFromShape(linkedDummyParent);
            CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geom);
            float mass=_simGetMass(geomInfo);
            mass/=float(info->massDividers[linkedDummyParent]);
            C7Vector tr,trinv,itr,dum1Tr;
            _simGetObjectCumulativeTransformation(object,dum1Tr.X.data,dum1Tr.Q.data,true);
            _simGetObjectCumulativeTransformation(linkedDummyParent,tr.X.data,tr.Q.data,true);
            _simGetObjectCumulativeTransformation(linkedDummy,trinv.X.data,trinv.Q.data,true);
            trinv.inverse();
            tr=trinv*tr;
            tr=dum1Tr*tr;
            _simGetLocalInertiaFrame(geomInfo,itr.X.data,itr.Q.data);
            tr=tr*itr;
            C3Vector im;
            _simGetPrincipalMomentOfInertia(geomInfo,im.data);

            /*
            xmlDoc->pushNewNode("geom");
            xmlDoc->setAttr("type","sphere");
            xmlDoc->setAttr("size",0.01);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
            xmlDoc->popNode();
            */
            _addInertiaElement(xmlDoc,mass,tr,im*mass);
        }
        else
        {
            if (!_addMeshes(object,xmlDoc,info,&_allGeoms))
                _simMakeDynamicAnnouncement(sim_announce_containsnonpurenonconvexshapes);
            if (g.shapeMode!=shapeModes::staticMode)
            {
                C7Vector tr,itr;
                C3Vector im;
                _simGetObjectCumulativeTransformation(object,tr.X.data,tr.Q.data,true);
/*
                CXGeomProxy* geom=(CXGeomProxy*)_simGetGeomProxyFromShape(object);
                CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geom);
                _simGetLocalInertiaFrame(geomInfo,itr.X.data,itr.Q.data);
                _simGetPrincipalMomentOfInertia(geomInfo,im.data);
                float mass=_simGetMass(geomInfo);
*/
                float mass=_simGetLocalInertiaInfo(object,itr.X.data,itr.Q.data,im.data);

                mass/=float(info->massDividers[object]); // the mass is possibly shared with a loop closure of type shape1 --> joint/fsensor --> dummy1(becomes aux. body) -- dummy2 <-- shape2
                if (g.shapeMode==shapeModes::kinematicMode)
                {
                    mass=1000.0f; // todo (allow this as a param)
                    im=C3Vector(1,1,1); // todo (allow this as a param)
                }
                tr=tr*itr;
                _addInertiaElement(xmlDoc,mass,tr,im);
            }
        }
    }
}

void CRigidBodyContainerDyn::_addInertiaElement(CXmlSer* xmlDoc,float mass,const C7Vector& tr,const C3Vector diagI)
{
    xmlDoc->pushNewNode("inertial");
    xmlDoc->setAttr("mass",mass);
    xmlDoc->setPosAttr("pos",tr.X.data);
    xmlDoc->setQuatAttr("quat",tr.Q.data);
    xmlDoc->setAttr("diaginertia",diagI(0),diagI(1),diagI(2));
    xmlDoc->popNode();
}

float CRigidBodyContainerDyn::computeInertia(int shapeHandle,C7Vector& tr,C3Vector& diagI,bool addRobustness)
{ // returns the diagonal mass-less inertia, for a density of 1000
    // Errors are silent here. Function can be reentrant (max. 1 time)
    mju_user_warning=nullptr;
    mju_user_error=nullptr;

    float mass=0.0;
    CXSceneObject* shape=(CXSceneObject*)_simGetObject(shapeHandle);
    char* _dir=simGetStringParam(sim_stringparam_tempdir);
    std::string dir(_dir);
    std::string mjFile(_dir);
    simReleaseBuffer(_dir);
    mjFile+="/inertiaCalc.xml";
    CXmlSer* xmlDoc=new CXmlSer(mjFile.c_str());
    xmlDoc->pushNewNode("worldbody");
    xmlDoc->pushNewNode("body");
    xmlDoc->setAttr("name","inertia");
    C7Vector shapeTr;
    _simGetObjectCumulativeTransformation(shape,shapeTr.X.data,shapeTr.Q.data,1);
    xmlDoc->setPosAttr("pos",shapeTr.X.data);
    xmlDoc->setQuatAttr("quat",shapeTr.Q.data);
    xmlDoc->pushNewNode("freejoint");
    xmlDoc->popNode();

    SInfo info;
    info.folder=dir;
    info.inertiaCalcRobust=addRobustness;
    _addMeshes(shape,xmlDoc,&info,nullptr);

    xmlDoc->popNode(); // body
    xmlDoc->popNode(); // worldbody

    xmlDoc->pushNewNode("asset");
    for (size_t i=0;i<info.meshFiles.size();i++)
    {
        xmlDoc->pushNewNode("mesh");
        xmlDoc->setAttr("file",info.meshFiles[i].c_str());
        xmlDoc->popNode();
    }
    xmlDoc->popNode(); // asset
    xmlDoc->popNode(); // file

    delete xmlDoc; // saves the file

    char error[1000] = "could not load binary model";
    mjModel* m=mj_loadXML(mjFile.c_str(),0,error,1000);
    if (m!=nullptr)
    {
        mjData* d=mj_makeData(m);
        int id=mj_name2id(m,mjOBJ_BODY,"inertia");
        mass=m->body_mass[id];
        diagI(0)=m->body_inertia[3*id+0];
        diagI(1)=m->body_inertia[3*id+1];
        diagI(2)=m->body_inertia[3*id+2];
        diagI/=mass;
        tr.X(0)=m->body_ipos[3*id+0];
        tr.X(1)=m->body_ipos[3*id+1];
        tr.X(2)=m->body_ipos[3*id+2];
        tr.Q(0)=m->body_iquat[4*id+0];
        tr.Q(1)=m->body_iquat[4*id+1];
        tr.Q(2)=m->body_iquat[4*id+2];
        tr.Q(3)=m->body_iquat[4*id+3];
        tr=shapeTr.getInverse()*tr;
        mj_deleteData(d);
        mj_deleteModel(m);
    }
    else
    {
        if (!addRobustness)
            mass=computeInertia(shapeHandle,tr,diagI,true);
    }
    mju_user_error=_errorCallback;
    mju_user_warning=_warningCallback;
    return(mass);
}


bool CRigidBodyContainerDyn::_addMeshes(CXSceneObject* object,CXmlSer* xmlDoc,SInfo* info,std::vector<SMjGeom>* geoms)
{ // retVal==false: display a warning if using non-pure non-convex shapes
    bool retVal=true;
    CXGeomProxy* geom=(CXGeomProxy*)_simGetGeomProxyFromShape(object); // even non respondable shapes have a geom
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geom);

    if ((_simGetPurePrimitiveType(geomInfo)==sim_primitiveshape_none)&&(_simIsGeomWrapConvex(geomInfo)==0)&&_simIsShapeDynamicallyRespondable(object)&&(_simGetTreeDynamicProperty(object)&sim_objdynprop_respondable))
        retVal=false;

    int componentListSize=_simGetGeometricCount(geomInfo);
    CXGeometric** componentList=new CXGeometric*[componentListSize];
    _simGetAllGeometrics(geomInfo,(simVoid**)componentList);
    C7Vector objectPose;
    _simGetObjectCumulativeTransformation(object,objectPose.X.data,objectPose.Q.data,1);
    for (int i=0;i<componentListSize;i++)
    {
        xmlDoc->pushNewNode("geom");
        std::string nm(_getObjectName(object)+std::to_string(i));
        xmlDoc->setAttr("name",nm.c_str());

        SMjGeom g;
        g.objectHandle=_simGetObjectID(object);
        g.name=nm;
        if (geoms!=nullptr)
            geoms->push_back(g);

        CXGeometric* sc=componentList[i];

        int pType=_simGetPurePrimitiveType(sc);
        C3Vector s;
        _simGetPurePrimitiveSizes(sc,s.data);

        C7Vector geomPose;
        geomPose.setIdentity();
        if (pType==sim_primitiveshape_heightfield)
        {
            xmlDoc->setAttr("type","hfield");
            std::string fn(_getObjectName(object)+std::to_string(i));
            xmlDoc->setAttr("hfield",fn.c_str());
            fn+=".bin";

            int xCnt,yCnt;
            float minH,maxH;
            const float* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH);

            float mmin=1000.0f;
            float mmax=-1000.0f;
            for (size_t j=0;j<xCnt*yCnt;j++)
            {
                float v=hData[j];
                if (v>mmax)
                    mmax=v;
                if (v<mmin)
                    mmin=v;
            }

            float mjhfbaseHeight=0.1*(s(0)+s(1));
            C3Vector pos(objectPose.X-objectPose.Q.getMatrix().axis[2]*(0.5f*(maxH-minH)));
            xmlDoc->setPosAttr("pos",pos.data);
            xmlDoc->setQuatAttr("quat",objectPose.Q.data);

            SHfield hf;
            hf.file=fn;
            hf.size[0]=0.5*s(0);
            hf.size[1]=0.5*s(1);
            hf.size[2]=maxH-minH;
            hf.size[3]=mjhfbaseHeight;
            info->heightfieldFiles.push_back(hf);

            fn=info->folder+"/"+fn;
            std::ofstream f(fn.c_str(),std::ios::out|std::ios::binary);
            f.write((char*)&yCnt,sizeof(int));
            f.write((char*)&xCnt,sizeof(int));
            f.write((char*)hData,sizeof(float)*xCnt*yCnt);
        }
        else
            _simGetVerticesLocalFrame(sc,geomPose.X.data,geomPose.Q.data);
        geomPose=objectPose*geomPose;

        if ( (pType==sim_primitiveshape_plane)||(pType==sim_primitiveshape_cuboid) )
        {
            float z=s(2);
            if (z<0.0001f)
                z=0.0001f;
            xmlDoc->setAttr("type","box");
            xmlDoc->setAttr("size",s(0)*0.5f,s(1)*0.5f,z*0.5f);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if ( (pType==sim_primitiveshape_disc)||(pType==sim_primitiveshape_cylinder) )
        {
            float z=s(2);
            if (z<0.0001f)
                z=0.0001f;
            xmlDoc->setAttr("type","cylinder");
            xmlDoc->setAttr("size",s(0)*0.5f,z*0.5f);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if (pType==sim_primitiveshape_cone)
        {
            pType=sim_primitiveshape_none; // handle it as a mesh
        }
        if (pType==sim_primitiveshape_spheroid)
        {
            if ( (fabs((s(0)-s(1))/s(0))<0.001f)&&(fabs((s(0)-s(2))/s(0))<0.001f) )
            {
                xmlDoc->setAttr("type","sphere");
                xmlDoc->setAttr("size",s(0)*0.5f);
                xmlDoc->setPosAttr("pos",geomPose.X.data);
                xmlDoc->setQuatAttr("quat",geomPose.Q.data);
            }
            else
            { // We have a spheroid
                xmlDoc->setAttr("type","ellipsoid");
                xmlDoc->setAttr("size",s(0)*0.5f,s(1)*0.5f,s(2)*0.5f);
                xmlDoc->setPosAttr("pos",geomPose.X.data);
                xmlDoc->setQuatAttr("quat",geomPose.Q.data);
            }
        }
        if (pType==sim_primitiveshape_capsule)
        {
            xmlDoc->setAttr("type","capsule");
            float avgR=(s(0)+s(1))*0.25f;
            xmlDoc->setAttr("size",avgR,s(2)*0.5f-avgR);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if (pType==sim_primitiveshape_none)
        {
            xmlDoc->setAttr("type","mesh");
            std::string fn(_getObjectName(object)+std::to_string(i));
            xmlDoc->setAttr("mesh",fn.c_str());
            fn+=".stl";
            info->meshFiles.push_back(fn);
            float* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
            float* mv=new float[allVerticesSize*4];
            int* mi=new int[allIndicesSize*4];
            float xmm[2]={9999.0,-9999.0};
            float ymm[2]={9999.0,-9999.0};
            float zmm[2]={9999.0,-9999.0};
            for (int j=0;j<allVerticesSize/3;j++)
            { // relative to the shape's frame
                C3Vector v(allVertices+3*j+0);
                v*=objectPose; //geomPose;
                mv[3*j+0]=v(0);
                mv[3*j+1]=v(1);
                mv[3*j+2]=v(2);
                if (v(0)<xmm[0])
                    xmm[0]=v(0);
                if (v(0)>xmm[1])
                    xmm[1]=v(0);
                if (v(1)<ymm[0])
                    ymm[0]=v(1);
                if (v(1)>ymm[1])
                    ymm[1]=v(1);
                if (v(2)<zmm[0])
                    zmm[0]=v(2);
                if (v(2)>zmm[1])
                    zmm[1]=v(2);
            }
            for (int j=0;j<allIndicesSize;j++)
                mi[j]=allIndices[j];
            if (info->inertiaCalcRobust)
            { // give mesh an artifical volume (is grown in 3 directions by 10%)
                float dxyz=0.1*(xmm[1]-xmm[0]+ymm[1]-ymm[0]+zmm[1]-zmm[0])/3;
                int voff=allVerticesSize;
                int ioff=allIndicesSize;
                int ioi=allVerticesSize/3;
                int io=ioi;;
                for (int k=0;k<3;k++)
                {
                    C3Vector w(C3Vector::zeroVector);
                    w(k)=dxyz;
                    for (int j=0;j<allVerticesSize/3;j++)
                    { // relative to the shape's frame
                        C3Vector v(allVertices+3*j+0);
                        v*=objectPose; //geomPose;
                        v+=w;
                        mv[voff++]=v(0);
                        mv[voff++]=v(1);
                        mv[voff++]=v(2);
                    }
                    for (int j=0;j<allIndicesSize;j++)
                        mi[ioff++]=allIndices[j]+io;
                    io+=ioi;
                }
                allVerticesSize*=4;
                allIndicesSize*=4;
            }
            fn=info->folder+"/"+fn;
            simExportMesh(4,fn.c_str(),0,1.0f,1,(const float**)&mv,&allVerticesSize,(const int**)&mi,&allIndicesSize,nullptr,nullptr);
            delete[] mi;
            delete[] mv;
            simReleaseBuffer((simChar*)allVertices);
            simReleaseBuffer((simChar*)allIndices);
        }

        xmlDoc->popNode();
    }
    return(retVal);
}

void CRigidBodyContainerDyn::handleDynamics(float dt,float simulationTime)
{
    if (!_simulationHalted)
    {
        float maxDynStep=simGetEngineFloatParam(sim_mujoco_global_stepsize,-1,nullptr,nullptr);
        _dynamicsCalculationPasses=int((dt/maxDynStep)+0.5f);
        if (_dynamicsCalculationPasses<1)
            _dynamicsCalculationPasses=1;
        _dynamicsInternalStepSize=dt/float(_dynamicsCalculationPasses);
        if (_firstDynPass)
        {
            std::string err=_buildMujocoWorld(_dynamicsInternalStepSize);
            if (err.size()>0)
                simAddLog(LIBRARY_NAME,sim_verbosity_errors,err.c_str());
            _firstDynPass=false;
        }
        if (_mjModel!=nullptr)
        {
            _simulationTime=simulationTime;
            _updateWorldFromCoppeliaSim();
            _contactInfo.clear();
            _contactPoints.clear();

            if (isDynamicContentAvailable())
            {
                for (int i=0;i<_dynamicsCalculationPasses;i++)
                {
                    _currentPass=i+1;
                    int integers[4]={0,i+1,_dynamicsCalculationPasses,0};
                    float floats[1]={_dynamicsInternalStepSize};
                    _simDynCallback(integers,floats);
                    _handleKinematicBodies_step(float(i+1)/float(_dynamicsCalculationPasses),dt);
 //                   printf("sim time: %f, %f\n",_simulationTime,_mjData->time);
                    _stepDynamics(_dynamicsInternalStepSize,i);
                    _handleContactPoints(i);
                    _simulationTime+=_dynamicsInternalStepSize;
                    _reportWorldToCoppeliaSim(_simulationTime,i,_dynamicsCalculationPasses);
                    integers[3]=1;
                    _simDynCallback(integers,floats);
                }
            }

            _clearAdditionalForcesAndTorques();
        }
    }
}

int CRigidBodyContainerDyn::_contactCallback(const mjModel* m,mjData* d,int geom1,int geom2)
{
    return(_dynWorld->_handleContact(m,d,geom1,geom2));
}

int CRigidBodyContainerDyn::_handleContact(const mjModel* m,mjData* d,int geom1,int geom2)
{
    int retVal=1; // ignore this contact
    int shape1Handle=_allGeoms[_geomIdIndex[geom1]].objectHandle;
    int shape2Handle=_allGeoms[_geomIdIndex[geom2]].objectHandle;
    if ( (shape1Handle>=0)&&(shape2Handle>=0) )
    {
        CXSceneObject* shapeA=(CXSceneObject*)_simGetObject(shape1Handle);
        CXSceneObject* shapeB=(CXSceneObject*)_simGetObject(shape2Handle);
        if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
        {
            unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
            unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
            bool canCollide=(_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB));
            if (canCollide)
            {
                CXSceneObject* lastPA=(CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeA);
                CXSceneObject* lastPB=(CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeB);
                if (lastPA==lastPB)
                    canCollide=(collFA&collFB&0x00ff); // we are local
                else
                    canCollide=(collFA&collFB&0xff00); // we are global
                if (canCollide)
                {
                    int dataInt[3]={0,0,0};
                    float dataFloat[14]={1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

                    int customHandleRes=_simHandleCustomContact(shape1Handle,shape2Handle,sim_physics_mujoco,dataInt,dataFloat);
                    if (customHandleRes!=0)
                        retVal=0; // should collide
                }
            }
        }
    }
    return(retVal);
}

void CRigidBodyContainerDyn::_controlCallback(const mjModel* m,mjData* d)
{
    _dynWorld->_handleControl(m,d);
}

void CRigidBodyContainerDyn::_handleControl(const mjModel* m,mjData* d)
{
    for (size_t i=0;i<_allShapes.size();i++)
    {
        CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
        if ( (shape!=nullptr)&&(_allShapes[i].shapeMode>=shapeModes::freeMode) )
        {
            int bodyId=_allShapes[i].mjId;
            C3Vector vf,vt;
            _simGetAdditionalForceAndTorque(shape,vf.data,vt.data);
            d->xfrc_applied[6*bodyId+0]=vf(0);
            d->xfrc_applied[6*bodyId+1]=vf(1);
            d->xfrc_applied[6*bodyId+2]=vf(2);
            d->xfrc_applied[6*bodyId+3]=vt(0);
            d->xfrc_applied[6*bodyId+4]=vt(1);
            d->xfrc_applied[6*bodyId+5]=vt(2);
        }
    }

    // Joints:
    std::vector<SMjJoint*> jointMujocoItems;
    std::vector<int> jointOrders;
    for (size_t i=0;i<_allJoints.size();i++)
    {
        CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if ( (joint!=nullptr)&&(_allJoints[i].actMode>0)&&(_allJoints[i].jointType!=sim_joint_spherical_subtype) )
        {
            _allJoints[i].object=joint;
            jointMujocoItems.push_back(&_allJoints[i]);
            if (_simGetJointDynCtrlMode(joint)==sim_jointdynctrl_callback)
                jointOrders.push_back(_simGetJointCallbackCallOrder(joint));
            else
                jointOrders.push_back(sim_scriptexecorder_normal);
        }
    }

    // First get control values:
    // handle first the higher priority joints:
    for (size_t i=0;i<jointOrders.size();i++)
    {
        if (jointOrders[i]<0)
            _handleMotorControl(jointMujocoItems[i],_currentPass,_dynamicsCalculationPasses);
    }
    // now the normal priority joints:
    for (size_t i=0;i<jointOrders.size();i++)
    {
        if (jointOrders[i]==0)
            _handleMotorControl(jointMujocoItems[i],_currentPass,_dynamicsCalculationPasses);
    }
    // now the low priority joints:
    for (size_t i=0;i<jointOrders.size();i++)
    {
        if (jointOrders[i]>0)
            _handleMotorControl(jointMujocoItems[i],_currentPass,_dynamicsCalculationPasses);
    }

    // Compute inverse dynamics to figure out force/torque values for a perfect velocity control:
    // mj_copyData(_mjDataCopy,m,d); // very slow
    _mjDataCopy->time=d->time;
    mju_copy(_mjDataCopy->qpos,d->qpos,m->nq);
    mju_copy(_mjDataCopy->qvel,d->qvel,m->nv);
    mju_copy(_mjDataCopy->act,d->act,m->na);
    mju_copy(_mjDataCopy->mocap_pos,d->mocap_pos,3*m->nmocap);
    mju_copy(_mjDataCopy->mocap_quat,d->mocap_quat,4*m->nmocap);
    mju_copy(_mjDataCopy->userdata,d->userdata,m->nuserdata);
    mju_copy(_mjDataCopy->qacc_warmstart,d->qacc_warmstart,m->nv);
    for (size_t i=0;i<jointMujocoItems.size();i++)
    {
        SMjJoint* mujocoItem=jointMujocoItems[i];
        if (mujocoItem->actMode==1)
            _mjDataCopy->ctrl[mujocoItem->mjId2]=mujocoItem->jointCtrlForceToApply;
        else
        {
            int vadr=m->jnt_dofadr[mujocoItem->mjId];
            _mjDataCopy->qacc[vadr]=mujocoItem->jointCtrlDv/m->opt.timestep;
        }
    }
    mj_inverse(m,_mjDataCopy);

    // Now apply joint forces/torques:
    for (size_t i=0;i<jointMujocoItems.size();i++)
    {
        SMjJoint* mujocoItem=jointMujocoItems[i];
        if (mujocoItem->actMode==1)
            d->ctrl[mujocoItem->mjId2]=mujocoItem->jointCtrlForceToApply;
        if (mujocoItem->actMode==2)
        {
            int vadr=m->jnt_dofadr[mujocoItem->mjId];
            double f=_mjDataCopy->qfrc_inverse[vadr];
            if (f>fabs(mujocoItem->jointCtrlForceToApply))
                f=fabs(mujocoItem->jointCtrlForceToApply);
            else if (f<-fabs(mujocoItem->jointCtrlForceToApply))
                f=-fabs(mujocoItem->jointCtrlForceToApply);
            d->ctrl[mujocoItem->mjId2]=f;
        }
    }

    _firstCtrlPass=false;
}

void CRigidBodyContainerDyn::_handleMotorControl(SMjJoint* mujocoItem,int passCnt,int totalPasses)
{
    CXSceneObject* joint=mujocoItem->object;
    int ctrlMode=_simGetJointDynCtrlMode(joint);
    float dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    float e=0.0f;
    int padr=_mjModel->jnt_qposadr[mujocoItem->mjId];
    float currentPos=_mjData->qpos[padr];
    int vadr=_mjModel->jnt_dofadr[mujocoItem->mjId];
    float currentVel=_mjData->qvel[vadr];
    float currentAccel=_mjData->qacc[vadr];
    if (mujocoItem->jointType==sim_joint_revolute_subtype)
    {
        if (ctrlMode>=sim_jointdynctrl_position)
        {
            if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
                e=_getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint),currentPos);
            else
                e=_simGetDynamicMotorTargetPosition(joint)-currentPos;
        }
    }
    else
    {
        if (ctrlMode>=sim_jointdynctrl_position)
            e=_simGetDynamicMotorTargetPosition(joint)-currentPos;
    }

    int auxV=2+4; // we provide vel and accel info too
    if (_firstCtrlPass)
        auxV|=1;
    int inputValuesInt[5]={0,0,0,0,0};
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    float inputValuesFloat[7]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    if (mujocoItem->jointType==sim_joint_revolute_subtype)
        inputValuesFloat[0]=currentPos;
    else
       inputValuesFloat[0]=currentPos;
    inputValuesFloat[1]=_mjData->actuator_force[mujocoItem->mjId2];
    inputValuesFloat[2]=dynStepSize;
    inputValuesFloat[3]=e;
    inputValuesFloat[4]=currentVel;
    inputValuesFloat[5]=currentAccel;
    float outputValues[5];
    int res=_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);

    if ((res&2)==0)
    { // motor is not locked
        if ((res&1)==1)
        {
            mujocoItem->jointCtrlDv=outputValues[0]-currentVel;
            mujocoItem->jointCtrlForceToApply=outputValues[1];
        }
        else
        {
            mujocoItem->jointCtrlDv=0.0;
            mujocoItem->jointCtrlForceToApply=0.0;
        }
    }
    else
    { // motor is locked. Not supported by Mujoco?!
    }
}

void CRigidBodyContainerDyn::_errorCallback(const char* err)
{
    simAddLog(LIBRARY_NAME,sim_verbosity_errors,err);
}

void CRigidBodyContainerDyn::_warningCallback(const char* warn)
{
    simAddLog(LIBRARY_NAME,sim_verbosity_warnings,warn);
    std::string str(warn);
    if (str.find("simulation is unstable")!=std::string::npos)
        _simulationHalted=true;
    if (str.find("constraint buffer is full")!=std::string::npos)
        _simulationHalted=true;
}

void CRigidBodyContainerDyn::_handleContactPoints(int dynPass)
{
    _contactPoints.clear();
    for (int i=0;i<_mjData->ncon;i++)
    {
        C3Vector pos(_mjData->contact[i].pos[0],_mjData->contact[i].pos[1],_mjData->contact[i].pos[2]); // midpoint
        _contactPoints.push_back(pos(0));
        _contactPoints.push_back(pos(1));
        _contactPoints.push_back(pos(2));
        // _mjData->contact[i]->dist // dist between pts, neg=penetration

        SContactInfo ci;
        ci.subPassNumber=dynPass;
        ci.objectID1=(unsigned long long)_allGeoms[_geomIdIndex[_mjData->contact[i].geom1]].objectHandle;
        ci.objectID2=(unsigned long long)_allGeoms[_geomIdIndex[_mjData->contact[i].geom2]].objectHandle;
        ci.position=pos;

        // _mjData->contact[i].frame is a transposed rotation matrix. Axis X is the contact normal vector
        mjtNum* frame=_mjData->contact[i].frame;
        ci.surfaceNormal=C3Vector(frame[0],frame[1],frame[2]);
        mjtNum ft[6];
        mj_contactForce(_mjModel,_mjData,i,ft); // forceTorque expressed in contact frame
        ci.directionAndAmplitude(0)=ft[0]*frame[0]+ft[1]*frame[3]+ft[2]*frame[6];
        ci.directionAndAmplitude(1)=ft[0]*frame[1]+ft[1]*frame[4]+ft[2]*frame[7];
        ci.directionAndAmplitude(2)=ft[0]*frame[2]+ft[1]*frame[5]+ft[2]*frame[8];
        _contactInfo.push_back(ci);
    }
}

dynReal CRigidBodyContainerDyn::_getAngleMinusAlpha(dynReal angle,dynReal alpha)
{    // Returns angle-alpha. Angle and alpha are cyclic angles!!
    dynReal sinAngle0 = sinf (angle);
    dynReal sinAngle1 = sinf (alpha);
    dynReal cosAngle0 = cosf(angle);
    dynReal cosAngle1 = cosf(alpha);
    dynReal sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
    dynReal cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
    dynReal angle_da = atan2(sin_da, cos_da);
    return angle_da;
}


std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    std::string v("Mujoco v");
    v+=mj_versionString();
    return(v);
}

bool CRigidBodyContainerDyn::_updateWorldFromCoppeliaSim()
{
    for (size_t i=0;i<_allShapes.size();i++)
    {
        if (_allShapes[i].shapeMode<=shapeModes::kinematicMode)
        { // prepare for static shape motion interpol.
            int bodyId=_mjModel->body_mocapid[_allShapes[i].mjId2];
            _allShapes[i].staticShapeStart.X=C3Vector(_mjData->mocap_pos[3*bodyId+0],_mjData->mocap_pos[3*bodyId+1],_mjData->mocap_pos[3*bodyId+2]);
            _allShapes[i].staticShapeStart.Q=C4Vector(_mjData->mocap_quat[4*bodyId+0],_mjData->mocap_quat[4*bodyId+1],_mjData->mocap_quat[4*bodyId+2],_mjData->mocap_quat[4*bodyId+3]);
            _allShapes[i].staticShapeGoal=_allShapes[i].staticShapeStart;
            CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
            if (shape!=nullptr)
                _simGetObjectCumulativeTransformation(shape,_allShapes[i].staticShapeGoal.X.data,_allShapes[i].staticShapeGoal.Q.data,false);
        }
    }
    return(true);
}

void CRigidBodyContainerDyn::_handleKinematicBodies_step(float t,float cumulatedTimeStep)
{
    for (size_t i=0;i<_allShapes.size();i++)
    {
        if (_allShapes[i].shapeMode<=shapeModes::kinematicMode)
        {
            CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
            if (shape!=nullptr)
            {
                int bodyId=_mjModel->body_mocapid[_allShapes[i].mjId2];
                C7Vector tr;
                tr.buildInterpolation(_allShapes[i].staticShapeStart,_allShapes[i].staticShapeGoal,t);
                _mjData->mocap_pos[3*bodyId+0]=tr.X(0);
                _mjData->mocap_pos[3*bodyId+1]=tr.X(1);
                _mjData->mocap_pos[3*bodyId+2]=tr.X(2);
                _mjData->mocap_quat[4*bodyId+0]=tr.Q(0);
                _mjData->mocap_quat[4*bodyId+1]=tr.Q(1);
                _mjData->mocap_quat[4*bodyId+2]=tr.Q(2);
                _mjData->mocap_quat[4*bodyId+3]=tr.Q(3);
            }
        }
    }
}

void CRigidBodyContainerDyn::_reportWorldToCoppeliaSim(float simulationTime,int currentPass,int totalPasses)
{
    for (size_t i=0;i<_allShapes.size();i++)
    { // objectHandle should be ordered so that each object's ancestor comes before
        CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
        if (shape!=nullptr)
        {
            int bodyId=_allShapes[i].mjId;
            if (_allShapes[i].shapeMode==shapeModes::freeMode)
            {
                C7Vector tr;
                tr.X=C3Vector(_mjData->xpos[3*bodyId+0],_mjData->xpos[3*bodyId+1],_mjData->xpos[3*bodyId+2]);
                tr.Q=C4Vector(_mjData->xquat[4*bodyId+0],_mjData->xquat[4*bodyId+1],_mjData->xquat[4*bodyId+2],_mjData->xquat[4*bodyId+3]);
                _simDynReportObjectCumulativeTransformation(shape,tr.X.data,tr.Q.data,simulationTime);
            }

            C3Vector av(_mjData->cvel[6*bodyId+0],_mjData->cvel[6*bodyId+1],_mjData->cvel[6*bodyId+2]);
            C3Vector lv(_mjData->cvel[6*bodyId+3],_mjData->cvel[6*bodyId+4],_mjData->cvel[6*bodyId+5]);
            // Above is COM vel.
            C4Vector q(_allShapes[i].shapeComTr.Q);
            av=q*av;
            lv=q*lv;
            _simSetShapeDynamicVelocity(shape,lv.data,av.data,simulationTime);
        }
    }

    for (size_t i=0;i<_allJoints.size();i++)
    {
        CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if (joint!=nullptr)
        {
            int padr=_mjModel->jnt_qposadr[_allJoints[i].mjId];
            int vadr=_mjModel->jnt_dofadr[_allJoints[i].mjId];
            if (_allJoints[i].jointType==sim_joint_spherical_subtype)
            {
                C4Vector q(_mjData->qpos[padr+0],_mjData->qpos[padr+1],_mjData->qpos[padr+2],_mjData->qpos[padr+3]);
                q=_allJoints[i].initialBallQuat*q;
                q.normalize();
                _simSetJointSphericalTransformation(joint,q.data,simulationTime);
            }
            else
            {
                _simSetJointPosition(joint,_mjData->qpos[padr]);
                int totalPassesCount=0;
                if (currentPass==totalPasses-1)
                    totalPassesCount=totalPasses;
                _simAddJointCumulativeForcesOrTorques(joint,-_mjData->actuator_force[_allJoints[i].mjId2],totalPassesCount,simulationTime);
                //  _simSetJointVelocity(joint,_mjData->qvel[vadr]);
            }
        }
    }

    for (size_t i=0;i<_allForceSensors.size();i++)
    {
        CXSceneObject* forceSens=(CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
        if (forceSens!=nullptr)
        {
            int fadr=_mjModel->sensor_adr[_allForceSensors[i].mjId];
            int tadr=_mjModel->sensor_adr[_allForceSensors[i].mjId2];
            int totalPassesCount=0;
            if (currentPass==totalPasses-1)
                totalPassesCount=totalPasses;
            float f[3]={float(-_mjData->sensordata[fadr+0]),float(-_mjData->sensordata[fadr+1]),float(-_mjData->sensordata[fadr+2])};
            float t[3]={float(-_mjData->sensordata[tadr+0]),float(-_mjData->sensordata[tadr+1]),float(-_mjData->sensordata[tadr+2])};
            _simAddForceSensorCumulativeForcesAndTorques(forceSens,f,t,totalPassesCount,simulationTime);
        }
    }
}

bool CRigidBodyContainerDyn::isDynamicContentAvailable()
{
    return(_allShapes.size()>0);
}

void CRigidBodyContainerDyn::_stepDynamics(float dt,int pass)
{
    mj_step(_mjModel,_mjData);
}
