#include <RigidBodyContainerDyn.h>
#include <CollShapeDyn.h>
#include <RigidBodyDyn.h>
#include <ConstraintDyn.h>
#include <simLib/simLib.h>
#include <simMath/4X4Matrix.h>

//bool CRigidBodyContainerDyn::_simulationHalted=false;

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine=sim_physics_physx;
    _engineVersion=0;
    gFoundation=nullptr;
    gPhysics=nullptr;
    gDispatcher=nullptr;
    gScene=nullptr;
    gPvd=nullptr;

    _objectCreationCounter=-1;
    _objectDestructionCounter=-1;
    _hierarchyChangeCounter=-1;

    /*
    _restartCount=0;
    _restartWarning=false;

    _mjModel=nullptr;
    _mjData=nullptr;
    _mjDataCopy=nullptr;
    _firstCtrlPass=true;
    _simulationHalted=false;
    _xmlInjectionChanged=false;
    _particleChanged=false;
    _nextCompositeHandle=-3;

    _rebuildTrigger=simGetEngineInt32Param(sim_mujoco_global_rebuildtrigger,-1,nullptr,nullptr);
    */
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
    if (gScene!=nullptr)
    {
        PX_RELEASE(gScene);
        PX_RELEASE(gDispatcher);
        PxCloseExtensions();
        PX_RELEASE(gPhysics);
        if (gPvd!=nullptr)
        {
            PxPvdTransport* transport = gPvd->getTransport();
            gPvd->release();
            PX_RELEASE(transport);
        }
        PX_RELEASE(gFoundation);
    }
    /*
    mju_user_warning=nullptr;
    mju_user_error=nullptr;
    mjcb_contactfilter=nullptr;
    mjcb_control=nullptr;
    mj_deleteData(_mjDataCopy);
    mj_deleteData(_mjData);
    mj_deleteModel(_mjModel);
    _xmlInjections.clear();
    _xmlCompositeInjections.clear();

    for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
    {
        CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(sim_handle_all,i);
        _simSetDynamicSimulationIconCode(it,sim_dynamicsimicon_none);
    }
    */
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams,intParams);
    return("");
}

std::string CRigidBodyContainerDyn::_buildPhysxWorld(double timeStep,double simTime,bool rebuild)
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    //*
    #define PVD_HOST "127.0.0.1"
    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
    //*/
    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
    PxInitExtensions(*gPhysics, gPvd);

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, 0.0f,-9.81f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher	= gDispatcher;
    sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,0,1,0), *gMaterial);
    gScene->addActor(*groundPlane);

//	PxRigidStatic* baseBody = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 2.0f, 0.0f)), PxBoxGeometry(1.0f, 2.0f, 3.0f), *gMaterial);
    float radius = 1.0f;
    PxRigidDynamic* sphereBody = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, 5.0f)), PxSphereGeometry(radius), *gMaterial, 1.0f);
    float halfHeight = 2.0f;
    PxRigidDynamic* capsuleBody = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(1.0f, 0.0f, 5.0f)), PxCapsuleGeometry(radius,halfHeight), *gMaterial,1.0f);
    PxRigidDynamic* boxBody = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(3.0f, 0.0f, 5.0f)), PxBoxGeometry(1.0,2.0,3.0), *gMaterial, 1.0f);
    gScene->addActor(*sphereBody);
//    gScene->addActor(*capsuleBody);
//    gScene->addActor(*boxBody);

    for (size_t i=0;i<1000;i++)
    {
        gScene->simulate(0.05);
        PxVec3 vel=sphereBody->getLinearVelocity();
        printf("Vel: %f, %f, %f\n",vel[0],vel[1],vel[2]);
        gScene->fetchResults(true);
    }
    /*
    mju_user_warning=nullptr;
    mju_user_error=nullptr;
    mjcb_contactfilter=nullptr;
    mjcb_control=nullptr;
    _xmlInjectionChanged=false;
    _particleChanged=false;
    _allGeoms.clear();
    _allJoints.clear();
    _allFreejoints.clear();
    _allForceSensors.clear();
    _allShapes.clear();
    _geomIdIndex.clear();
    mjModel* _mjPrevModel=nullptr;
    mjData* _mjPrevData=nullptr;

    for (size_t i=0;i<_xmlInjections.size();i++)
        _xmlInjections[i].xmlDummyString.clear();
    for (size_t i=0;i<_xmlCompositeInjections.size();i++)
        _xmlCompositeInjections[i].xmlDummyString.clear();

    _overrideKinematicFlag=simGetEngineInt32Param(sim_mujoco_global_overridekin,-1,nullptr,nullptr);
    char* _dir=simGetStringParam(sim_stringparam_mujocodir);
    std::string mjFile(_dir);
    std::string dir(_dir);
    simReleaseBuffer(_dir);
    if (rebuild)
    {
        mj_deleteData(_mjDataCopy);
        _mjPrevModel=_mjModel;
        _mjPrevData=_mjData;
        _restartCount++;
    }
    else
    {
        std::filesystem::remove_all(mjFile.c_str());
        std::filesystem::create_directory(mjFile.c_str());
    }
    mjFile+="/coppeliaSim.xml";
    CXmlSer* xmlDoc=new CXmlSer(mjFile.c_str());
    _addInjections(xmlDoc,-1,"mujoco");
    xmlDoc->pushNewNode("compiler");
    if (useGlobalCoords)
        xmlDoc->setAttr("coordinate","global");
    else
        xmlDoc->setAttr("coordinate","local");
    xmlDoc->setAttr("angle","radian");
    xmlDoc->setAttr("usethread",bool(simGetEngineBoolParam(sim_mujoco_global_multithreaded,-1,nullptr,nullptr)));
    xmlDoc->setAttr("balanceinertia",bool(simGetEngineBoolParam(sim_mujoco_global_balanceinertias,-1,nullptr,nullptr)));
    xmlDoc->setAttr("boundmass",simGetEngineFloatParam(sim_mujoco_global_boundmass,-1,nullptr,nullptr));
    xmlDoc->setAttr("boundinertia",simGetEngineFloatParam(sim_mujoco_global_boundinertia,-1,nullptr,nullptr));
    _addInjections(xmlDoc,-1,"compiler");
    xmlDoc->popNode();

    xmlDoc->pushNewNode("visual");
    _addInjections(xmlDoc,-1,"visual");
    xmlDoc->popNode();

    xmlDoc->pushNewNode("size");
    xmlDoc->setAttr("njmax",simGetEngineInt32Param(sim_mujoco_global_njmax,-1,nullptr,nullptr));
    xmlDoc->setAttr("nconmax",simGetEngineInt32Param(sim_mujoco_global_nconmax,-1,nullptr,nullptr));
    xmlDoc->setAttr("nstack",simGetEngineInt32Param(sim_mujoco_global_nstack,-1,nullptr,nullptr));
    xmlDoc->popNode();

    xmlDoc->pushNewNode("default");
    xmlDoc->pushNewNode("geom");
    xmlDoc->setAttr("rgba",0.8,0.6,0.4,1.0);
    xmlDoc->popNode();
    _addInjections(xmlDoc,-1,"default");
    xmlDoc->popNode();

    xmlDoc->pushNewNode("option");
    xmlDoc->setAttr("timestep",timeStep);
    xmlDoc->setAttr("impratio",simGetEngineFloatParam(sim_mujoco_global_impratio,-1,nullptr,nullptr));
    double w[5];
    for (size_t i=0;i<3;i++)
        w[i]=simGetEngineFloatParam(sim_mujoco_global_wind1+i,-1,nullptr,nullptr);
    xmlDoc->setAttr("wind",w,3);
    xmlDoc->setAttr("density",simGetEngineFloatParam(sim_mujoco_global_density,-1,nullptr,nullptr));
    xmlDoc->setAttr("viscosity",simGetEngineFloatParam(sim_mujoco_global_viscosity,-1,nullptr,nullptr));
    xmlDoc->setAttr("o_margin",simGetEngineFloatParam(sim_mujoco_global_overridemargin,-1,nullptr,nullptr));
    for (size_t i=0;i<2;i++)
        w[i]=simGetEngineFloatParam(sim_mujoco_global_overridesolref1+i,-1,nullptr,nullptr);
    xmlDoc->setAttr("o_solref",w,2);
    for (size_t i=0;i<2;i++)
        w[i]=simGetEngineFloatParam(sim_mujoco_global_overridesolimp1+i,-1,nullptr,nullptr);
    xmlDoc->setAttr("o_solimp",w,5);
    const char* integrator[]={"Euler","RK4","implicit"};
    int integratorIndex=simGetEngineInt32Param(sim_mujoco_global_integrator,-1,nullptr,nullptr);
    xmlDoc->setAttr("integrator",integrator[integratorIndex]);
    if (integratorIndex==1)
        _rg4Cnt=1; // using Runge-Kutta4
    else
        _rg4Cnt=0;
    const char* cone[]={"pyramidal","elliptic"};
    xmlDoc->setAttr("cone",cone[simGetEngineInt32Param(sim_mujoco_global_cone,-1,nullptr,nullptr)]);
    const char* solver[]={"PGS","CG","Newton"};
    xmlDoc->setAttr("solver",solver[simGetEngineInt32Param(sim_mujoco_global_solver,-1,nullptr,nullptr)]);
    xmlDoc->setAttr("iterations",simGetEngineInt32Param(sim_mujoco_global_iterations,-1,nullptr,nullptr));
    C3Vector gravity;
    _simGetGravity(gravity.data);
    xmlDoc->setAttr("gravity",gravity(0),gravity(1),gravity(2));
    xmlDoc->pushNewNode("flag");
    xmlDoc->setAttr("filterparent","disable");
    xmlDoc->setAttr("fwdinv","disable");
    const char* disableEnable[]={"disable","enable"};
    xmlDoc->setAttr("multiccd",disableEnable[simGetEngineBoolParam(sim_mujoco_global_multiccd,-1,nullptr,nullptr)]);
    xmlDoc->setAttr("override",disableEnable[simGetEngineBoolParam(sim_mujoco_global_overridecontacts,-1,nullptr,nullptr)]);
    xmlDoc->popNode();
    xmlDoc->popNode();

    xmlDoc->pushNewNode("worldbody");
    _addInjections(xmlDoc,-1,"worldbody");
    _addComposites(xmlDoc,-1,"worldbody");
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

    CParticleDyn::xmlDoc=xmlDoc;
    CParticleDyn::allGeoms=&_allGeoms;

    _particleCont->removeKilledParticles();
    _particleCont->addParticlesIfNeeded();

    SInfo info;
    info.folder=dir;
    info.inertiaCalcRobust=false;

    // We need to balance the mass of certain shapes across other shapes, when involved in loop closure with a joint/forceSensor, e.g.:
    // shape1 --> joint --> dummy1 -- dummy2 <-- shape2
    // In Mujoco we can only implement this if we place an aux. body where dummy1 is, and equality constrain it to be 'glued' to shape2
    // In order to not bias the mass/inertia with that aux. body, we split the mass/inertia of shape2 onto all aux. bodies that might
    // be 'glued' to it. This requires 2 exploration passes, where the first pass only adjusts the mass dividers
    std::map<CXSceneObject*,int> massDividers; // shape,divider
    int shapeListSize=_simGetObjectListSize(sim_sceneobject_shape);
    for (int i=0;i<shapeListSize;i++)
    {
        CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(sim_sceneobject_shape,i);
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
        info.tendons.clear();
        int orphanListSize=_simGetObjectListSize(-1);
        for (int i=0;i<orphanListSize;i++)
        {
            CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(-1,i);
            toExplore.push_back(it);
        }
        for (size_t i=0;i<toExplore.size();i++)
        {
            info.moreToExplore.clear();
            info.isTreeDynamic=false;
            if (_addObjectBranch(toExplore[i],nullptr,ser,&info))
                toExplore.insert(toExplore.end(),info.moreToExplore.begin(),info.moreToExplore.end());
        }

        if (ser!=nullptr)
        {
            xmlDoc->popNode();

            xmlDoc->pushNewNode("asset");
            _addInjections(xmlDoc,-1,"asset");
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


            xmlDoc->pushNewNode("tendon");
            _addInjections(xmlDoc,-1,"tendon");
            for (size_t i=0;i<info.tendons.size();i++)
            {
                CXSceneObject* dummy1=info.tendons[i];
                int dummy1Handle=_simGetObjectID(dummy1);
                int dummy2Handle=-1;
                _simGetDummyLinkType(dummy1,&dummy2Handle);
                CXSceneObject* dummy2=(CXSceneObject*)_simGetObject(dummy2Handle);
                if (dummy1Handle<dummy2Handle)
                { // make sure to not have 2 identical tendons!
                    xmlDoc->pushNewNode("spatial");
                    double solrefLimit[2];
                    double range[2];
                    for (size_t j=0;j<2;j++)
                    {
                        solrefLimit[j]=simGetEngineFloatParam(sim_mujoco_dummy_solreflimit1+j,-1,dummy1,nullptr);
                        range[j]=simGetEngineFloatParam(sim_mujoco_dummy_range1+j,-1,dummy1,nullptr);
                    }
                    double solimpLimit[5];
                    for (size_t j=0;j<5;j++)
                    {
                        solimpLimit[j]=simGetEngineFloatParam(sim_mujoco_dummy_solimplimit1+j,-1,dummy1,nullptr);
                    }
                    double stiffness=simGetEngineFloatParam(sim_mujoco_dummy_stiffness,-1,dummy1,nullptr);
                    double damping=simGetEngineFloatParam(sim_mujoco_dummy_damping,-1,dummy1,nullptr);
                    double springlength=simGetEngineFloatParam(sim_mujoco_dummy_springlength,-1,dummy1,nullptr);
                    double margin=simGetEngineFloatParam(sim_mujoco_dummy_margin,-1,dummy1,nullptr);
                    bool limited=simGetEngineBoolParam(sim_mujoco_dummy_limited,-1,dummy1,nullptr);
                    int proxyJointId=simGetEngineInt32Param(sim_mujoco_dummy_proxyjointid,-1,dummy1,nullptr);
                    CXSceneObject* proxyJoint=nullptr;
                    if (proxyJointId>=0)
                    {
                        proxyJoint=(CXSceneObject*)_simGetObject(proxyJointId);
                        if (_simGetObjectType(proxyJoint)==sim_sceneobject_joint)
                        {
                            if ( (!isJointInDynamicMode(proxyJoint))||(_simGetJointType(proxyJoint)!=sim_joint_prismatic) )
                            {
                                char* f=simGetObjectAlias(proxyJointId,5);
                                std::string nm(f);
                                simReleaseBuffer(f);
                                std::string msg("joint '");
                                msg+=nm+"' is referenced from a tendon constraint, but is not in dynamic mode, or is not a prismatic joint. The tendon will not operate as expected.";
                                simAddLog(LIBRARY_NAME,sim_verbosity_warnings,msg.c_str());
                                proxyJoint=nullptr;
                            }
                        }
                        else
                            proxyJoint=nullptr; // should never happen
                    }

                    if (proxyJoint!=nullptr)
                    {
                        SMjJoint gjoint;
                        gjoint.dependencyJointHandle=-1;
                        gjoint.objectHandle=proxyJointId;
                        gjoint.name=_getObjectName(proxyJoint);
                        gjoint.jointType=sim_joint_prismatic;
                        gjoint.tendonJoint=true;
                        _allJoints.push_back(gjoint);
                        _simSetDynamicSimulationIconCode(proxyJoint,sim_dynamicsimicon_objectisdynamicallysimulated);
                        _simSetDynamicObjectFlagForVisualization(proxyJoint,4);
                        xmlDoc->setAttr("name",gjoint.name.c_str()); // name comes from the proxy joint
                        double minp,rangep;
                        bool limited=_simGetJointPositionInterval(proxyJoint,&minp,&rangep);
                        xmlDoc->setAttr("limited",limited);
                        if (limited)
                            xmlDoc->setAttr("range",minp,minp+rangep);
                    }
                    else
                    {
                        xmlDoc->setAttr("name",_getObjectName(dummy1).c_str()); // name comes from the dummy with lowest handle
                        xmlDoc->setAttr("limited",limited);
                        xmlDoc->setAttr("range",range,2);
                    }
                    xmlDoc->setAttr("solreflimit",solrefLimit,2);
                    xmlDoc->setAttr("solimplimit",solimpLimit,5);
                    xmlDoc->setAttr("margin",margin);
                    xmlDoc->setAttr("springlength",springlength);
                    xmlDoc->setAttr("stiffness",stiffness);
                    xmlDoc->setAttr("damping",damping);

                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("site",_getObjectName(dummy1).c_str());
                    xmlDoc->popNode(); // site

                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("site",_getObjectName(dummy2).c_str());
                    xmlDoc->popNode(); // site

                    xmlDoc->popNode(); // spatial
                }
            }
            xmlDoc->popNode(); // tendon


            xmlDoc->pushNewNode("equality");
            _addInjections(xmlDoc,-1,"equality");
            for (size_t i=0;i<info.loopClosures.size();i++)
            {
                CXSceneObject* dummy1=info.loopClosures[i];
                int dummy1Handle=_simGetObjectID(dummy1);
                int dummy2Handle=-1;
                _simGetDummyLinkType(info.loopClosures[i],&dummy2Handle);
                CXSceneObject* dummy2=(CXSceneObject*)_simGetObject(dummy2Handle);
                CXSceneObject* shape2=(CXSceneObject*)_simGetParentObject(dummy2);
                CXSceneObject* dummy1Parent=(CXSceneObject*)_simGetParentObject(dummy1);
                if (_simGetObjectType(dummy1Parent)==sim_sceneobject_shape)
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
                        _simGetObjectCumulativeTransformation(dummy1,tr2.X.data,tr2.Q.data,1);
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
                    double v[7]={0.0,0.0,0.0,1.0,0.0,0.0,0.0};
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
            for (size_t i=0;i<_allJoints.size();i++)
            {
                if (_allJoints[i].dependencyJointHandle!=-1)
                {
                    CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
                    CXSceneObject* depJoint=(CXSceneObject*)_simGetObject(_allJoints[i].dependencyJointHandle);
                    xmlDoc->pushNewNode("joint");
                    xmlDoc->setAttr("joint1",_getObjectName(joint).c_str());
                    xmlDoc->setAttr("joint2",_getObjectName(depJoint).c_str());
                    xmlDoc->setAttr("polycoef",_allJoints[i].polycoef,5);
                    xmlDoc->popNode();
                }
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
            _addInjections(xmlDoc,-1,"actuator");
            for (size_t i=0;i<_allJoints.size();i++)
            {
                CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
                int m;
                simGetObjectInt32Param(_allJoints[i].objectHandle,sim_jointintparam_dynctrlmode,&m);
                xmlDoc->pushNewNode("motor");
                std::string nm(_getObjectName(joint));
                xmlDoc->setAttr("name",(nm+"act").c_str());
                if (_allJoints[i].tendonJoint)
                    xmlDoc->setAttr("tendon",nm.c_str());
                else
                    xmlDoc->setAttr("joint",nm.c_str());
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
    if (_xmlInjections.size()>0)
    {
        std::string xml(xmlDoc->getString());
        for (size_t i=0;i<_xmlInjections.size();i++)
        {
            std::string s(_xmlInjections[i].xmlDummyString);
            if (s.size()>0)
            {
                std::size_t p1=xml.find(s);
                if (p1!=std::string::npos)
                {
                    std::size_t p2=xml.find(s,p1+1);
                    xml.replace(p1-1,p2-p1+s.size()+2,_xmlInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }
    if (_xmlCompositeInjections.size()>0)
    {
        std::string xml(xmlDoc->getString());
        for (size_t i=0;i<_xmlCompositeInjections.size();i++)
        {
            std::string s(_xmlCompositeInjections[i].xmlDummyString);
            if (s.size()>0)
            {
                std::size_t p1=xml.find(s);
                if (p1!=std::string::npos)
                {
                    std::size_t p2=xml.find(s,p1+1);
                    xml.replace(p1-1,p2-p1+s.size()+2,_xmlCompositeInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }
    delete xmlDoc; // saves the file

    static bool passed=false;
    if (!passed)
        simAddLog(LIBRARY_NAME,sim_verbosity_warnings|sim_verbosity_onlyterminal,"make sure your CPU supports AVX, SSE and similar, otherwise you might see a crash now...");
    passed=true;

    std::string retVal;
    char error[1000] = "could not load binary model";
    _mjModel=mj_loadXML(mjFile.c_str(),0,error,1000);
    if (_mjModel!=nullptr)
    {
        _mjData=mj_makeData(_mjModel);
        _mjDataCopy=mj_makeData(_mjModel);

        CParticleDyn::mjModel=_mjModel;
        CParticleDyn::mjData=_mjData;

        _geomIdIndex.resize(_mjModel->ngeom,-1);
        int lastCompositeIndex=-1;
        std::string lastCompositePrefix;
        for (int i=0;i<int(_allGeoms.size());i++)
        { // _allGeoms might contain non-existing geoms from composites (e.g. for a m*n*o box we create m*n*o items in _allGeoms, for simplicity purpose)
            int mjId=mj_name2id(_mjModel,mjOBJ_GEOM,_allGeoms[i].name.c_str());
            if (_allGeoms[i].itemType==compositeItem)
            {
                if (_allGeoms[i].prefix!=lastCompositePrefix)
                {
                    lastCompositeIndex=getCompositeIndexFromPrefix(_allGeoms[i].prefix.c_str());
                    lastCompositePrefix=_allGeoms[i].prefix;
                }
                _xmlCompositeInjections[lastCompositeIndex].mjIds.push_back(mjId);
            }
            if (mjId>=0)
            {
                _allGeoms[i].mjId=mjId;
                _geomIdIndex[mjId]=i;
            }
            else
            {
                _allGeoms.erase(_allGeoms.begin()+i);
                i--;
            }
        }
        std::map<std::string,bool> sceneJoints;
        for (size_t i=0;i<_allJoints.size();i++)
        {
            _allJoints[i].object=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
            if (_allJoints[i].tendonJoint)
                _allJoints[i].mjId=mj_name2id(_mjModel,mjOBJ_TENDON,_allJoints[i].name.c_str());
            else
                _allJoints[i].mjId=mj_name2id(_mjModel,mjOBJ_JOINT,_allJoints[i].name.c_str());
            _allJoints[i].mjIdActuator=mj_name2id(_mjModel,mjOBJ_ACTUATOR,(_allJoints[i].name+"act").c_str());
            sceneJoints[_allJoints[i].name]=true;
        }
        for (size_t i=0;i<_allFreejoints.size();i++)
        {
            _allFreejoints[i].object=(CXSceneObject*)_simGetObject(_allFreejoints[i].objectHandle);
            _allFreejoints[i].mjId=mj_name2id(_mjModel,mjOBJ_JOINT,_allFreejoints[i].name.c_str());
            sceneJoints[_allFreejoints[i].name]=true;
        }
        for (size_t i=0;i<_allForceSensors.size();i++)
        {
            _allForceSensors[i].object=(CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
            _allForceSensors[i].mjId=mj_name2id(_mjModel,mjOBJ_SENSOR,(_allForceSensors[i].name+"force").c_str());
            _allForceSensors[i].mjId2=mj_name2id(_mjModel,mjOBJ_SENSOR,(_allForceSensors[i].name+"torque").c_str());
        }
        std::map<int,size_t> tempShapeMap;
        for (size_t i=0;i<_allShapes.size();i++)
        {
            tempShapeMap[_allShapes[i].objectHandle]=i;
            _allShapes[i].mjId=mj_name2id(_mjModel,mjOBJ_BODY,_allShapes[i].name.c_str());
            _allShapes[i].mjIdStatic=-1;
            _allShapes[i].mjIdJoint=-1;
            if (_allShapes[i].itemType==shapeItem)
            {
                _allShapes[i].object=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (_allShapes[i].shapeMode==shapeModes::kinematicMode)
                {
                    _allShapes[i].mjIdStatic=mj_name2id(_mjModel,mjOBJ_BODY,(_allShapes[i].name+"staticCounterpart").c_str());
                    _allShapes[i].mjIdJoint=mj_name2id(_mjModel,mjOBJ_JOINT,(_allShapes[i].name+"freejoint").c_str());
                }
                if (_allShapes[i].shapeMode==shapeModes::freeMode)
                { // handle initial velocity for free bodies:
                    _allShapes[i].mjIdJoint=mj_name2id(_mjModel,mjOBJ_JOINT,(_allShapes[i].name+"freejoint").c_str());
                    int nvadr=_mjModel->body_dofadr[_allShapes[i].mjId];
                    C3Vector v;
                    _simGetInitialDynamicVelocity(_allShapes[i].object,v.data);
                    if (v.getLength()>0.0)
                    {
                        _mjData->qvel[nvadr+0]=v(0);
                        _mjData->qvel[nvadr+1]=v(1);
                        _mjData->qvel[nvadr+2]=v(2);
                        _simSetInitialDynamicVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                    }
                    _simGetInitialDynamicAngVelocity(_allShapes[i].object,v.data);
                    if (v.getLength()>0.0)
                    {
                        _mjData->qvel[nvadr+3]=v(0);
                        _mjData->qvel[nvadr+4]=v(1);
                        _mjData->qvel[nvadr+5]=v(2);
                        _simSetInitialDynamicAngVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                    }
                }
            }
        }
        for (size_t i=0;i<_allGeoms.size();i++)
        {
            if (tempShapeMap.find(_allGeoms[i].objectHandle)!=tempShapeMap.end())
            {
                size_t ind=tempShapeMap[_allGeoms[i].objectHandle];
                _allShapes[ind].geomIndices.push_back(i);
            }
        }
        if (rebuild)
        {
            for (int jcnt=0;jcnt<_mjPrevModel->njnt;jcnt++)
            { // loop through all joints in the old model
                char* nm=_mjPrevModel->names+_mjPrevModel->name_jntadr[jcnt];
                if (_dynamicallyResetObjects.find(nm)==_dynamicallyResetObjects.end())
                { // that object was not dynamically reset on CoppeliaSim side
                    int mjId=mj_name2id(_mjModel,mjOBJ_JOINT,nm);
                    if (mjId>=0)
                    { // that joint is also present in the new model
                        int mjPrevId=mj_name2id(_mjPrevModel,mjOBJ_JOINT,nm);
                        if (_mjPrevModel->jnt_type[mjPrevId]==_mjModel->jnt_type[mjId])
                        { // same type (should actually always be the same type)
                            int vadr=_mjModel->jnt_dofadr[mjId];
                            int prevVadr=_mjPrevModel->jnt_dofadr[mjId];
                            size_t posd=1;
                            size_t dof=1;
                            if (_mjPrevModel->jnt_type[mjPrevId]==mjJNT_FREE)
                            {
                                posd=7;
                                dof=6;
                            }
                            if (_mjPrevModel->jnt_type[mjPrevId]==mjJNT_BALL)
                            {
                                posd=4;
                                dof=3;
                            }
                            for (size_t i=0;i<dof;i++)
                                _mjData->qvel[vadr+i]=_mjPrevData->qvel[prevVadr+i];
                            //---- not needed, but who knows... ---
                            for (size_t i=0;i<dof;i++)
                                _mjData->qacc[vadr+i]=_mjPrevData->qacc[prevVadr+i];
                            for (size_t i=0;i<dof;i++)
                                _mjData->qacc_warmstart[vadr+i]=_mjPrevData->qacc_warmstart[prevVadr+i];
                            //----------------------------
                            if (sceneJoints.find(nm)==sceneJoints.end())
                            { // those are not joints related to CoppeliaSim scene objects. Reuse also previous pos info:
                                int padr=_mjModel->jnt_qposadr[mjId];
                                int prevPadr=_mjPrevModel->jnt_qposadr[mjId];
                                for (size_t i=0;i<posd;i++)
                                    _mjData->qpos[padr+i]=_mjPrevData->qpos[prevPadr+i];
                            }
                        }
                    }
                }
            }
            _mjData->time=simTime;
            mj_deleteData(_mjPrevData);
            mj_deleteModel(_mjPrevModel);
            //---- not needed, but who knows... ---
            mj_energyPos(_mjModel,_mjData);
            mj_energyVel(_mjModel,_mjData);
            //----------------------------
            if ( (_restartCount>20)&&(_restartCount>int(simTime))&&(!_restartWarning) )
            {
                simAddLog(LIBRARY_NAME,sim_verbosity_warnings,"detected frequent dynamic world rebuilds. The MuJoCo engine is not optimized for frequent changes in the world, and the simulation might not be running in an optimal fashion.");
                _restartWarning=true;
            }
        }
        mjcb_contactfilter=_contactCallback;
        mjcb_control=_controlCallback;
        mju_user_error=_errorCallback;
        mju_user_warning=_warningCallback;
    }
    else
    {
        if (rebuild)
        {
            mj_deleteData(_mjPrevData);
            mj_deleteModel(_mjPrevModel);
        }
        retVal=error;
    }
    return(retVal);
    */
    return("");
}

void CRigidBodyContainerDyn::particlesAdded()
{
  //  _particleChanged=true;
}

std::string CRigidBodyContainerDyn::_getObjectName(CXSceneObject* object)
{
    int objectHandle=_simGetObjectID(object);
    char* f=simGetObjectAlias(objectHandle,4);
    std::string retVal(f);
    simReleaseBuffer(f);
    return(retVal);
}

/*
bool CRigidBodyContainerDyn::_addObjectBranch(CXSceneObject* object,CXSceneObject* parent,CXmlSer* xmlDoc,SInfo* info)
{
    size_t moreToExploreInitSize=info->moreToExplore.size();
    int dynProp=_simGetTreeDynamicProperty(object);
    if (dynProp&(sim_objdynprop_dynamic|sim_objdynprop_respondable))
    { // elements in tree could be dynamic and/or respondable
        int objType=_simGetObjectType(object);
        bool objectWasAdded=false;
        bool ignoreChildren=false;
        if (objType==sim_sceneobject_shape)
        {
            bool isStatic=((dynProp&sim_objdynprop_dynamic)==0)||(_simIsShapeDynamicallyStatic(object)!=0);
            if (isStatic)
            {
                if ( (_simIsShapeDynamicallyRespondable(object)!=0)&&info->isTreeDynamic )
                    _simMakeDynamicAnnouncement(sim_announce_containsstaticshapesondynamicconstruction);
            }
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
                if (parentType==sim_sceneobject_shape)
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
                            if (_simGetObjectType(child)==sim_sceneobject_forcesensor)
                            {
                                addIt=true;
                                break;
                            }
                            if ( (_simGetObjectType(child)==sim_sceneobject_joint)&&(isJointInDynamicMode(child)&&((_simGetTreeDynamicProperty(child)&sim_objdynprop_dynamic)!=0)) )
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
        if (objType==sim_sceneobject_joint)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&((_simGetObjectType(parent)==sim_sceneobject_shape)||(_simGetObjectType(parent)==sim_sceneobject_joint))&&(childrenCount==1)&&(isJointInDynamicMode(object)||_simIsJointInHybridOperation(object))&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape (or joint, consecutive joints are allowed!), joint is in correct mode, and has only one child
                if ( ( (_simGetObjectType(childrenPointer[0])==sim_sceneobject_shape)&&(_simIsShapeDynamicallyStatic(childrenPointer[0])==0) ) ||
                     ( (_simGetObjectType(childrenPointer[0])==sim_sceneobject_joint)&&isJointInDynamicMode(childrenPointer[0]) ) )
                { // child is a dyn. shape (or a dyn. joint)
                    _addObjectBranch(childrenPointer[0],object,xmlDoc,info);
                    ignoreChildren=true;
                }
                else
                    child=childrenPointer[0]; // check further down if that joint is involved in a loop closure
            }
        }
        if (objType==sim_sceneobject_forcesensor)
        {
            int childrenCount;
            CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_sceneobject_shape)&&(childrenCount==1)&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape, and force sensor has only one child
                if ( (_simGetObjectType(childrenPointer[0])==sim_sceneobject_shape)&&(_simIsShapeDynamicallyStatic(childrenPointer[0])==0) )
                { // child is a dyn. shape
                    _addObjectBranch(childrenPointer[0],object,xmlDoc,info);
                    ignoreChildren=true;
                }
                else
                    child=childrenPointer[0]; // check further down if that force sensor is involved in a loop closure
            }
        }
        if (objType==sim_sceneobject_dummy)
        {
            int childrenCount;
            _simGetObjectChildren(object,&childrenCount);
            if ( (parent!=nullptr)&&(_simGetObjectType(parent)==sim_sceneobject_shape)&&(childrenCount==0)&&(dynProp&sim_objdynprop_dynamic) )
            { // parent is shape, and dummy has no child
                int linkedDummyHandle=-1;
                int linkType=_simGetDummyLinkType(object,&linkedDummyHandle);
                if (linkedDummyHandle!=-1)
                { // there is a linked dummy
                    CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
                    _simGetObjectChildren(linkedDummy,&childrenCount);
                    if ( (linkedDummyParent!=nullptr)&&(_simGetObjectType(linkedDummyParent)==sim_sceneobject_shape)&&(childrenCount==0) )
                    { // linked dummy has a shape parent, and no children on its own
                        if ( _simIsShapeDynamicallyRespondable(linkedDummyParent)||(_simIsShapeDynamicallyStatic(linkedDummyParent)==0) )
                        { // the linked dummy's parent is dyn. or respondable
                            if ( _simGetTreeDynamicProperty(linkedDummyParent)&(sim_objdynprop_dynamic|sim_objdynprop_respondable) )
                            {
                                if (linkType==sim_dummylink_dynloopclosure)
                                    info->loopClosures.push_back(object); // this means a shape --> dummy -- dummy <-- shape loopClosure
                                if (linkType==sim_dummylink_dyntendon)
                                    info->tendons.push_back(object); // this means a shape --> dummy -- dummy <-- shape tendon
                                if ( (xmlDoc!=nullptr)&&((linkType==sim_dummylink_dynloopclosure)||(linkType==sim_dummylink_dyntendon)) )
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
                // for a dummy that has a shape parent, we always add a site, which can be used for
                // various purposes (some built-in like tendons, others set-up by users via script code)
                if (xmlDoc!=nullptr)
                {
                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("name",_getObjectName(object).c_str());
                    //-------------------
                    double pos[3];
                    double quat[4];
                    C7Vector tr;
                    if (useGlobalCoords)
                        _simGetObjectCumulativeTransformation(object,tr.X.data,tr.Q.data,1);
                    else
                    {
                        C7Vector pTrInv,objTr;
                        _simGetObjectCumulativeTransformation(parent,pTrInv.X.data,pTrInv.Q.data,true);
                        pTrInv.inverse();
                        _simGetObjectCumulativeTransformation(object,objTr.X.data,objTr.Q.data,true);
                        tr=pTrInv*objTr;
                    }
                    tr.X.getData(pos);
                    xmlDoc->setPosAttr("pos",pos);
                    tr.Q.getData(quat);
                    xmlDoc->setQuatAttr("quat",quat);
                    //-------------------
                    xmlDoc->popNode();
                }
            }
        }
        if (child!=nullptr)
        { // joint or force sensor possibly involved in a loop closure
            if (_simGetObjectType(child)==sim_sceneobject_dummy)
            { // child is a dummy
                int linkedDummyHandle=-1;
                int linkType=_simGetDummyLinkType(child,&linkedDummyHandle);
                if ( (linkType==sim_dummylink_dynloopclosure)&&(linkedDummyHandle!=-1) )
                { // the dummy is linked to another dummy via a dyn. overlap constr.
                    CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);
                    if ( (linkedDummyParent!=nullptr)&&(_simGetObjectType(linkedDummyParent)==sim_sceneobject_shape) )
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
{ // object can also be a loop closure dummy! (as in shape1 --> joint/fsensor --> DUMMY1 -- dummy2 <-- shape2).
  // In that case we need to add a dummy body inside of shape1, which contains the joint/fsensor
  // That dummy body needs to be the same as shape2, and the mass needs to be balanced across all dummy bodies,
  // check out info->massDividers
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

    if (_simGetObjectType(object)==sim_sceneobject_dummy)
    { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
        g.itemType=dummyShapeItem;
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
        g.itemType=shapeItem;
        if (parent==nullptr)
        {
            if ( forceStatic||(_simIsShapeDynamicallyStatic(object)!=0) )
            { // we have a static shape.
                int kin;
                simGetObjectInt32Param(objectHandle,sim_shapeintparam_kinematic,&kin);
                if (_overrideKinematicFlag==1)
                    kin=0;
                if (_overrideKinematicFlag==2)
                    kin=1;

                CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(object);
                if (_simGetPurePrimitiveType(geomInfo)==sim_primitiveshape_heightfield)
                    kin=0;
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
                        // -------------------
                        double pos[3];
                        double quat[4];
                        _simGetObjectCumulativeTransformation(object,pos,quat,1);
                        xmlDoc->setPosAttr("pos",pos);
                        xmlDoc->setQuatAttr("quat",quat);
                        // -------------------
                        xmlDoc->setAttr("mocap",true);
                        xmlDoc->popNode();
                    }
                }
            }
        }
        if (g.shapeMode>=shapeModes::freeMode)
        {
            flag=flag|2; // free or attached shapes
            info->isTreeDynamic=true;
        }
        if ( (!forceNonRespondable)&&_simIsShapeDynamicallyRespondable(object) )
            flag=flag|1;
    }

    if (xmlDoc!=nullptr)
        xmlDoc->pushNewNode("body");
    if (_simGetObjectType(object)==sim_sceneobject_dummy)
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
            _addInjections(xmlDoc,objectHandle,"");
            _addComposites(xmlDoc,objectHandle,"");
            xmlDoc->setAttr("name",objectName.c_str());
            _allShapes.push_back(g);
            _simSetDynamicSimulationIconCode(object,sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(object,flag);
        }
    }

    CXSceneObject* containingShape=parent; //i.e. shape1 as in shape1 --> joint/fsensor --> shape2/dummy
    while (containingShape!=nullptr)
    {
        if (_simGetObjectType(containingShape)==sim_sceneobject_shape)
            break;
        containingShape=(CXSceneObject*)_simGetParentObject(containingShape);
    }

    C7Vector objectPose; // pose of current shape (can also be a dummy shape!!)
    if (xmlDoc!=nullptr)
    {
        if (_simGetObjectType(object)==sim_sceneobject_dummy)
        { // we have dummy -- linkedDummy <- shape.
           // the dummy shape we create here is very special
            C7Vector tr;
            _simGetObjectCumulativeTransformation(object,tr.X.data,tr.Q.data,1);
            int linkedDummyHandle=-1;
            _simGetDummyLinkType(object,&linkedDummyHandle);
            CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
            C7Vector linkedDummyLocal;
            _simGetObjectLocalTransformation(linkedDummy,linkedDummyLocal.X.data,linkedDummyLocal.Q.data,1);
            objectPose=tr*linkedDummyLocal.getInverse();
            // objectPose is the abs. pose for the dummy shape (that doesn't exist in CoppeliaSim)
        }
        else
            _simGetObjectCumulativeTransformation(object,objectPose.X.data,objectPose.Q.data,1);

        // ------------------
        double pos[3];
        double quat[4];
        C7Vector tr;
        if ( useGlobalCoords||(parent==nullptr) )
            tr=objectPose;
        else
        {
            C7Vector pTr;
            _simGetObjectCumulativeTransformation(containingShape,pTr.X.data,pTr.Q.data,true);
            tr=pTr.getInverse()*objectPose;
        }
        tr.X.getData(pos);
        xmlDoc->setPosAttr("pos",pos);
        tr.Q.getData(quat);
        xmlDoc->setQuatAttr("quat",quat);
        // ------------------

        if (parent==nullptr)
        { // static or free shapes.
            if (g.shapeMode==shapeModes::staticMode)
                xmlDoc->setAttr("mocap",true);
            else
            {
                xmlDoc->pushNewNode("freejoint");
                xmlDoc->setAttr("name",(objectName+"freejoint").c_str());
                xmlDoc->popNode();
                SMjFreejoint fj;
                fj.objectHandle=objectHandle;
                fj.name=objectName+"freejoint";
                _allFreejoints.push_back(fj);
            }
        }
    }

    if (xmlDoc!=nullptr)
    {
        if (parentType==sim_sceneobject_joint)
        { // dyn. shape attached to joint (and consecutive joints are allowed!)
            std::vector<CXSceneObject*> parentJoints;
            CXSceneObject* jointIterator=parent;
            while (_simGetObjectType(jointIterator)==sim_sceneobject_joint)
            {
                parentJoints.push_back(jointIterator);
                jointIterator=(CXSceneObject*)_simGetParentObject(jointIterator);
                if (jointIterator==nullptr) // should not happen
                    break;
            }
            for (int i=int(parentJoints.size())-1;i>=0;i--)
            {
                CXSceneObject* joint=parentJoints[i];
                std::string jointName(_getObjectName(joint));
                int jt=_simGetJointType(joint);

                xmlDoc->pushNewNode("joint");
                double solrefLimit[2];
                double solrefFriction[2];
                double springdamper[2];
                for (size_t j=0;j<2;j++)
                {
                    solrefLimit[j]=simGetEngineFloatParam(sim_mujoco_joint_solreflimit1+j,-1,joint,nullptr);
                    solrefFriction[j]=simGetEngineFloatParam(sim_mujoco_joint_solreffriction1+j,-1,joint,nullptr);
                    springdamper[j]=simGetEngineFloatParam(sim_mujoco_joint_springdamper1+j,-1,joint,nullptr);
                }
                double solimpLimit[5];
                double solimpFriction[5];
                for (size_t j=0;j<5;j++)
                {
                    solimpLimit[j]=simGetEngineFloatParam(sim_mujoco_joint_solimplimit1+j,-1,joint,nullptr);
                    solimpFriction[j]=simGetEngineFloatParam(sim_mujoco_joint_solimpfriction1+j,-1,joint,nullptr);
                }
                double stiffness=simGetEngineFloatParam(sim_mujoco_joint_stiffness,-1,joint,nullptr);
                double damping=simGetEngineFloatParam(sim_mujoco_joint_damping,-1,joint,nullptr);
                double springref=simGetEngineFloatParam(sim_mujoco_joint_springref,-1,joint,nullptr);
                double armature=simGetEngineFloatParam(sim_mujoco_joint_armature,-1,joint,nullptr);
                double margin=simGetEngineFloatParam(sim_mujoco_joint_margin,-1,joint,nullptr);
                xmlDoc->setAttr("solreflimit",solrefLimit,2);
                xmlDoc->setAttr("solimplimit",solimpLimit,5);
                xmlDoc->setAttr("solreffriction",solrefFriction,2);
                xmlDoc->setAttr("solimpfriction",solimpFriction,5);
                xmlDoc->setAttr("stiffness",stiffness);
                xmlDoc->setAttr("damping",damping);
                xmlDoc->setAttr("springref",springref);
                xmlDoc->setAttr("armature",armature);
                xmlDoc->setAttr("margin",margin);
                xmlDoc->setAttr("name",jointName.c_str());

                SMjJoint gjoint;
#ifdef testingArmature
                gjoint.armature=armature;
                gjoint.maxForceOverArmature=0.0;
#endif
                if (jt==sim_joint_spherical)
                {
                    xmlDoc->setAttr("type","ball");
                    double p[7];
                    simGetObjectChildPose(_simGetObjectID(joint),p);
                    gjoint.initialBallQuat=C4Vector(p+3,true);
                    double p2[7];
                    simGetObjectPose(_simGetObjectID(joint),_simGetObjectID(containingShape),p2);
                    gjoint.initialBallQuat2=C4Vector(p2+3,true);
                    gjoint.dependencyJointHandle=-1;
                }
                else
                {
                    if (jt==sim_joint_revolute)
                        xmlDoc->setAttr("type","hinge");
                    if (jt==sim_joint_prismatic)
                        xmlDoc->setAttr("type","slide");
                    double minp,rangep;
                    bool limited=_simGetJointPositionInterval(joint,&minp,&rangep);
                    xmlDoc->setAttr("limited",limited);
                    if (limited)
                        xmlDoc->setAttr("range",minp,minp+rangep);
                    xmlDoc->setAttr("ref",_simGetJointPosition(joint));
                    double off,mult;
                    simGetJointDependency(_simGetObjectID(joint),&gjoint.dependencyJointHandle,&off,&mult);
                    //gjoint.dependencyJointHandle=simGetEngineInt32Param(sim_mujoco_joint_dependentobjectid,-1,joint,nullptr);
                    if (gjoint.dependencyJointHandle!=-1)
                    {
                        gjoint.polycoef[0]=off;
                        gjoint.polycoef[1]=mult;
                        for (size_t j=0;j<3;j++)
                            gjoint.polycoef[2+j]=simGetEngineFloatParam(sim_mujoco_joint_polycoef3+j,-1,joint,nullptr);
                    }
                }

                // ---------------------
                double pos[3];
                C7Vector tr;
                if (useGlobalCoords)
                    _simGetObjectCumulativeTransformation(joint,tr.X.data,tr.Q.data,1);
                else
                {
                    C7Vector jointTr;
                    _simGetObjectCumulativeTransformation(joint,jointTr.X.data,jointTr.Q.data,true);
                    tr=objectPose.getInverse()*jointTr;
                }
                tr.X.getData(pos);
                xmlDoc->setPosAttr("pos",pos);
                C4X4Matrix m(tr);
                xmlDoc->setPosAttr("axis",m.M.axis[2].data);
                // ---------------------

                xmlDoc->popNode();

                gjoint.objectHandle=_simGetObjectID(joint);
                gjoint.name=jointName;
                gjoint.jointType=jt;
                gjoint.tendonJoint=false;
                _allJoints.push_back(gjoint);

                _simSetDynamicSimulationIconCode(joint,sim_dynamicsimicon_objectisdynamicallysimulated);
                _simSetDynamicObjectFlagForVisualization(joint,4);
            }
        }
        if (parentType==sim_sceneobject_forcesensor)
        { // dyn. shape attached to force sensor
            CXSceneObject* forceSensor=parent;

            std::string forceSensorName(_getObjectName(forceSensor));

            SMjForceSensor gfsensor;
            gfsensor.objectHandle=_simGetObjectID(forceSensor);
            gfsensor.name=forceSensorName;
            _allForceSensors.push_back(gfsensor);

            xmlDoc->pushNewNode("site");
            xmlDoc->setAttr("name",forceSensorName.c_str());

            // -----------------
            double pos[3];
            double quat[4];
            C7Vector tr;
            if (useGlobalCoords)
                _simGetObjectCumulativeTransformation(forceSensor,tr.X.data,tr.Q.data,1);
            else
            {
                C7Vector sensorTr;
                _simGetObjectCumulativeTransformation(forceSensor,sensorTr.X.data,sensorTr.Q.data,true);
                tr=objectPose.getInverse()*sensorTr;
            }
            tr.X.getData(pos);
            xmlDoc->setPosAttr("pos",pos);
            tr.Q.getData(quat);
            xmlDoc->setQuatAttr("quat",quat);
            // -----------------

            xmlDoc->popNode();

            _simSetDynamicSimulationIconCode(forceSensor,sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(forceSensor,32);
        }
    }

    if (xmlDoc!=nullptr)
    {
        if (_simGetObjectType(object)==sim_sceneobject_dummy)
        { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
            // We already know that dummy2 has a shape as parent, and the link type is overlap constr.
            // Keep in mind that dummy1 and dummy2 might not (yet) be overlapping!
            int linkedDummyHandle=-1;
            _simGetDummyLinkType(object,&linkedDummyHandle);
            CXDummy* linkedDummy=(CXDummy*)_simGetObject(linkedDummyHandle);
            CXSceneObject* linkedDummyParent=(CXSceneObject*)_simGetParentObject(linkedDummy);

            C7Vector tr;
            C3Vector im;
            double mass=_simGetLocalInertiaInfo(linkedDummyParent,tr.X.data,tr.Q.data,im.data); // im includes the mass!
            mass/=double(info->massDividers[linkedDummyParent]);
            im/=double(info->massDividers[linkedDummyParent]);
            if (useGlobalCoords)
                tr=objectPose*tr;
            _addInertiaElement(xmlDoc,mass,tr,im);
        }
        else
        {
            if (!_addMeshes(object,xmlDoc,info,&_allGeoms,forceStatic||(_simIsShapeDynamicallyStatic(object)!=0)))
                _simMakeDynamicAnnouncement(sim_announce_containsnonpurenonconvexshapes);
            if (g.shapeMode!=shapeModes::staticMode)
            {
                C7Vector tr;
                C3Vector im;
                double mass=_simGetLocalInertiaInfo(object,tr.X.data,tr.Q.data,im.data); // im includes the mass!
                mass/=double(info->massDividers[object]); // the mass is possibly shared with a loop closure of type shape1 --> joint/fsensor --> dummy1(becomes aux. body) -- dummy2 <-- shape2
                im/=double(info->massDividers[object]);
                if (useGlobalCoords)
                    tr=objectPose*tr;
                if (g.shapeMode==shapeModes::kinematicMode)
                {
                    mass=simGetEngineFloatParam(sim_mujoco_global_kinmass,-1,nullptr,nullptr);
                    double inertia=simGetEngineFloatParam(sim_mujoco_global_kininertia,-1,nullptr,nullptr);
                    im=C3Vector(inertia,inertia,inertia);
                }
                _addInertiaElement(xmlDoc,mass,tr,im);
            }
        }
    }
}
*/

/*
bool CRigidBodyContainerDyn::_addMeshes(CXSceneObject* object,CXmlSer* xmlDoc,SInfo* info,std::vector<SMjGeom>* geoms,bool shapeIsStatic)
{ // retVal==false: display a warning if using non-pure non-convex shapes
    bool retVal=true;
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(object);

    if ((_simGetPurePrimitiveType(geomInfo)==sim_primitiveshape_none)&&(_simIsGeomWrapConvex(geomInfo)==0)&&_simIsShapeDynamicallyRespondable(object)&&(_simGetTreeDynamicProperty(object)&sim_objdynprop_respondable))
        retVal=false;

    double friction[3];
    friction[0]=simGetEngineFloatParam(sim_mujoco_body_friction1,-1,object,nullptr);
    friction[1]=simGetEngineFloatParam(sim_mujoco_body_friction2,-1,object,nullptr);
    friction[2]=simGetEngineFloatParam(sim_mujoco_body_friction3,-1,object,nullptr);
    double solref[2];
    solref[0]=simGetEngineFloatParam(sim_mujoco_body_solref1,-1,object,nullptr);
    solref[1]=simGetEngineFloatParam(sim_mujoco_body_solref2,-1,object,nullptr);
    double solimp[5];
    solimp[0]=simGetEngineFloatParam(sim_mujoco_body_solimp1,-1,object,nullptr);
    solimp[1]=simGetEngineFloatParam(sim_mujoco_body_solimp2,-1,object,nullptr);
    solimp[2]=simGetEngineFloatParam(sim_mujoco_body_solimp3,-1,object,nullptr);
    solimp[3]=simGetEngineFloatParam(sim_mujoco_body_solimp4,-1,object,nullptr);
    solimp[4]=simGetEngineFloatParam(sim_mujoco_body_solimp5,-1,object,nullptr);
    double solmix=simGetEngineFloatParam(sim_mujoco_body_solmix,-1,object,nullptr);
    int condim=simGetEngineInt32Param(sim_mujoco_body_condim,-1,object,nullptr);
    double margin=simGetEngineFloatParam(sim_mujoco_body_margin,-1,object,nullptr);
    int priority=simGetEngineInt32Param(sim_mujoco_body_priority,-1,object,nullptr);

    int componentListSize=_simGetGeometricCount(geomInfo);
    CXGeometric** componentList=new CXGeometric*[componentListSize];
    _simGetAllGeometrics(geomInfo,(void**)componentList);
    C7Vector objectPose;
    if (useGlobalCoords)
        _simGetObjectCumulativeTransformation(object,objectPose.X.data,objectPose.Q.data,1);
    else
        objectPose=C7Vector::identityTransformation;
    for (int i=0;i<componentListSize;i++)
    {
        xmlDoc->pushNewNode("geom");
        xmlDoc->setAttr("friction",friction,3);
        xmlDoc->setAttr("solref",solref,2);
        xmlDoc->setAttr("solimp",solimp,5);
        xmlDoc->setAttr("solmix",solmix);
        xmlDoc->setAttr("condim",condim);
        xmlDoc->setAttr("margin",margin);
        xmlDoc->setAttr("priority",priority);
        std::string nm(_getObjectName(object)+std::to_string(i));
        xmlDoc->setAttr("name",nm.c_str());

        SMjGeom g;
        g.belongsToStaticItem=shapeIsStatic;
        g.itemType=shapeItem;
        g.objectHandle=_simGetObjectID(object);
        g.name=nm;
        g.respondableMask=_simGetDynamicCollisionMask(object);

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
            double minH,maxH;
            const double* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH);

            double mmin=1000.0;
            double mmax=-1000.0;
            std::vector<float> hDataF;
            hDataF.resize(xCnt*yCnt);
            for (size_t j=0;j<xCnt*yCnt;j++)
            {
                hDataF[j]=hData[j];
                double v=hData[j];
                if (v>mmax)
                    mmax=v;
                if (v<mmin)
                    mmin=v;
            }

            double mjhfbaseHeight=0.1*(s(0)+s(1));
            C3Vector pos(objectPose.X-objectPose.Q.getMatrix().axis[2]*(0.5*(maxH-minH)));
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
            f.write((char*)hDataF.data(),sizeof(float)*xCnt*yCnt);
        }
        else
            _simGetVerticesLocalFrame(object,sc,geomPose.X.data,geomPose.Q.data);
        geomPose=objectPose*geomPose;

        if ( (pType==sim_primitiveshape_plane)||(pType==sim_primitiveshape_cuboid) )
        {
            double z=s(2);
            if (z<0.0001)
                z=0.0001;
            xmlDoc->setAttr("type","box");
            xmlDoc->setAttr("size",s(0)*0.5,s(1)*0.5,z*0.5);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if ( (pType==sim_primitiveshape_disc)||(pType==sim_primitiveshape_cylinder) )
        {
            double z=s(2);
            if (z<0.0001)
                z=0.0001;
            xmlDoc->setAttr("type","cylinder");
            xmlDoc->setAttr("size",s(0)*0.5,z*0.5);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if (pType==sim_primitiveshape_cone)
        {
            pType=sim_primitiveshape_none; // handle it as a mesh
        }
        if (pType==sim_primitiveshape_spheroid)
        {
            if ( (fabs((s(0)-s(1))/s(0))<0.001)&&(fabs((s(0)-s(2))/s(0))<0.001) )
            {
                xmlDoc->setAttr("type","sphere");
                xmlDoc->setAttr("size",s(0)*0.5);
                xmlDoc->setPosAttr("pos",geomPose.X.data);
                xmlDoc->setQuatAttr("quat",geomPose.Q.data);
            }
            else
            { // We have a spheroid
                xmlDoc->setAttr("type","ellipsoid");
                xmlDoc->setAttr("size",s(0)*0.5,s(1)*0.5,s(2)*0.5);
                xmlDoc->setPosAttr("pos",geomPose.X.data);
                xmlDoc->setQuatAttr("quat",geomPose.Q.data);
            }
        }
        if (pType==sim_primitiveshape_capsule)
        {
            xmlDoc->setAttr("type","capsule");
            double avgR=(s(0)+s(1))*0.25;
            xmlDoc->setAttr("size",avgR,s(2)*0.5-avgR);
            xmlDoc->setPosAttr("pos",geomPose.X.data);
            xmlDoc->setQuatAttr("quat",geomPose.Q.data);
        }
        if (pType==sim_primitiveshape_none)
        {
            xmlDoc->setAttr("type","mesh");
            std::string fn(_getObjectName(object)+std::to_string(i)); // ideally get the hash of the local mesh for the filename. This would avoid creating file duplicates with identical meshes
            xmlDoc->setAttr("mesh",fn.c_str());
            fn+=".stl";
            info->meshFiles.push_back(fn);
            double* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(object,sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
            double* mv=new double[allVerticesSize*4];
            int* mi=new int[allIndicesSize*4];
            double xmm[2]={9999.0,-9999.0};
            double ymm[2]={9999.0,-9999.0};
            double zmm[2]={9999.0,-9999.0};
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
                double dxyz=0.1*(xmm[1]-xmm[0]+ymm[1]-ymm[0]+zmm[1]-zmm[0])/3;
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
            if (simDoesFileExist(fn.c_str())==0)
                simExportMesh(4,fn.c_str(),0,1.0,1,(const double**)&mv,&allVerticesSize,(const int**)&mi,&allIndicesSize,nullptr,nullptr);
            delete[] mi;
            delete[] mv;
            simReleaseBuffer((char*)allVertices);
            simReleaseBuffer((char*)allIndices);
        }

        xmlDoc->popNode();
    }
    return(retVal);
}
*/

int CRigidBodyContainerDyn::_hasContentChanged()
{
    int occ,odc,hcc;
    simGetInt32Param(sim_intparam_objectcreationcounter,&occ);
    simGetInt32Param(sim_intparam_objectdestructioncounter,&odc);
    simGetInt32Param(sim_intparam_hierarchychangecounter,&hcc);
    int retVal=0;
    if (_objectCreationCounter==-1)
        retVal=1; // there is no content yet
    else
    { // in here we could eventually look a bit closer to see if the change is really involving pyhsics-simulated objects...
        /*
        if ( (_objectCreationCounter!=occ)&&(_rebuildTrigger&1) )
            retVal=2;
        if ( (_objectDestructionCounter!=odc)&&(_rebuildTrigger&2) )
            retVal=2;
        if ( (_hierarchyChangeCounter!=hcc)&&(_rebuildTrigger&4) )
            retVal=2;
        if ( _xmlInjectionChanged&&(_rebuildTrigger&16) )
            retVal=2;
        if ( _particleChanged&&(_rebuildTrigger&32) )
            retVal=2;
        if ( (retVal==0)&&(_rebuildTrigger&8) )
        {
            _dynamicallyResetObjects.clear();
            int lSize=_simGetObjectListSize(sim_handle_all);
            for (int i=0;i<lSize;i++)
            {
                CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(sim_handle_all,i);
                if (_simGetDynamicsFullRefreshFlag(it)!=0)
                {
                    retVal=2;
                    _simSetDynamicsFullRefreshFlag(it,false);
                    _dynamicallyResetObjects[_getObjectName(it)];
                }
            }
        }
        */
    }
    _objectCreationCounter=occ;
    _objectDestructionCounter=odc;
    _hierarchyChangeCounter=hcc;
    return(retVal);
}

void CRigidBodyContainerDyn::handleDynamics(double dt,double simulationTime)
{
    double maxDynStep;
    simGetFloatParam(sim_floatparam_physicstimestep,&maxDynStep);

    _dynamicsCalculationPasses=int((dt/maxDynStep)+0.5);
    if (_dynamicsCalculationPasses<1)
        _dynamicsCalculationPasses=1;
    _dynamicsInternalStepSize=dt/double(_dynamicsCalculationPasses);
    int contentChanged=_hasContentChanged();
    if (contentChanged>0)
    {
        std::string err=_buildPhysxWorld(_dynamicsInternalStepSize,simulationTime,contentChanged>1);
        if (err.size()>0)
            simAddLog(LIBRARY_NAME,sim_verbosity_errors,err.c_str());
    }
    /*
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
                double floats[1]={_dynamicsInternalStepSize};
                _simDynCallback(integers,floats);
                _handleKinematicBodies_step(double(i+1)/double(_dynamicsCalculationPasses),dt);
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
    */
}

double CRigidBodyContainerDyn::_getAngleMinusAlpha(double angle,double alpha)
{    // Returns angle-alpha. Angle and alpha are cyclic angles!!
    double sinAngle0 = sinf (angle);
    double sinAngle1 = sinf (alpha);
    double cosAngle0 = cosf(angle);
    double cosAngle1 = cosf(alpha);
    double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
    double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
    double angle_da = atan2(sin_da, cos_da);
    return angle_da;
}


std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    std::string v("PhysX v");
    v+=std::to_string(PX_PHYSICS_VERSION_MAJOR)+"."+std::to_string(PX_PHYSICS_VERSION_MINOR);
    return(v);
}

bool CRigidBodyContainerDyn::_updateWorldFromCoppeliaSim()
{
    /*
    for (size_t i=0;i<_allShapes.size();i++)
    {
        if (_allShapes[i].itemType==shapeItem)
        { // only shapes that exist in CoppeliaSim
            if (_allShapes[i].shapeMode<=shapeModes::kinematicMode)
            { // prepare for static shape motion interpol.
                int bodyId=_mjModel->body_mocapid[_allShapes[i].mjIdStatic];
                _allShapes[i].staticShapeStart.X=C3Vector(_mjData->mocap_pos[3*bodyId+0],_mjData->mocap_pos[3*bodyId+1],_mjData->mocap_pos[3*bodyId+2]);
                _allShapes[i].staticShapeStart.Q=C4Vector(_mjData->mocap_quat[4*bodyId+0],_mjData->mocap_quat[4*bodyId+1],_mjData->mocap_quat[4*bodyId+2],_mjData->mocap_quat[4*bodyId+3]);
                _allShapes[i].staticShapeGoal=_allShapes[i].staticShapeStart;
                CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (shape!=nullptr)
                {
                        _simGetObjectCumulativeTransformation(shape,_allShapes[i].staticShapeGoal.X.data,_allShapes[i].staticShapeGoal.Q.data,false);
                }
            }
        }
    }
*/
    return(true);
}

void CRigidBodyContainerDyn::_handleKinematicBodies_step(double t,double cumulatedTimeStep)
{
    /*
    for (size_t i=0;i<_allShapes.size();i++)
    {
        if (_allShapes[i].itemType==shapeItem)
        { // only shapes that exist in CoppeliaSim
            if (_allShapes[i].shapeMode<=shapeModes::kinematicMode)
            {
                CXSceneObject* shape=(CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (shape!=nullptr)
                {
                    int bodyId=_mjModel->body_mocapid[_allShapes[i].mjIdStatic];
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
    */
}

void CRigidBodyContainerDyn::_reportWorldToCoppeliaSim(double simulationTime,int currentPass,int totalPasses)
{
/*
    // First joints:
    for (size_t i=0;i<_allJoints.size();i++)
    {
        CXSceneObject* joint=(CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if (joint!=nullptr)
        {
            if (_allJoints[i].tendonJoint)
            { // TODO
                _simSetJointPosition(joint,_mjData->actuator_length[_allJoints[i].mjIdActuator]);
            }
            else
            {
                int padr=_mjModel->jnt_qposadr[_allJoints[i].mjId];
                int vadr=_mjModel->jnt_dofadr[_allJoints[i].mjId];
                if (_allJoints[i].jointType==sim_joint_spherical)
                {
                    C4Vector q(_mjData->qpos[padr+0],_mjData->qpos[padr+1],_mjData->qpos[padr+2],_mjData->qpos[padr+3]);
                    q=_allJoints[i].initialBallQuat2.getInverse()*(q*_allJoints[i].initialBallQuat)*_allJoints[i].initialBallQuat2;
                    // Something is still not right here, in situations where the joint is involved in loop closure, and has an initialBallQuat different from the identity quaternion
                    // But this as only a visual effect on the spherical joint only (involved shapes are correctly placed)
                    q.normalize();
                    _simSetJointSphericalTransformation(joint,q.data,simulationTime);
                }
                else
                {
                    _simSetJointPosition(joint,_mjData->qpos[padr]);
                    int totalPassesCount=0;
                    if (currentPass==totalPasses-1)
                        totalPassesCount=totalPasses;
                    _simAddJointCumulativeForcesOrTorques(joint,-_mjData->actuator_force[_allJoints[i].mjIdActuator],totalPassesCount,simulationTime);
                    _simSetJointVelocity(joint,_mjData->qvel[vadr]);
                }
            }
        }
    }

    // Then force sensors:
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
            double f[3]={-_mjData->sensordata[fadr+0],-_mjData->sensordata[fadr+1],-_mjData->sensordata[fadr+2]};
            double t[3]={-_mjData->sensordata[tadr+0],-_mjData->sensordata[tadr+1],-_mjData->sensordata[tadr+2]};
            _simAddForceSensorCumulativeForcesAndTorques(forceSens,f,t,totalPassesCount,simulationTime);
        }
    }

    // Then shapes:
    for (size_t i=0;i<_allShapes.size();i++)
    { // objectHandle should be ordered so that each object's ancestor comes before
        if (_allShapes[i].itemType==shapeItem)
        { // only shapes that exist in CoppeliaSim
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

                double av[3]={_mjData->cvel[6*bodyId+0],_mjData->cvel[6*bodyId+1],_mjData->cvel[6*bodyId+2]};
                double lv[3]={_mjData->cvel[6*bodyId+3],_mjData->cvel[6*bodyId+4],_mjData->cvel[6*bodyId+5]};
                // Above is COM vel. in spatial vector
                double comPos[3]={_mjData->xipos[3*bodyId+0],_mjData->xipos[3*bodyId+1],_mjData->xipos[3*bodyId+2]};
                // Above com is relative to the origin. We however need the com relative to the last parent body.
                // This is strange and not the "spatial velocity" that I know of:
                int parentId=bodyId;
                while (true)
                {
                    int p=_mjModel->body_parentid[parentId];
                    if (p>0)
                        parentId=p;
                    else
                        break;
                }
                double llv[3];
                if (parentId!=bodyId)
                {
                    comPos[0]-=_mjData->xipos[3*parentId+0];
                    comPos[1]-=_mjData->xipos[3*parentId+1];
                    comPos[2]-=_mjData->xipos[3*parentId+2];
                    double x[3];
                    x[0]=comPos[1]*av[2]-comPos[2]*av[1];
                    x[1]=comPos[2]*av[0]-comPos[0]*av[2];
                    x[2]=comPos[0]*av[1]-comPos[1]*av[0];
                    llv[0]=lv[0]-x[0];
                    llv[1]=lv[1]-x[1];
                    llv[2]=lv[2]-x[2];
                }
                else
                {
                    llv[0]=lv[0];
                    llv[1]=lv[1];
                    llv[2]=lv[2];
                }
                _simSetShapeDynamicVelocity(shape,llv,av,simulationTime);
            }
        }
    }

    // Finally particles:
    _particleCont->updateParticlesPosition(simulationTime);
    */
}

bool CRigidBodyContainerDyn::isDynamicContentAvailable()
{
    return(false);
}

void CRigidBodyContainerDyn::_stepDynamics(double dt,int pass)
{
//    mj_step(_mjModel,_mjData);
}

