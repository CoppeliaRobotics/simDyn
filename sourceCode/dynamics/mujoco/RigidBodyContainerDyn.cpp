#include <RigidBodyContainerDyn.h>
#include <CollShapeDyn.h>
#include <RigidBodyDyn.h>
#include <ConstraintDyn.h>
#include <simLib/simLib.h>
#include <simMath/4X4Matrix.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <regex>
#include <simStack/stackArray.h>
#include <simStack/stackMap.h>

bool CRigidBodyContainerDyn::_simulationHalted = false;
std::vector<SInject> CRigidBodyContainerDyn::_xmlInjections; // old
std::vector<SCompositeInject> CRigidBodyContainerDyn::_xmlCompositeInjections;
std::vector<SFlexcompInject> CRigidBodyContainerDyn::_xmlFlexcompInjections;
std::vector<SGeneralInject> CRigidBodyContainerDyn::_xmlGeneralInjections;
int CRigidBodyContainerDyn::_nextInjectionId = 0;

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine = sim_physics_mujoco;
    _engineVersion = 0;
    _restartCount = 0;
    _restartWarning = false;
    _mjModel = nullptr;
    _mjData = nullptr;
    _mjDataCopy = nullptr;
    _firstCtrlPass = true;
    _simulationHalted = false;
    _objectCreationCounter = -1;
    _objectDestructionCounter = -1;
    _hierarchyChangeCounter = -1;
    _xmlInjectionChanged = false;
    _particleChanged = false;
    _nextCompositeHandle = -3;
    _nextInjectionId = 0;

    _rebuildTrigger = simGetEngineInt32Param(sim_mujoco_global_rebuildtrigger, -1, nullptr, nullptr);
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
    mju_user_warning = nullptr;
    mju_user_error = nullptr;
    mjcb_contactfilter = nullptr;
    mjcb_control = nullptr;
    if (_mjModel != nullptr)
    {
        if (_mjData != nullptr)
            mj_deleteData(_mjData);
        if (_mjDataCopy != nullptr)
            mj_deleteData(_mjDataCopy);
        mj_deleteModel(_mjModel);
    }
    _xmlInjections.clear(); // old
    _xmlCompositeInjections.clear();
    _xmlFlexcompInjections.clear();
    _xmlGeneralInjections.clear();

    for (int i = 0; i < _simGetObjectListSize(sim_handle_all); i++)
    {
        CXSceneObject* it = (CXSceneObject*)_simGetObjectFromIndex(sim_handle_all, i);
        _simSetDynamicSimulationIconCode(it, sim_dynamicsimicon_none);
    }
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20], const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams, intParams);
    return ("");
}

std::string CRigidBodyContainerDyn::_buildMujocoWorld(double timeStep, double simTime, bool rebuild)
{
    mju_user_warning = nullptr;
    mju_user_error = nullptr;
    mjcb_contactfilter = nullptr;
    mjcb_control = nullptr;
    _xmlInjectionChanged = false;
    _particleChanged = false;
    _allGeoms.clear();
    _allJoints.clear();
    _allFreejoints.clear();
    _allForceSensors.clear();
    _allShapes.clear();
    _geomIdIndex.clear();
    mjModel* _mjPrevModel = nullptr;
    mjData* _mjPrevData = nullptr;

    for (size_t i = 0; i < _xmlInjections.size(); i++)
        _xmlInjections[i].xmlDummyString.clear(); // old
    for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
        _xmlCompositeInjections[i].xmlDummyString.clear();
    for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
        _xmlFlexcompInjections[i].xmlDummyString.clear();
    for (size_t i = 0; i < _xmlGeneralInjections.size(); i++)
        _xmlGeneralInjections[i].xmlDummyString.clear();

    _overrideKinematicFlag = simGetEngineInt32Param(sim_mujoco_global_overridekin, -1, nullptr, nullptr);
    char* _dir = simGetStringParam(sim_stringparam_mujocodir);
    std::string mjFile(_dir);
    std::string mjFileExt1(_dir);
    std::string mjFileExt2(_dir);
    std::string dir(_dir);
    simReleaseBuffer(_dir);
    if (rebuild)
    {
        mj_deleteData(_mjDataCopy);
        _mjPrevModel = _mjModel;
        _mjPrevData = _mjData;
        _restartCount++;
    }
    else
    {
        std::filesystem::remove_all(dir.c_str());
        std::filesystem::create_directory(dir.c_str());
    }
    mjFile += "/coppeliaSim.xml";
    mjFileExt1 += "/coppeliaSimExt1.xml";
    mjFileExt2 += "/coppeliaSimExt2.xml";
    CXmlSer* xmlDoc = new CXmlSer(mjFile.c_str());
    _addInjections(xmlDoc, -1, "mujoco"); // old
    xmlDoc->pushNewNode("compiler");
    xmlDoc->setAttr("coordinate", "local");
    xmlDoc->setAttr("angle", "radian");
    xmlDoc->setAttr("usethread", bool(simGetEngineBoolParam(sim_mujoco_global_multithreaded, -1, nullptr, nullptr)));
    xmlDoc->setAttr("balanceinertia", bool(simGetEngineBoolParam(sim_mujoco_global_balanceinertias, -1, nullptr, nullptr)));
    xmlDoc->setAttr("boundmass", simGetEngineFloatParam(sim_mujoco_global_boundmass, -1, nullptr, nullptr));
    xmlDoc->setAttr("boundinertia", simGetEngineFloatParam(sim_mujoco_global_boundinertia, -1, nullptr, nullptr));
    int bv = 0;
    simGetBoolProperty(sim_handle_scene, "mujoco.alignfree", &bv);
    xmlDoc->setAttr("alignfree", bool(bv));
    _addInjections(xmlDoc, -1, "compiler"); // old
    xmlDoc->popNode();

    xmlDoc->pushNewNode("visual");
    _addInjections(xmlDoc, -1, "visual"); // old
    xmlDoc->popNode();

    xmlDoc->pushNewNode("size");
    int mbMemory;
    simGetIntProperty(sim_handle_scene, "mujoco.mbmemory", &mbMemory);
    std::string memor(std::to_string(mbMemory));
    if (mbMemory >= 0)
    {
        memor += "M";
        xmlDoc->setAttr("memory", memor.c_str());
    }
    //xmlDoc->setAttr("njmax",simGetEngineInt32Param(sim_mujoco_global_njmax,-1,nullptr,nullptr));
    //xmlDoc->setAttr("nconmax",simGetEngineInt32Param(sim_mujoco_global_nconmax,-1,nullptr,nullptr));
    //xmlDoc->setAttr("nstack",simGetEngineInt32Param(sim_mujoco_global_nstack,-1,nullptr,nullptr));
    xmlDoc->popNode();

    xmlDoc->pushNewNode("default");
    xmlDoc->pushNewNode("geom");
    xmlDoc->setAttr("rgba", 0.8, 0.6, 0.4, 1.0);
    xmlDoc->popNode();
    _addInjections(xmlDoc, -1, "default"); // old
    xmlDoc->popNode();

    xmlDoc->pushNewNode("option");
    xmlDoc->setAttr("timestep", timeStep);
    xmlDoc->setAttr("impratio", simGetEngineFloatParam(sim_mujoco_global_impratio, -1, nullptr, nullptr));
    double w[5];
    for (size_t i = 0; i < 3; i++)
        w[i] = simGetEngineFloatParam(sim_mujoco_global_wind1 + i, -1, nullptr, nullptr);
    xmlDoc->setAttr("wind", w, 3);
    xmlDoc->setAttr("density", simGetEngineFloatParam(sim_mujoco_global_density, -1, nullptr, nullptr));
    xmlDoc->setAttr("viscosity", simGetEngineFloatParam(sim_mujoco_global_viscosity, -1, nullptr, nullptr));
    xmlDoc->setAttr("o_margin", simGetEngineFloatParam(sim_mujoco_global_overridemargin, -1, nullptr, nullptr));
    for (size_t i = 0; i < 2; i++)
        w[i] = simGetEngineFloatParam(sim_mujoco_global_overridesolref1 + i, -1, nullptr, nullptr);
    xmlDoc->setAttr("o_solref", w, 2);
    for (size_t i = 0; i < 5; i++)
        w[i] = simGetEngineFloatParam(sim_mujoco_global_overridesolimp1 + i, -1, nullptr, nullptr);
    xmlDoc->setAttr("o_solimp", w, 5);
    const char* integrator[] = {"Euler", "RK4", "implicit", "implicitfast"};
    int integratorIndex = simGetEngineInt32Param(sim_mujoco_global_integrator, -1, nullptr, nullptr);
    xmlDoc->setAttr("integrator", integrator[integratorIndex]);
    if (integratorIndex == 1)
        _rg4Cnt = 1; // using Runge-Kutta4
    else
        _rg4Cnt = 0;
    const char* cone[] = {"pyramidal", "elliptic"};
    xmlDoc->setAttr("cone", cone[simGetEngineInt32Param(sim_mujoco_global_cone, -1, nullptr, nullptr)]);
    const char* solver[] = {"PGS", "CG", "Newton"};
    xmlDoc->setAttr("solver", solver[simGetEngineInt32Param(sim_mujoco_global_solver, -1, nullptr, nullptr)]);
    xmlDoc->setAttr("iterations", simGetEngineInt32Param(sim_mujoco_global_iterations, -1, nullptr, nullptr));
    C3Vector gravity;
    _simGetGravity(gravity.data);
    xmlDoc->setAttr("gravity", gravity(0), gravity(1), gravity(2));
    int jacobian = -1;
    const char* jacob[] = {"auto", "dense", "sparse"};
    simGetIntProperty(sim_handle_scene, "mujoco.jacobian", &jacobian);
    xmlDoc->setAttr("jacobian", jacob[jacobian + 1]);
    double tolerance = 1e-8;
    simGetFloatProperty(sim_handle_scene, "mujoco.tolerance", &tolerance);
    xmlDoc->setAttr("tolerance", tolerance);
    int ls_iterations = 50;
    simGetIntProperty(sim_handle_scene, "mujoco.ls_iterations", &ls_iterations);
    xmlDoc->setAttr("ls_iterations", ls_iterations);
    double ls_tolerance = 0.01;
    simGetFloatProperty(sim_handle_scene, "mujoco.ls_tolerance", &ls_tolerance);
    xmlDoc->setAttr("ls_tolerance", ls_tolerance);
    int noslip_iterations = 0;
    simGetIntProperty(sim_handle_scene, "mujoco.noslip_iterations", &noslip_iterations);
    xmlDoc->setAttr("noslip_iterations", noslip_iterations);
    double noslip_tolerance = 1e-6;
    simGetFloatProperty(sim_handle_scene, "mujoco.noslip_tolerance", &noslip_tolerance);
    xmlDoc->setAttr("noslip_tolerance", noslip_tolerance);
    int ccd_iterations = 50;
    simGetIntProperty(sim_handle_scene, "mujoco.ccd_iterations", &ccd_iterations);
    xmlDoc->setAttr("ccd_iterations", ccd_iterations);
    double ccd_tolerance = 1e-6;
    simGetFloatProperty(sim_handle_scene, "mujoco.ccd_tolerance", &ccd_tolerance);
    xmlDoc->setAttr("ccd_tolerance", ccd_tolerance);
    int sdf_iterations = 10;
    simGetIntProperty(sim_handle_scene, "mujoco.sdf_iterations", &sdf_iterations);
    xmlDoc->setAttr("sdf_iterations", sdf_iterations);
    int sdf_initpoints = 40;
    simGetIntProperty(sim_handle_scene, "mujoco.sdf_initpoints", &sdf_initpoints);
    xmlDoc->setAttr("sdf_initpoints", sdf_initpoints);

    xmlDoc->pushNewNode("flag");
    xmlDoc->setAttr("filterparent", "disable");
    xmlDoc->setAttr("fwdinv", "disable");
    const char* disableEnable[] = {"disable", "enable"};
    xmlDoc->setAttr("multiccd", disableEnable[simGetEngineBoolParam(sim_mujoco_global_multiccd, -1, nullptr, nullptr)]);
    xmlDoc->setAttr("override", disableEnable[simGetEngineBoolParam(sim_mujoco_global_overridecontacts, -1, nullptr, nullptr)]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.equalityEnable", &bv);
    xmlDoc->setAttr("equality", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.frictionlossEnable", &bv);
    xmlDoc->setAttr("frictionloss", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.limitEnable", &bv);
    xmlDoc->setAttr("limit", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.contactEnable", &bv);
    xmlDoc->setAttr("contact", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.passiveEnable", &bv);
    xmlDoc->setAttr("passive", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.gravityEnable", &bv);
    xmlDoc->setAttr("gravity", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.warmstartEnable", &bv);
    xmlDoc->setAttr("warmstart", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.actuationEnable", &bv);
    xmlDoc->setAttr("actuation", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.refsafeEnable", &bv);
    xmlDoc->setAttr("refsafe", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.sensorEnable", &bv);
    xmlDoc->setAttr("sensor", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.midphaseEnable", &bv);
    xmlDoc->setAttr("midphase", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.eulerdampEnable", &bv);
    xmlDoc->setAttr("eulerdamp", disableEnable[bv]);
    bv = 1;
    simGetBoolProperty(sim_handle_scene, "mujoco.autoresetEnable", &bv);
    xmlDoc->setAttr("autoreset", disableEnable[bv]);
    bv = 0;
    simGetBoolProperty(sim_handle_scene, "mujoco.energyEnable", &bv);
    xmlDoc->setAttr("energy", disableEnable[bv]);
    bv = 0;
    simGetBoolProperty(sim_handle_scene, "mujoco.invdiscreteEnable", &bv);
    xmlDoc->setAttr("invdiscrete", disableEnable[bv]);
    bv = 0;
    simGetBoolProperty(sim_handle_scene, "mujoco.nativeccdEnable", &bv);
    xmlDoc->setAttr("nativeccd", disableEnable[bv]);
    xmlDoc->popNode();
    xmlDoc->popNode();

    xmlDoc->pushNewNode("worldbody");
    _addInjections(xmlDoc, -1, "worldbody"); // old
    xmlDoc->pushNewNode("light");
    xmlDoc->setAttr("pos", 0.0, 0.0, 2.0);
    xmlDoc->setAttr("dir", 0.0, -1.0, -1.0);
    xmlDoc->setAttr("diffuse", 1, 1, 1);
    xmlDoc->popNode(); // light
    xmlDoc->pushNewNode("light");
    xmlDoc->setAttr("pos", 1.0, 1.0, 2.0);
    xmlDoc->setAttr("dir", -1.0, -1.0, -1.0);
    xmlDoc->setAttr("diffuse", 1, 1, 1);
    xmlDoc->popNode(); // light

    CParticleDyn::xmlDoc = xmlDoc;
    CParticleDyn::allGeoms = &_allGeoms;

    _particleCont->removeKilledParticles();
    _particleCont->addParticlesIfNeeded();

    SInfo info;
    info.folder = dir;
    info.inertiaCalcRobust = false;

    // We need to balance the mass of certain shapes across other shapes, when involved in loop closure with a joint/forceSensor, e.g.:
    // shape1 --> joint --> dummy1 -- dummy2 <-- shape2
    // In Mujoco we can only implement this if we place an aux. body where dummy1 is, and equality constrain it to be 'glued' to shape2
    // In order to not bias the mass/inertia with that aux. body, we split the mass/inertia of shape2 onto all aux. bodies that might
    // be 'glued' to it. This requires 2 exploration passes, where the first pass only adjusts the mass dividers
    std::map<CXSceneObject*, int> massDividers; // shape,divider
    int shapeListSize = _simGetObjectListSize(sim_sceneobject_shape);
    for (int i = 0; i < shapeListSize; i++)
    {
        CXSceneObject* it = (CXSceneObject*)_simGetObjectFromIndex(sim_sceneobject_shape, i);
        info.massDividers[it] = 1;
    }

    for (size_t pass = 0; pass < 2; pass++)
    { // pass0 is to get the mass dividers only. Pass1 is the actual pass
        CXmlSer* ser = nullptr;
        if (pass == 1)
            ser = xmlDoc;
        std::vector<CXSceneObject*> toExplore;
        info.meshFiles.clear();
        info.loopClosures.clear();
        info.staticWelds.clear();
        info.tendons.clear();
        int orphanListSize = _simGetObjectListSize(-1);
        for (int i = 0; i < orphanListSize; i++)
        {
            CXSceneObject* it = (CXSceneObject*)_simGetObjectFromIndex(-1, i);
            toExplore.push_back(it);
        }
        for (size_t i = 0; i < toExplore.size(); i++)
        {
            info.moreToExplore.clear();
            info.isTreeDynamic = false;
            if (_addObjectBranch(toExplore[i], nullptr, ser, &info))
                toExplore.insert(toExplore.end(), info.moreToExplore.begin(), info.moreToExplore.end());
        }

        if (ser != nullptr)
        {
            _addComposites(xmlDoc, -1, "worldbody");
            _addFlexcomps(xmlDoc, -1, "worldbody");
            _addGeneralInjections(xmlDoc, -1); // just to possibly disable invalid injections (because parent shape disappeared)
            xmlDoc->popNode();                 // worldbody

            xmlDoc->pushNewNode("asset");
            _addInjections(xmlDoc, -1, "asset"); // old
            xmlDoc->pushNewNode("texture");
            xmlDoc->setAttr("type", "skybox");
            xmlDoc->setAttr("builtin", "gradient");
            xmlDoc->setAttr("rgb1", 1, 1, 1);
            xmlDoc->setAttr("rgb2", 0.6, 0.8, 1.0);
            xmlDoc->setAttr("width", 256);
            xmlDoc->setAttr("height", 256);
            xmlDoc->popNode(); // texture

            for (size_t i = 0; i < info.meshFiles.size(); i++)
            {
                xmlDoc->pushNewNode("mesh");
                xmlDoc->setAttr("file", info.meshFiles[i].c_str());
                xmlDoc->popNode(); // mesh
            }
            for (size_t i = 0; i < info.heightfieldFiles.size(); i++)
            {
                xmlDoc->pushNewNode("hfield");
                xmlDoc->setAttr("file", info.heightfieldFiles[i].file.c_str());
                xmlDoc->setAttr("size", info.heightfieldFiles[i].size, 4);
                xmlDoc->popNode(); // hfield
            }

            xmlDoc->popNode(); // asset

            xmlDoc->pushNewNode("tendon");
            _addInjections(xmlDoc, -1, "tendon"); // old
            for (size_t i = 0; i < info.tendons.size(); i++)
            {
                CXSceneObject* dummy1 = info.tendons[i];
                int dummy1Handle = _simGetObjectID(dummy1);
                int dummy2Handle = -1;
                _simGetDummyLinkType(dummy1, &dummy2Handle);
                CXSceneObject* dummy2 = (CXSceneObject*)_simGetObject(dummy2Handle);
                if (dummy1Handle < dummy2Handle)
                { // make sure to not have 2 identical tendons!
                    xmlDoc->pushNewNode("spatial");
                    double solrefLimit[2];
                    double range[2];
                    for (size_t j = 0; j < 2; j++)
                    {
                        solrefLimit[j] = simGetEngineFloatParam(sim_mujoco_dummy_solreflimit1 + j, -1, dummy1, nullptr);
                        range[j] = simGetEngineFloatParam(sim_mujoco_dummy_range1 + j, -1, dummy1, nullptr);
                    }
                    double solimpLimit[5];
                    for (size_t j = 0; j < 5; j++)
                    {
                        solimpLimit[j] = simGetEngineFloatParam(sim_mujoco_dummy_solimplimit1 + j, -1, dummy1, nullptr);
                    }
                    double stiffness = simGetEngineFloatParam(sim_mujoco_dummy_stiffness, -1, dummy1, nullptr);
                    double damping = simGetEngineFloatParam(sim_mujoco_dummy_damping, -1, dummy1, nullptr);
                    double springlength = simGetEngineFloatParam(sim_mujoco_dummy_springlength, -1, dummy1, nullptr);
                    double margin = simGetEngineFloatParam(sim_mujoco_dummy_margin, -1, dummy1, nullptr);
                    bool limited = simGetEngineBoolParam(sim_mujoco_dummy_limited, -1, dummy1, nullptr);
                    int proxyJointId = simGetEngineInt32Param(sim_mujoco_dummy_proxyjointid, -1, dummy1, nullptr);
                    CXSceneObject* proxyJoint = nullptr;
                    if (proxyJointId >= 0)
                    {
                        proxyJoint = (CXSceneObject*)_simGetObject(proxyJointId);
                        if (_simGetObjectType(proxyJoint) == sim_sceneobject_joint)
                        {
                            if ((!isJointInDynamicMode(proxyJoint)) || (_simGetJointType(proxyJoint) != sim_joint_prismatic))
                            {
                                char* f = simGetObjectAlias(proxyJointId, 5);
                                std::string nm(f);
                                simReleaseBuffer(f);
                                std::string msg("joint '");
                                msg += nm + "' is referenced from a tendon constraint, but is not in dynamic mode, or is not a prismatic joint. The tendon will not operate as expected.";
                                simAddLog(LIBRARY_NAME, sim_verbosity_warnings, msg.c_str());
                                proxyJoint = nullptr;
                            }
                        }
                        else
                            proxyJoint = nullptr; // should never happen
                    }

                    if (proxyJoint != nullptr)
                    {
                        SMjJoint gjoint;
                        gjoint.dependencyJointHandle = -1;
                        gjoint.objectHandle = proxyJointId;
                        gjoint.name = _getObjectName(proxyJoint);
                        gjoint.jointType = sim_joint_prismatic;
                        gjoint.tendonJoint = true;
                        _allJoints.push_back(gjoint);
                        _simSetDynamicSimulationIconCode(proxyJoint, sim_dynamicsimicon_objectisdynamicallysimulated);
                        _simSetDynamicObjectFlagForVisualization(proxyJoint, 4);
                        xmlDoc->setAttr("name", gjoint.name.c_str()); // name comes from the proxy joint
                        double minp, rangep;
                        bool limited = _simGetJointPositionInterval(proxyJoint, &minp, &rangep);
                        xmlDoc->setAttr("limited", limited);
                        if (limited)
                            xmlDoc->setAttr("range", minp, minp + rangep);
                    }
                    else
                    {
                        xmlDoc->setAttr("name", _getObjectName(dummy1).c_str()); // name comes from the dummy with lowest handle
                        xmlDoc->setAttr("limited", limited);
                        xmlDoc->setAttr("range", range, 2);
                    }
                    xmlDoc->setAttr("solreflimit", solrefLimit, 2);
                    xmlDoc->setAttr("solimplimit", solimpLimit, 5);
                    xmlDoc->setAttr("margin", margin);
                    xmlDoc->setAttr("springlength", springlength);
                    xmlDoc->setAttr("stiffness", stiffness);
                    xmlDoc->setAttr("damping", damping);

                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("site", _getObjectName(dummy1).c_str());
                    xmlDoc->popNode(); // site

                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("site", _getObjectName(dummy2).c_str());
                    xmlDoc->popNode(); // site

                    xmlDoc->popNode(); // spatial
                }
            }
            xmlDoc->popNode(); // tendon

            xmlDoc->pushNewNode("equality");
            _addInjections(xmlDoc, -1, "equality"); // old
            for (size_t i = 0; i < info.loopClosures.size(); i++)
            {
                CXSceneObject* dummy1 = info.loopClosures[i];
                int dummy1Handle = _simGetObjectID(dummy1);
                int dummy2Handle = -1;
                _simGetDummyLinkType(info.loopClosures[i], &dummy2Handle);
                CXSceneObject* dummy2 = (CXSceneObject*)_simGetObject(dummy2Handle);
                CXSceneObject* shape2 = (CXSceneObject*)_simGetParentObject(dummy2);
                CXSceneObject* dummy1Parent = (CXSceneObject*)_simGetParentObject(dummy1);
                double constrOverlap_torqueScale;
                simGetFloatProperty(dummy1Handle, "mujoco.overlapConstrTorquescale", &constrOverlap_torqueScale);
                int sv;
                double* constrOverlap_solref = simGetFloatArrayProperty(dummy1Handle, "mujoco.overlapConstrSolref", &sv);
                double* constrOverlap_solimp = simGetFloatArrayProperty(dummy1Handle, "mujoco.overlapConstrSolimp", &sv);
                if (dummy2Handle != -1)
                {
                    double tmp;
                    simGetFloatProperty(dummy2Handle, "mujoco.overlapConstrTorquescale", &tmp);
                    constrOverlap_torqueScale = 0.5 * (constrOverlap_torqueScale + tmp);
                    double* solrefT = simGetFloatArrayProperty(dummy2Handle, "mujoco.overlapConstrSolref", &sv);
                    double* solimpT = simGetFloatArrayProperty(dummy2Handle, "mujoco.overlapConstrSolimp", &sv);
                    for (size_t j = 0; j < 2; j++)
                        constrOverlap_solref[j] = 0.5 * (constrOverlap_solref[j] + solrefT[j]);
                    for (size_t j = 0; j < 5; j++)
                        constrOverlap_solimp[j] = 0.5 * (constrOverlap_solimp[j] + solimpT[j]);
                    simReleaseBuffer(solrefT);
                    simReleaseBuffer(solimpT);
                }
                
                if (_simGetObjectType(dummy1Parent) == sim_sceneobject_shape)
                { // we have shape --> dummy -- dummy <-- shape
                    if (dummy1Handle < dummy2Handle)
                    { // make sure to not have 2 weld constraints for the same loop closure!
                        xmlDoc->pushNewNode("weld");
                        std::string nm(_getObjectName(dummy1Parent));
                        xmlDoc->setAttr("body1", nm.c_str());
                        nm = _getObjectName(shape2);
                        xmlDoc->setAttr("body2", nm.c_str());
                        xmlDoc->setAttr("torquescale", constrOverlap_torqueScale);
                        xmlDoc->setAttr("solref", constrOverlap_solref, 2);
                        xmlDoc->setAttr("solimp", constrOverlap_solimp, 5);
                        C7Vector tr1, tr2;
                        _simGetObjectCumulativeTransformation(dummy1Parent, tr1.X.data, tr1.Q.data, 1);
                        _simGetObjectCumulativeTransformation(dummy1, tr2.X.data, tr2.Q.data, 1);
                        C7Vector tra(tr1.getInverse() * tr2);
                        _simGetObjectCumulativeTransformation(shape2, tr1.X.data, tr1.Q.data, 1);
                        _simGetObjectCumulativeTransformation(dummy2, tr2.X.data, tr2.Q.data, 1);
                        C7Vector trb(tr2.getInverse() * tr1);
                        C7Vector tr(tra * trb);
                        double v[7] = {tr.X(0), tr.X(1), tr.X(2), tr.Q(0), tr.Q(1), tr.Q(2), tr.Q(3)};
                        xmlDoc->setAttr("relpose", v, 7);
                        xmlDoc->popNode(); // weld
                    }
                }
                else
                { // we have shape --> joint/forceSensor --> dummy -- dummy <-- shape
                    xmlDoc->pushNewNode("weld");
                    std::string nm(_getObjectName(info.loopClosures[i]) + "loop");
                    xmlDoc->setAttr("body1", nm.c_str());
                    xmlDoc->setAttr("body2", _getObjectName(shape2).c_str());
                    xmlDoc->setAttr("torquescale", constrOverlap_torqueScale);
                    xmlDoc->setAttr("solref", constrOverlap_solref, 2);
                    xmlDoc->setAttr("solimp", constrOverlap_solimp, 5);
                    double v[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
                    xmlDoc->setAttr("relpose", v, 7);
                    xmlDoc->popNode(); // weld
                }
                simReleaseBuffer(constrOverlap_solref);
                simReleaseBuffer(constrOverlap_solimp);
            }
            for (size_t i = 0; i < info.staticWelds.size(); i++)
            {
                xmlDoc->pushNewNode("weld");
                std::string nm(_getObjectName(info.staticWelds[i]));
                xmlDoc->setAttr("body1", (nm + "staticCounterpart").c_str());
                xmlDoc->setAttr("body2", nm.c_str());
                double kinematicWeld_torqueScale;
                simGetFloatProperty(sim_handle_scene, "mujoco.kinematicWeldTorquescale", &kinematicWeld_torqueScale);
                xmlDoc->setAttr("torquescale", kinematicWeld_torqueScale);
                int sv;
                double* solrefT = simGetFloatArrayProperty(sim_handle_scene, "mujoco.kinematicWeldSolref", &sv);
                double* solimpT = simGetFloatArrayProperty(sim_handle_scene, "mujoco.kinematicWeldSolimp", &sv);
                xmlDoc->setAttr("solref", solrefT, 2);
                xmlDoc->setAttr("solimp", solimpT, 5);
                simReleaseBuffer(solrefT);
                simReleaseBuffer(solimpT);
                double v[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
                xmlDoc->setAttr("relpose", v, 7);
                xmlDoc->popNode(); // weld
            }
            for (size_t i = 0; i < _allJoints.size(); i++)
            {
                if (_allJoints[i].dependencyJointHandle != -1)
                {
                    CXSceneObject* joint = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
                    CXSceneObject* depJoint = (CXSceneObject*)_simGetObject(_allJoints[i].dependencyJointHandle);
                    xmlDoc->pushNewNode("joint");
                    xmlDoc->setAttr("joint1", _getObjectName(joint).c_str());
                    xmlDoc->setAttr("joint2", _getObjectName(depJoint).c_str());
                    xmlDoc->setAttr("polycoef", _allJoints[i].polycoef, 5);
                    xmlDoc->popNode(); // joint
                }
            }
            xmlDoc->popNode(); // equality

            xmlDoc->pushNewNode("sensor");
            for (size_t i = 0; i < _allForceSensors.size(); i++)
            {
                CXSceneObject* fsensor = (CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
                std::string nm(_getObjectName(fsensor));
                xmlDoc->pushNewNode("force");
                xmlDoc->setAttr("name", (nm + "force").c_str());
                xmlDoc->setAttr("site", nm.c_str());
                xmlDoc->popNode(); // force
                xmlDoc->pushNewNode("torque");
                xmlDoc->setAttr("name", (nm + "torque").c_str());
                xmlDoc->setAttr("site", nm.c_str());
                xmlDoc->popNode(); // torque
            }
            xmlDoc->popNode(); // sensor

            xmlDoc->pushNewNode("actuator");
            _addInjections(xmlDoc, -1, "actuator"); // old
            for (size_t i = 0; i < _allJoints.size(); i++)
            {
                CXSceneObject* joint = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
                const char* customXml = nullptr;
                int m;
                simGetIntProperty(_allJoints[i].objectHandle, "dynCtrlMode", &m);
                simGetObjectInt32Param(_allJoints[i].objectHandle, sim_jointintparam_dynctrlmode, &m);
                if (m == sim_jointdynctrl_free)
                    _allJoints[i].actMode = 0; // not actuated
                else if ((m == sim_jointdynctrl_force) || (m == sim_jointdynctrl_spring))
                    _allJoints[i].actMode = 1; // pure force/torque
                else
                {
                    if (m == sim_jointdynctrl_callback)
                    {
                        SPropertyInfo infos;
                        if (0 < simGetPropertyInfo(_allJoints[i].objectHandle, "customData.mujoco.customActuator", &infos, nullptr))
                        {
                            customXml = simGetStringProperty(_allJoints[i].objectHandle, "customData.mujoco.customActuator");
                            if (customXml != nullptr)
                            {
                                _allJoints[i].actMode = 3; // general Mujoco actuator
                                _allJoints[i].customActuatorXml = customXml;
                                std::string str("__xmlCustomActuatorInject__");
                                str += std::to_string(_allJoints[i].objectHandle);
                                _allJoints[i].customActuatorReplStr = str;
                                simReleaseBuffer(customXml);
                                xmlDoc->pushNewNode(str.c_str());
                                xmlDoc->pushNewNode("dummy");
                                xmlDoc->popNode();
                                xmlDoc->popNode();
                            }
                        }
                    }
                    if (customXml == nullptr)
                        _allJoints[i].actMode = 2; // mixed
                }
                if (customXml == nullptr)
                {
                    xmlDoc->pushNewNode("motor");
                    std::string nm(_getObjectName(joint));
                    xmlDoc->setAttr("name", (nm + "act").c_str());
                    if (_allJoints[i].tendonJoint)
                        xmlDoc->setAttr("tendon", nm.c_str());
                    else
                        xmlDoc->setAttr("joint", nm.c_str());
                    xmlDoc->popNode();             // motor
                }
            }
            for (size_t i = 0; i < _allShapes.size(); i++)
            {
                if ((_allShapes[i].itemType == shapeItem) && _allShapes[i].adhesion)
                {
                    int objectHandle = _allShapes[i].objectHandle;
                    CXSceneObject* shape = (CXSceneObject*)_simGetObject(objectHandle);
                    xmlDoc->pushNewNode("adhesion");
                    std::string nm(_getObjectName(shape));
                    xmlDoc->setAttr("name", (nm + "adh").c_str());
                    xmlDoc->setAttr("body", nm.c_str());
                    double gain;
                    simGetFloatProperty(objectHandle, "mujoco.adhesiongain", &gain);
                    xmlDoc->setAttr("gain", gain);
                    int fl;
                    simGetIntProperty(objectHandle, "mujoco.adhesionforcelimited", &fl);
                    if (fl >= 0)
                        xmlDoc->setAttr("forcelimited", bool(fl));
                    else
                        xmlDoc->setAttr("forcelimited", "auto");
                    int l;
                    double* dv = simGetFloatArrayProperty(objectHandle, "mujoco.adhesionctrlrange", &l);
                    xmlDoc->setAttr("ctrlrange", dv, 2);
                    simReleaseBuffer(dv);
                    dv = simGetFloatArrayProperty(objectHandle, "mujoco.adhesionforcerange", &l);
                    xmlDoc->setAttr("forcerange", dv, 2);
                    simReleaseBuffer(dv);
                    xmlDoc->popNode(); // adhesion
                }
            }
            xmlDoc->popNode(); // actuator

            xmlDoc->popNode(); // mujoco
        }
    }
    if (_xmlInjections.size() > 0)
    { // old
        std::string xml(xmlDoc->getString());
        for (size_t i = 0; i < _xmlInjections.size(); i++)
        {
            std::string s(_xmlInjections[i].xmlDummyString);
            if (s.size() > 0)
            {
                std::size_t p1 = xml.find(s);
                if (p1 != std::string::npos)
                {
                    std::size_t p2 = xml.find(s, p1 + 1);
                    xml.replace(p1 - 1, p2 - p1 + s.size() + 2, _xmlInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }
    if (_xmlCompositeInjections.size() > 0)
    {
        std::string xml(xmlDoc->getString());
        for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
        {
            std::string s(_xmlCompositeInjections[i].xmlDummyString);
            if (s.size() > 0)
            {
                std::size_t p1 = xml.find(s);
                if (p1 != std::string::npos)
                {
                    std::size_t p2 = xml.find(s, p1 + 1);
                    xml.replace(p1 - 1, p2 - p1 + s.size() + 2, _xmlCompositeInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }
    if (_xmlFlexcompInjections.size() > 0)
    {
        std::string xml(xmlDoc->getString());
        for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
        {
            std::string s(_xmlFlexcompInjections[i].xmlDummyString);
            if (s.size() > 0)
            {
                std::size_t p1 = xml.find(s);
                if (p1 != std::string::npos)
                {
                    std::size_t p2 = xml.find(s, p1 + 1);
                    xml.replace(p1 - 1, p2 - p1 + s.size() + 2, _xmlFlexcompInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }
    if (_xmlGeneralInjections.size() > 0)
    {
        std::string xml(xmlDoc->getString());
        for (size_t i = 0; i < _xmlGeneralInjections.size(); i++)
        {
            std::string s(_xmlGeneralInjections[i].xmlDummyString);
            if (s.size() > 0)
            {
                std::size_t p1 = xml.find(s);
                if (p1 != std::string::npos)
                {
                    std::size_t p2 = xml.find(s, p1 + 1);
                    xml.replace(p1 - 1, p2 - p1 + s.size() + 2, _xmlGeneralInjections[i].xml);
                }
            }
        }
        xmlDoc->setString(xml.c_str());
    }

    // handle general actuators (custom control)
    std::string xml(xmlDoc->getString());
    for (size_t i = 0; i < _allJoints.size(); i++)
    {
        if (_allJoints[i].actMode == 3)
        {
            std::size_t p1 = xml.find(_allJoints[i].customActuatorReplStr);
            if (p1 != std::string::npos)
            {
                std::size_t p2 = xml.find(_allJoints[i].customActuatorReplStr, p1 + 1);
                xml.replace(p1 - 1, p2 - p1 + _allJoints[i].customActuatorReplStr.size() + 2, _allJoints[i].customActuatorXml);
            }
        }
    }
    xmlDoc->setString(xml.c_str());

    delete xmlDoc; // saves the file

    _displayWarningAboutCPUCompatibility();

    std::string retVal;
    char error[1000] = "could not load binary model";

    _mjModel = nullptr;
    if (_xmlFlexcompInjections.size() + _xmlGeneralInjections.size() + _xmlCompositeInjections.size() > 0)
    { // we have flexcomps and/or other XML injections. We need to access/modify the expanded XML data:
        mjSpec* spec = mj_parseXML(mjFile.c_str(), nullptr, error, 1000);
        if (spec != nullptr)
        {
            _mjModel = mj_compile(spec, nullptr);
            if (_mjModel != nullptr)
            {
                int r = mj_saveXML(spec, mjFileExt1.c_str(), error, 1000);
                mj_deleteModel(_mjModel);
                _mjModel = nullptr;
#if mjVERSION_HEADER == 325
                if (r != 0) // due to a bug in V3.2.6
#else
                if (r == 0)
#endif
                {
                    std::ifstream extFile1(mjFileExt1);
                    if (extFile1)
                    {
                        std::string data((std::istreambuf_iterator<char>(extFile1)), std::istreambuf_iterator<char>());
                        extFile1.close();

                        _appendSitesToFlexcompBodies(data);
                        _appendGeneralInjections(data);

                        std::ofstream extFile2(mjFileExt2);
                        if (extFile2)
                        {
                            extFile2 << data;
                            extFile2.close();

                            _mjModel = mj_loadXML(mjFileExt2.c_str(), 0, error, 1000);
                        }
                        else
                            strncpy(error, (std::string("could not open ") + mjFileExt2).c_str(), 999);
                    }
                    else
                        strncpy(error, (std::string("could not open ") + mjFileExt1).c_str(), 999);
                }
            }
            else
                strncpy(error, mjs_getError(spec), 999);
            mj_deleteSpec(spec);
        }
    }
    else
        _mjModel = mj_loadXML(mjFile.c_str(), 0, error, 1000);

    if (_mjModel != nullptr)
    {
        _mjData = mj_makeData(_mjModel);
        _mjDataCopy = mj_makeData(_mjModel);

        CParticleDyn::mjModel = _mjModel;
        CParticleDyn::mjData = _mjData;

        _geomIdIndex.resize(_mjModel->ngeom, -1);

        int lastCompositeIndex = -1;
        std::string lastCompositePrefix;
        for (int i = 0; i < int(_allGeoms.size()); i++)
        { // _allGeoms might contain non-existing geoms from composites (e.g. for a m*n*o box we create m*n*o items in _allGeoms, for simplicity purpose)
            int mjId = mj_name2id(_mjModel, mjOBJ_GEOM, _allGeoms[i].name.c_str());
            if (_allGeoms[i].itemType == compositeItem)
            {
                if (_allGeoms[i].prefix != lastCompositePrefix)
                {
                    lastCompositeIndex = getCompositeIndexFromPrefix(_allGeoms[i].prefix.c_str());
                    lastCompositePrefix = _allGeoms[i].prefix;
                }
                _xmlCompositeInjections[lastCompositeIndex].mjIds.push_back(mjId);
            }
            if (mjId >= 0)
            {
                _allGeoms[i].mjId = mjId;
                _geomIdIndex[mjId] = i;
            }
            else
            {
                _allGeoms.erase(_allGeoms.begin() + i);
                i--;
            }
        }

        for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
        {
            SFlexcompInject* comp = &_xmlFlexcompInjections[i];
            comp->parentMjId = -1;
            if (comp->shapeHandle != -1)
            {
                char* f = simGetObjectAlias(comp->shapeHandle, 4);
                comp->parentMjId = mj_name2id(_mjModel, mjOBJ_BODY, f);
                simReleaseBuffer(f);
            }
            comp->bodyIds.clear();
            comp->bodyNames.clear();
            for (size_t j = 0; j < comp->count[0] * comp->count[1] * comp->count[2]; j++)
            {
                std::string n = comp->prefix + "_" + std::to_string(j);
                int mjId = mj_name2id(_mjModel, mjOBJ_BODY, n.c_str());
                comp->bodyIds.push_back(mjId);
                if (mjId == -1)
                    n.clear();
                comp->bodyNames.push_back(n);
            }
        }

        std::map<std::string, bool> sceneJoints;
        for (size_t i = 0; i < _allJoints.size(); i++)
        {
            _allJoints[i].object = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
            if (_allJoints[i].tendonJoint)
                _allJoints[i].mjId = mj_name2id(_mjModel, mjOBJ_TENDON, _allJoints[i].name.c_str());
            else
                _allJoints[i].mjId = mj_name2id(_mjModel, mjOBJ_JOINT, _allJoints[i].name.c_str());
            _allJoints[i].mjIdActuator = mj_name2id(_mjModel, mjOBJ_ACTUATOR, (_allJoints[i].name + "act").c_str());
            sceneJoints[_allJoints[i].name] = true;
        }
        for (size_t i = 0; i < _allFreejoints.size(); i++)
        {
            _allFreejoints[i].object = (CXSceneObject*)_simGetObject(_allFreejoints[i].objectHandle);
            _allFreejoints[i].mjId = mj_name2id(_mjModel, mjOBJ_JOINT, _allFreejoints[i].name.c_str());
            sceneJoints[_allFreejoints[i].name] = true;
        }
        for (size_t i = 0; i < _allForceSensors.size(); i++)
        {
            _allForceSensors[i].object = (CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
            _allForceSensors[i].mjId = mj_name2id(_mjModel, mjOBJ_SENSOR, (_allForceSensors[i].name + "force").c_str());
            _allForceSensors[i].mjId2 = mj_name2id(_mjModel, mjOBJ_SENSOR, (_allForceSensors[i].name + "torque").c_str());
        }
        std::map<int, size_t> tempShapeMap;

        for (size_t i = 0; i < _allShapes.size(); i++)
        {
            tempShapeMap[_allShapes[i].objectHandle] = i;
            _allShapes[i].mjId = mj_name2id(_mjModel, mjOBJ_BODY, _allShapes[i].name.c_str());
            _allShapes[i].mjIdStatic = -1;
            _allShapes[i].mjIdJoint = -1;
            _allShapes[i].mjIdAdhesion = -1;
            if (_allShapes[i].itemType == shapeItem)
            {
                _allShapes[i].object = (CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (_allShapes[i].shapeMode == shapeModes::kinematicMode)
                {
                    _allShapes[i].mjIdStatic = mj_name2id(_mjModel, mjOBJ_BODY, (_allShapes[i].name + "staticCounterpart").c_str());
                    _allShapes[i].mjIdJoint = mj_name2id(_mjModel, mjOBJ_JOINT, (_allShapes[i].name + "freejoint").c_str());
                    /*
                    C3Vector lv,av;
                    _simGetInitialDynamicVelocity(_allShapes[i].object,lv.data);
                    _simGetInitialDynamicAngVelocity(_allShapes[i].object,av.data);
                    if ( (lv.getLength()>0.0)||(av.getLength()>0.0) )
                        simAddLog(LIBRARY_NAME,sim_verbosity_warnings,"detected an initial static shape velocity. This will not work as expected with the MuJoCo engine.");
                        */
                }
                if (_allShapes[i].shapeMode == shapeModes::freeMode)
                { // handle initial velocity for free bodies:
                    _allShapes[i].mjIdJoint = mj_name2id(_mjModel, mjOBJ_JOINT, (_allShapes[i].name + "freejoint").c_str());
                    int nvadr = _mjModel->body_dofadr[_allShapes[i].mjId];
                    C3Vector v;
                    _simGetInitialDynamicVelocity(_allShapes[i].object, v.data);
                    if (v.getLength() > 0.0)
                    {
                        _mjData->qvel[nvadr + 0] = v(0);
                        _mjData->qvel[nvadr + 1] = v(1);
                        _mjData->qvel[nvadr + 2] = v(2);
                        _simSetInitialDynamicVelocity(_allShapes[i].object, C3Vector::zeroVector.data); // important to reset it
                    }
                    _simGetInitialDynamicAngVelocity(_allShapes[i].object, v.data);
                    if (v.getLength() > 0.0)
                    {
                        _mjData->qvel[nvadr + 3] = v(0);
                        _mjData->qvel[nvadr + 4] = v(1);
                        _mjData->qvel[nvadr + 5] = v(2);
                        _simSetInitialDynamicAngVelocity(_allShapes[i].object, C3Vector::zeroVector.data); // important to reset it
                    }
                }
                if (_allShapes[i].adhesion)
                    _allShapes[i].mjIdAdhesion = mj_name2id(_mjModel, mjOBJ_ACTUATOR, (_allShapes[i].name + "adh").c_str());
            }
        }
        for (size_t i = 0; i < _allGeoms.size(); i++)
        {
            if (tempShapeMap.find(_allGeoms[i].objectHandle) != tempShapeMap.end())
            {
                size_t ind = tempShapeMap[_allGeoms[i].objectHandle];
                _allShapes[ind].geomIndices.push_back(i);
            }
        }
        if (rebuild)
        {
            for (int jcnt = 0; jcnt < _mjPrevModel->njnt; jcnt++)
            { // loop through all joints in the old model (except for flexcomps, which are handled further down)
                char* nm = _mjPrevModel->names + _mjPrevModel->name_jntadr[jcnt];
                if (_dynamicallyResetObjects.find(nm) == _dynamicallyResetObjects.end())
                { // that object was not dynamically reset on CoppeliaSim side
                    int mjId = mj_name2id(_mjModel, mjOBJ_JOINT, nm);
                    if (mjId >= 0)
                    { // that joint is also present in the new model
                        int mjPrevId = mj_name2id(_mjPrevModel, mjOBJ_JOINT, nm);
                        if (_mjPrevModel->jnt_type[mjPrevId] == _mjModel->jnt_type[mjId])
                        { // same type (should actually always be the same type)
                            int padr = _mjModel->jnt_qposadr[mjId];
                            int prevPadr = _mjPrevModel->jnt_qposadr[mjPrevId];
                            int vadr = _mjModel->jnt_dofadr[mjId];
                            int prevVadr = _mjPrevModel->jnt_dofadr[mjPrevId];
                            size_t posd = 1;
                            size_t dof = 1;
                            if (_mjPrevModel->jnt_type[mjPrevId] == mjJNT_FREE)
                            {
                                posd = 7;
                                dof = 6;
                            }
                            if (_mjPrevModel->jnt_type[mjPrevId] == mjJNT_BALL)
                            {
                                posd = 4;
                                dof = 3;
                            }
                            if (sceneJoints.find(nm) == sceneJoints.end())
                            { // those are not joints related to CoppeliaSim scene objects. Reuse also previous pos info:
                                for (size_t i = 0; i < posd; i++)
                                    _mjData->qpos[padr + i] = _mjPrevData->qpos[prevPadr + i];
                            }
                            for (size_t i = 0; i < dof; i++)
                                _mjData->qvel[vadr + i] = _mjPrevData->qvel[prevVadr + i];
                            //---- not needed, but who knows... ---
                            for (size_t i = 0; i < dof; i++)
                                _mjData->qacc[vadr + i] = _mjPrevData->qacc[prevVadr + i];
                            for (size_t i = 0; i < dof; i++)
                                _mjData->qacc_warmstart[vadr + i] = _mjPrevData->qacc_warmstart[prevVadr + i];
                            //----------------------------
                        }
                    }
                }
            }

            for (size_t fc = 0; fc < _xmlFlexcompInjections.size(); fc++)
            { // loop through all flexcomp bodies in the old model
                for (size_t bcnt = 0; bcnt < _xmlFlexcompInjections[fc].bodyNames.size(); bcnt++)
                {
                    if (_xmlFlexcompInjections[fc].bodyNames[bcnt] != "")
                    {
                        int mjBodyId = mj_name2id(_mjModel, mjOBJ_BODY, _xmlFlexcompInjections[fc].bodyNames[bcnt].c_str());
                        int mjBodyPrevId = mj_name2id(_mjPrevModel, mjOBJ_BODY, _xmlFlexcompInjections[fc].bodyNames[bcnt].c_str());
                        if ((mjBodyId >= 0) && (mjBodyPrevId >= 0))
                        { // should always pass
                            int jointStartAddr = _mjModel->body_jntadr[mjBodyId];
                            int prevJointStartAddr = _mjPrevModel->body_jntadr[mjBodyPrevId];
                            for (int k = 0; k < _mjModel->body_jntnum[mjBodyId]; k++)
                            {
                                int mjJointId = jointStartAddr + k;
                                int mjJointPrevId = prevJointStartAddr + k;
                                if (_mjPrevModel->jnt_type[mjJointPrevId] == _mjModel->jnt_type[mjJointId])
                                { // same type (should actually always be the same type)
                                    // When a flexcomp had its parent shape erased, then following does not yield the correct position somehow,
                                    // eventhough free joints are expressed in an absolute manner
                                    int padr = _mjModel->jnt_qposadr[mjJointId];
                                    int prevPadr = _mjPrevModel->jnt_qposadr[mjJointPrevId];
                                    int vadr = _mjModel->jnt_dofadr[mjJointId];
                                    int prevVadr = _mjPrevModel->jnt_dofadr[mjJointPrevId];
                                    size_t posd = 1;
                                    size_t dof = 1;
                                    if (_mjModel->jnt_type[mjJointId] == mjJNT_FREE)
                                    {
                                        posd = 7;
                                        dof = 6;
                                    }
                                    if (_mjModel->jnt_type[mjJointId] == mjJNT_BALL)
                                    {
                                        posd = 4;
                                        dof = 3;
                                    }
                                    for (size_t i = 0; i < posd; i++)
                                        _mjData->qpos[padr + i] = _mjPrevData->qpos[prevPadr + i];
                                    for (size_t i = 0; i < dof; i++)
                                        _mjData->qvel[vadr + i] = _mjPrevData->qvel[prevVadr + i];
                                    //---- not needed, but who knows... ---
                                    for (size_t i = 0; i < dof; i++)
                                        _mjData->qacc[vadr + i] = _mjPrevData->qacc[prevVadr + i];
                                    for (size_t i = 0; i < dof; i++)
                                        _mjData->qacc_warmstart[vadr + i] = _mjPrevData->qacc_warmstart[prevVadr + i];
                                    //----------------------------
                                }
                            }
                        }
                    }
                }
            }

            _mjData->time = simTime;
            mj_deleteData(_mjPrevData);
            mj_deleteModel(_mjPrevModel);
            //---- not needed, but who knows... ---
            mj_energyPos(_mjModel, _mjData);
            mj_energyVel(_mjModel, _mjData);
            //----------------------------
            if ((_restartCount > 20) && (_restartCount > int(simTime)) && (!_restartWarning))
            {
                simAddLog(LIBRARY_NAME, sim_verbosity_warnings, "detected frequent dynamic world rebuilds. The MuJoCo engine is not optimized for frequent changes in the world, and the simulation might not be running in an optimal fashion.");
                _restartWarning = true;
            }
        }
        mjcb_contactfilter = _contactCallback;
        mjcb_control = _controlCallback;
        mju_user_error = _errorCallback;
        mju_user_warning = _warningCallback;
    }
    else
    {
        if (rebuild)
        {
            mj_deleteData(_mjPrevData);
            mj_deleteModel(_mjPrevModel);
        }
        _simulationHalted = true;
        retVal = error;
    }
    return retVal;
}

void CRigidBodyContainerDyn::_appendSitesToFlexcompBodies(std::string& data) const
{
    for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
    {
        std::string nm(_xmlFlexcompInjections[i].prefix);
        std::string rgex("<body\\s+name=\\\"");
        rgex += nm;
        rgex += "([^\\\"]+)\\\"([^>\\n\\r]*?)>";
        std::regex bodyRegex(rgex);

        std::string result;
        size_t lastPos = 0;
        std::smatch match;

        std::string::const_iterator searchStart(data.cbegin());
        while (std::regex_search(searchStart, data.cend(), match, bodyRegex))
        {
            size_t matchStart = match.position(0) + std::distance(data.cbegin(), searchStart);
            result += data.substr(lastPos, matchStart - lastPos);

            std::string bodyName = nm + match[1].str();
            std::string other = match[2].str();

            result += "<body name=\"" + bodyName + "\"" + other +
                      "><site name=\"" + bodyName + "\"/>";

            lastPos = matchStart + match.length(0);
            searchStart = match.suffix().first;
        }

        result += data.substr(lastPos);
        data = result;
    }

    for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
    {
        std::string nm(_xmlCompositeInjections[i].prefix);
        std::string rgex("<body\\s+name=\\\"");
        rgex += nm;
        rgex += "([^\\\"]+)\\\"([^>\\n\\r]*?)>";
        std::regex bodyRegex(rgex);

        std::string result;
        size_t lastPos = 0;
        std::smatch match;

        std::string::const_iterator searchStart(data.cbegin());
        while (std::regex_search(searchStart, data.cend(), match, bodyRegex))
        {
            size_t matchStart = match.position(0) + std::distance(data.cbegin(), searchStart);
            result += data.substr(lastPos, matchStart - lastPos);

            std::string bodyName = nm + match[1].str();
            std::string other = match[2].str();

            result += "<body name=\"" + bodyName + "\"" + other +
                      "><site name=\"" + bodyName + "\"/>";

            lastPos = matchStart + match.length(0);
            searchStart = match.suffix().first;
        }

        result += data.substr(lastPos);
        data = result;
    }
}

void CRigidBodyContainerDyn::_appendGeneralInjections(std::string& data) const
{
    for (size_t inj = 0; inj < _xmlGeneralInjections.size(); inj++)
    {
        SGeneralInject* comp = &_xmlGeneralInjections[inj];
        if ((comp->element != "") || (comp->shapeHandle != -1))
        {
            comp->updateCnt++;
            if ((comp->cbFunc.size() > 0) && (comp->cbScript != -1) && (comp->updateCnt > 1))
            { // possibly get updated injection data via callback:
                int stack = simCreateStack();
                CStackArray outArguments;
                outArguments.pushInt(comp->injectionId);
                outArguments.buildOntoStack(stack);
                if (simCallScriptFunctionEx(comp->cbScript, comp->cbFunc.c_str(), stack) != -1)
                {
                    // simDebugStack(stack, -1);
                    CStackArray inArguments;
                    inArguments.buildFromStack(stack);
                    addOrUpdateGeneralInjection(&inArguments, comp->injectionId, nullptr, -1);
                }
                simReleaseStack(stack);
            }

            std::string rgext("</");
            rgext += comp->element;
            rgext += ">";
            size_t p = data.find(rgext);
            if (p == std::string::npos)
            {
                p = data.find("</mujoco>");
                std::string rgext3("<");
                rgext3 += comp->element;
                rgext3 += "></";
                rgext3 += comp->element;
                rgext3 += ">";
                data.insert(p, rgext3);
                p = data.find(rgext);
            }
            data.insert(p, comp->xml);
        }
    }
}

void CRigidBodyContainerDyn::particlesAdded()
{
    _particleChanged = true;
}

std::string CRigidBodyContainerDyn::getInfo(const char* queryString) const
{
    std::string retVal;
    if (std::string(queryString) == "bodies")
    {
        if (_mjModel != nullptr)
        {
            for (int i = 0; i < _mjModel->nbody; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_BODY, i);
                retVal += "body ";
                retVal += n;
                retVal += ":\n    position:     {";
                const double* pos = _mjData->xpos + 3 * i;
                for (int j = 0; j < 3; j++)
                {
                    retVal += std::to_string(pos[j]);
                    if (j != 2)
                        retVal += ", ";
                }
                retVal += "}\n    quaternion:   {";
                const double* quat = _mjData->xquat + 4 * i;
                for (int j = 0; j < 4; j++)
                {
                    retVal += std::to_string(quat[j]);
                    if (j != 3)
                        retVal += ", ";
                }
                retVal += "}:\n    linear vel.:    {";
                const double* lvel = _mjData->cvel + 6 * i;
                for (int j = 0; j < 3; j++)
                {
                    retVal += std::to_string(lvel[j]);
                    if (j != 2)
                        retVal += ", ";
                }
                retVal += "}:\n    angular vel.:   {";
                const double* avel = _mjData->cvel + 6 * i + 3;
                for (int j = 0; j < 3; j++)
                {
                    retVal += std::to_string(avel[j]);
                    if (j != 2)
                        retVal += ", ";
                }
                retVal += "}\n";
            }
        }
    }
    if (std::string(queryString) == "nameAndIds")
    {
        if (_mjModel != nullptr)
        {
            for (int i = 0; i < _mjModel->nbody; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_BODY, i);
                retVal += "body ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->njnt; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_JOINT, i);
                retVal += "joint ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->ngeom; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_GEOM, i);
                retVal += "geom ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nmesh; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_MESH, i);
                retVal += "mesh ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nhfield; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_HFIELD, i);
                retVal += "heightfield ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nsite; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_SITE, i);
                retVal += "site ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nflex; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_FLEX, i);
                retVal += "flex ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->neq; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_EQUALITY, i);
                retVal += "equality ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->ntendon; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_TENDON, i);
                retVal += "tendon ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nu; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_ACTUATOR, i);
                retVal += "actuator ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
            for (int i = 0; i < _mjModel->nsensor; i++)
            {
                const char* n = mj_id2name(_mjModel, mjOBJ_SENSOR, i);
                retVal += "sensor ";
                retVal += std::to_string(i);
                retVal += " ";
                if (n != nullptr)
                    retVal += n;
                else
                    retVal += "<null>";
                retVal += "\n";
            }
        }
    }
    return (retVal);
}

std::string CRigidBodyContainerDyn::getCompositeInfo(int compIndex, int what, std::vector<double>& info, int count[3]) const
{
    std::string retVal;
    if (_mjData != nullptr)
    {
        if (compIndex != -1)
        {
            SCompositeInject* composite = &_xmlCompositeInjections[compIndex];
            count[0] = composite->count[0];
            count[1] = composite->count[1];
            count[2] = composite->count[2];
            retVal = composite->type;

            if (what == 0)
            { // positions
                for (size_t i = 0; i < composite->mjIds.size(); i++)
                {
                    int id = composite->mjIds[i];
                    if (id >= 0)
                    {
                        int bodyId = _mjModel->geom_bodyid[id];
                        info.push_back(_mjData->xpos[3 * bodyId + 0]);
                        info.push_back(_mjData->xpos[3 * bodyId + 1]);
                        info.push_back(_mjData->xpos[3 * bodyId + 2]);
                    }
                }
            }
            if (what == 1)
            { // poses
                for (size_t i = 0; i < composite->mjIds.size(); i++)
                {
                    int id = composite->mjIds[i];
                    if (id >= 0)
                    {
                        int bodyId = _mjModel->geom_bodyid[id];
                        info.push_back(_mjData->xpos[3 * bodyId + 0]);
                        info.push_back(_mjData->xpos[3 * bodyId + 1]);
                        info.push_back(_mjData->xpos[3 * bodyId + 2]);
                        info.push_back(_mjData->xquat[4 * bodyId + 1]);
                        info.push_back(_mjData->xquat[4 * bodyId + 2]);
                        info.push_back(_mjData->xquat[4 * bodyId + 3]);
                        info.push_back(_mjData->xquat[4 * bodyId + 0]);
                    }
                }
            }
            if ((what == 2) || (what == 3))
            { // triangles
                std::vector<double> pts;
                for (size_t i = 0; i < composite->mjIds.size(); i++)
                {
                    int id = composite->mjIds[i];
                    if (id >= 0)
                    {
                        int bodyId = _mjModel->geom_bodyid[id];
                        pts.push_back(_mjData->xpos[3 * bodyId + 0]);
                        pts.push_back(_mjData->xpos[3 * bodyId + 1]);
                        pts.push_back(_mjData->xpos[3 * bodyId + 2]);
                    }
                    else
                    { // inside pt
                        pts.push_back(0.0);
                        pts.push_back(0.0);
                        pts.push_back(0.0);
                    }
                }

                // Box, cylinder, ellipsoid and grids:
                size_t centerGeom = 0;
                if ((composite->type == "box") || (composite->type == "cylinder") || (composite->type == "ellipsoid"))
                    centerGeom = 1;
                else
                    what = 2; // only boxes, cylinders and ellipsoides can be grown since they have a center
                for (size_t fb = 0; fb < 2; fb++)
                {
                    // +- x faces:
                    size_t off = centerGeom * 3 + 3 * (count[0] - 1) * count[1] * count[2] * fb;
                    for (size_t y = 0; y < count[1] - 1; y++)
                    {
                        for (size_t z = 0; z < count[2] - 1; z++)
                        {
                            double p[4][3];
                            for (size_t i = 0; i < 3; i++)
                            {
                                p[0][i] = pts[off + 3 * (y * count[2] + z) + i];
                                p[1 + 2 * fb][i] = pts[off + 3 * (y * count[2] + z + 1) + i];
                                p[2][i] = pts[off + 3 * ((y + 1) * count[2] + z + 1) + i];
                                p[3 - 2 * fb][i] = pts[off + 3 * ((y + 1) * count[2] + z) + i];
                            }
                            if ((composite->grow != 0.0) && (what == 3))
                            {
                                C3Vector center(pts[0], pts[1], pts[2]);
                                for (size_t i = 0; i < 4; i++)
                                {
                                    C3Vector v(p[i][0], p[i][1], p[i][2]);
                                    C3Vector w(v - center);
                                    w.normalize();
                                    v = v + w * composite->grow;
                                    p[i][0] = v(0);
                                    p[i][1] = v(1);
                                    p[i][2] = v(2);
                                }
                            }
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                        }
                    }

                    // +- y faces:
                    off = centerGeom * 3 + (3 * count[2] * (count[1] - 1)) * fb;
                    size_t soff = count[2] * count[1];
                    for (size_t x = 0; x < count[0] - 1; x++)
                    {
                        for (size_t z = 0; z < count[2] - 1; z++)
                        {
                            double p[4][3];
                            for (size_t i = 0; i < 3; i++)
                            {
                                p[0][i] = pts[off + 3 * (x * soff + z) + i];
                                p[1 + 2 * fb][i] = pts[off + 3 * ((x + 1) * soff + z) + i];
                                p[2][i] = pts[off + 3 * ((x + 1) * soff + z + 1) + i];
                                p[3 - 2 * fb][i] = pts[off + 3 * (x * soff + z + 1) + i];
                            }
                            if ((composite->grow != 0.0) && (what == 3))
                            {
                                C3Vector center(pts[0], pts[1], pts[2]);
                                for (size_t i = 0; i < 4; i++)
                                {
                                    C3Vector v(p[i][0], p[i][1], p[i][2]);
                                    C3Vector w(v - center);
                                    w.normalize();
                                    v = v + w * composite->grow;
                                    p[i][0] = v(0);
                                    p[i][1] = v(1);
                                    p[i][2] = v(2);
                                }
                            }
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                        }
                    }
                    // +- z faces:
                    off = centerGeom * 3 + 3 * (count[2] - 1) * fb;
                    soff = count[2] * count[1];
                    for (size_t x = 0; x < count[0] - 1; x++)
                    {
                        for (size_t y = 0; y < count[1] - 1; y++)
                        {
                            double p[4][3];
                            for (size_t i = 0; i < 3; i++)
                            {
                                p[0][i] = pts[off + 3 * (x * soff + y * count[2]) + i];
                                p[1 + 2 * fb][i] = pts[off + 3 * (x * soff + (y + 1) * count[2]) + i];
                                p[2][i] = pts[off + 3 * ((x + 1) * soff + (y + 1) * count[2]) + i];
                                p[3 - 2 * fb][i] = pts[off + 3 * ((x + 1) * soff + y * count[2]) + i];
                            }
                            if ((composite->grow != 0.0) && (what == 3))
                            {
                                C3Vector center(pts[0], pts[1], pts[2]);
                                for (size_t i = 0; i < 4; i++)
                                {
                                    C3Vector v(p[i][0], p[i][1], p[i][2]);
                                    C3Vector w(v - center);
                                    w.normalize();
                                    v = v + w * composite->grow;
                                    p[i][0] = v(0);
                                    p[i][1] = v(1);
                                    p[i][2] = v(2);
                                }
                            }
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2][i]);
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3][i]);
                        }
                    }
                }
            }
        }
    }
    return (retVal);
}

std::string CRigidBodyContainerDyn::getFlexcompInfo(int flexcompIndex, int what, std::vector<double>& info, int count[3]) const
{
    std::string retVal;
    if (_mjData != nullptr)
    {
        if (flexcompIndex != -1)
        {
            SFlexcompInject* flexcomp = &_xmlFlexcompInjections[flexcompIndex];
            count[0] = int(flexcomp->count[0]);
            count[1] = int(flexcomp->count[1]);
            count[2] = int(flexcomp->count[2]);
            retVal = flexcomp->type;
            C7Vector parentTr(C7Vector::identityTransformation);
            if (flexcomp->parentMjId != -1)
            {
                parentTr.X = C3Vector(_mjData->xpos[3 * flexcomp->parentMjId + 0], _mjData->xpos[3 * flexcomp->parentMjId + 1], _mjData->xpos[3 * flexcomp->parentMjId + 2]);
                parentTr.Q = C4Vector(_mjData->xquat[4 * flexcomp->parentMjId + 0], _mjData->xquat[4 * flexcomp->parentMjId + 1], _mjData->xquat[4 * flexcomp->parentMjId + 2], _mjData->xquat[4 * flexcomp->parentMjId + 3]);
            }
            std::vector<C3Vector> pts;
            if ((what == 0) || (what == 1))
            { // positions
                for (size_t i = 0; i < flexcomp->bodyIds.size(); i++)
                {
                    int id = flexcomp->bodyIds[i];
                    C3Vector v;
                    if (id >= 0)
                        v.setData(_mjData->xpos[3 * id + 0], _mjData->xpos[3 * id + 1], _mjData->xpos[3 * id + 2]);
                    else
                        v = parentTr * flexcomp->initialPositions[i];
                    pts.push_back(v);
                    if (what == 0)
                    {
                        info.push_back(v(0));
                        info.push_back(v(1));
                        info.push_back(v(2));
                    }
                }
            }
            if (what == 1)
            { // triangles
                for (size_t fb = 0; fb < 2; fb++)
                {
                    // +- x faces:
                    double ffb = -1.0;
                    if (fb > 0)
                        ffb = 1.0;
                    size_t off = (count[0] - 1) * count[1] * count[2] * fb;
                    for (int y = 0; y < count[1] - 1; y++)
                    {
                        for (int z = 0; z < count[2] - 1; z++)
                        {
                            C3Vector p[4];
                            p[0] = pts[off + (y * count[2] + z)];
                            p[1 + 2 * fb] = pts[off + (y * count[2] + z + 1)];
                            p[2] = pts[off + ((y + 1) * count[2] + z + 1)];
                            p[3 - 2 * fb] = pts[off + ((y + 1) * count[2] + z)];
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                        }
                    }

                    // +- y faces:
                    ffb = -1.0;
                    if (fb > 0)
                        ffb = 1.0;
                    off = (count[2] * (count[1] - 1)) * fb;
                    size_t soff = count[2] * count[1];
                    for (int x = 0; x < count[0] - 1; x++)
                    {
                        for (int z = 0; z < count[2] - 1; z++)
                        {
                            C3Vector p[4];
                            p[0] = pts[off + (x * soff + z)];
                            p[1 + 2 * fb] = pts[off + ((x + 1) * soff + z)];
                            p[2] = pts[off + ((x + 1) * soff + z + 1)];
                            p[3 - 2 * fb] = pts[off + (x * soff + z + 1)];
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                        }
                    }

                    // +- z faces:
                    ffb = -1.0;
                    if (fb > 0)
                        ffb = 1.0;
                    off = (count[2] - 1) * fb;
                    soff = count[2] * count[1];
                    for (int x = 0; x < count[0] - 1; x++)
                    {
                        for (int y = 0; y < count[1] - 1; y++)
                        {
                            C3Vector p[4];
                            p[0] = pts[off + (x * soff + y * count[2])];
                            p[1 + 2 * fb] = pts[off + (x * soff + (y + 1) * count[2])];
                            p[2] = pts[off + ((x + 1) * soff + (y + 1) * count[2])];
                            p[3 - 2 * fb] = pts[off + ((x + 1) * soff + y * count[2])];
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[0](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[1](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[2](i));
                            for (size_t i = 0; i < 3; i++)
                                info.push_back(p[3](i));
                        }
                    }
                }
            }
        }
    }
    return retVal;
}

void CRigidBodyContainerDyn::_addInjections(CXmlSer* xmlDoc, int objectHandle, const char* currentElement)
{ // old
    for (size_t inj = 0; inj < _xmlInjections.size(); inj++)
    {
        SInject* injection = &_xmlInjections[inj];
        if (injection->xmlDummyString.size() == 0)
        {
            std::string ds;
            if (objectHandle == -1)
            {
                if (injection->element == currentElement)
                    ds = std::string("__xmlInject__") + std::to_string(inj);
            }
            else
            {
                if (injection->objectHandle == objectHandle)
                    ds = std::string("__xmlObjInject__") + std::to_string(objectHandle);
            }
            if (ds.size() > 0)
            {
                if (injection->cbFunc.size() > 0)
                { // possibly get updated composite data via callback:
                    int stack = simCreateStack();
                    CStackArray outArguments;
                    outArguments.pushString(injection->xml);
                    CStackMap* info = new CStackMap();
                    info->setString("cbId", injection->cbId);
                    outArguments.pushMap(info);
                    outArguments.buildOntoStack(stack);
                    if (simCallScriptFunctionEx(injection->cbScript, injection->cbFunc.c_str(), stack) != -1)
                    {
                        CStackArray inArguments;
                        inArguments.buildFromStack(stack);
                        if (inArguments.getSize() >= 1)
                        {
                            if (inArguments.isString(0))
                                injection->xml = inArguments.getString(0);
                        }
                    }
                    simReleaseStack(stack);
                }

                xmlDoc->pushNewNode(ds.c_str());
                xmlDoc->pushNewNode("dummy");
                xmlDoc->popNode();
                xmlDoc->popNode();
                injection->xmlDummyString = ds;
            }
        }
    }
}

void CRigidBodyContainerDyn::_addComposites(CXmlSer* xmlDoc, int shapeHandle, const char* currentElement)
{
    for (size_t inj = 0; inj < _xmlCompositeInjections.size(); inj++)
    {
        SCompositeInject* comp = &_xmlCompositeInjections[inj];
        if (comp->xmlDummyString.size() == 0)
        {
            std::string ds;
            if (shapeHandle != -1)
            {
                if (comp->shapeHandle == shapeHandle)
                    ds = std::string("__xmlCompShapeInject__") + std::to_string(shapeHandle) + "_" + std::to_string(inj);
            }
            else
            {
                if (comp->element == currentElement)
                    ds = std::string("__xmlCompInject__") + std::to_string(inj);
            }
            if (ds.size() > 0)
            {
                if (comp->cbFunc.size() > 0)
                { // possibly get updated composite data via callback:
                    int stack = simCreateStack();
                    CStackArray outArguments;
                    outArguments.pushString(comp->xml);
                    CStackMap* info = new CStackMap();
                    info->setString("prefix", comp->prefix);
                    info->setInt("respondableMask", comp->respondableMask);
                    info->setDouble("grow", comp->grow);
                    outArguments.pushMap(info);
                    outArguments.buildOntoStack(stack);
                    if (simCallScriptFunctionEx(comp->cbScript, comp->cbFunc.c_str(), stack) != -1)
                    {
                        CStackArray inArguments;
                        inArguments.buildFromStack(stack);
                        if (inArguments.getSize() >= 1)
                        {
                            if (inArguments.isString(0))
                                comp->xml = inArguments.getString(0);
                            if (inArguments.getSize() >= 1)
                            {
                                if (inArguments.isMap(1))
                                {
                                    CStackMap* map = inArguments.getMap(1);
                                    if (map->isNumber("respondableMask"))
                                        comp->respondableMask = map->getInt("respondableMask");
                                    if (map->isNumber("grow"))
                                        comp->grow = map->getDouble("grow");
                                }
                            }
                        }
                    }
                    simReleaseStack(stack);
                }

                if ((comp->type == "box") || (comp->type == "cylinder") || (comp->type == "ellipsoid"))
                {
                    SMjGeom g;
                    g.belongsToStaticItem = false;
                    g.name = comp->prefix + "Gcenter";
                    g.objectHandle = _nextCompositeHandle;
                    g.prefix = comp->prefix;
                    g.itemType = compositeItem;
                    g.respondableMask = 0;
                    _allGeoms.push_back(g);

                    for (size_t i = 0; i < comp->count[0]; i++)
                    {
                        for (size_t j = 0; j < comp->count[1]; j++)
                        {
                            for (size_t k = 0; k < comp->count[2]; k++)
                            {
                                SMjGeom g;
                                g.belongsToStaticItem = false;
                                g.name = comp->prefix + "G" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k);
                                g.objectHandle = _nextCompositeHandle;
                                g.prefix = comp->prefix;
                                g.itemType = compositeItem;
                                g.respondableMask = comp->respondableMask;
                                _allGeoms.push_back(g);
                            }
                        }
                    }
                }
                if ((comp->type == "grid") || (comp->type == "cloth"))
                {
                    for (size_t i = 0; i < comp->count[0]; i++)
                    {
                        for (size_t j = 0; j < comp->count[1]; j++)
                        {
                            SMjGeom g;
                            g.belongsToStaticItem = false;
                            g.name = comp->prefix + "G" + std::to_string(i) + "_" + std::to_string(j);
                            g.objectHandle = _nextCompositeHandle;
                            g.prefix = comp->prefix;
                            g.itemType = compositeItem;
                            g.respondableMask = comp->respondableMask;
                            _allGeoms.push_back(g);
                        }
                    }
                }
                if ((comp->type == "cable") || (comp->type == "loop"))
                {
                    for (size_t i = 0; i < comp->count[0]; i++)
                    {
                        SMjGeom g;
                        g.belongsToStaticItem = false;
                        g.name = comp->prefix + "G" + std::to_string(i);
                        g.objectHandle = _nextCompositeHandle;
                        g.prefix = comp->prefix;
                        g.itemType = compositeItem;
                        g.respondableMask = comp->respondableMask;
                        _allGeoms.push_back(g);
                    }
                }
                _nextCompositeHandle--;
                xmlDoc->pushNewNode(ds.c_str());
                xmlDoc->pushNewNode("dummy");
                xmlDoc->popNode();
                xmlDoc->popNode();
                comp->xmlDummyString = ds;
            }
        }
    }
}

void CRigidBodyContainerDyn::_addFlexcomps(CXmlSer* xmlDoc, int shapeHandle, const char* currentElement)
{
    for (size_t inj = 0; inj < _xmlFlexcompInjections.size(); inj++)
    {
        SFlexcompInject* comp = &_xmlFlexcompInjections[inj];
        bool ignorePins = false;
        if (shapeHandle == -1)
        { // this function is called from the worldbody element. Fix situation where a parent shape of a flexcomp was removed (in a world rebuild)
            if (comp->shapeHandle != -1)
            {
                ignorePins = true;
                for (size_t i = 0; i < _allShapes.size(); i++)
                {
                    if (_allShapes[i].objectHandle == comp->shapeHandle)
                    {
                        ignorePins = false;
                        break;
                    }
                }
                if (ignorePins)
                {
                    comp->shapeHandle = -1;
                    comp->element = "worldbody";
                }
            }
        }

        if (comp->xmlDummyString.size() == 0)
        {
            std::string ds;
            if (shapeHandle != -1)
            {
                if (comp->shapeHandle == shapeHandle)
                    ds = std::string("__xmlFlexcompShapeInject__") + std::to_string(shapeHandle) + std::to_string(inj);
            }
            else
            {
                if (comp->element == currentElement)
                    ds = std::string("__xmlFlexcompInject__") + std::to_string(inj);
            }
            if (ds.size() > 0)
            {
                comp->updateCnt++;
                if ((comp->cbFunc.size() > 0) && (comp->cbScript != -1) && (comp->updateCnt > 1))
                { // possibly get updated flexcomp data via callback:
                    int stack = simCreateStack();
                    CStackArray outArguments;
                    outArguments.pushInt(comp->injectionId);
                    outArguments.buildOntoStack(stack);
                    if (simCallScriptFunctionEx(comp->cbScript, comp->cbFunc.c_str(), stack) != -1)
                    {
                        // simDebugStack(stack, -1);
                        CStackArray inArguments;
                        inArguments.buildFromStack(stack);
                        addOrUpdateFlexcompInjection(&inArguments, comp->injectionId, nullptr, -1);
                    }
                    simReleaseStack(stack);
                }

                int dim = 0;
                comp->xml = "<flexcomp name=\"" + comp->prefix + "\" type=\"" + comp->type + "\" count=\"";
                for (size_t i = 0; i < 3; i++)
                {
                    comp->xml += std::to_string(comp->count[i]) + " ";
                    if (comp->count[i] > 1)
                        dim++;
                }
                comp->xml += "\" spacing=\"";
                for (size_t i = 0; i < 3; i++)
                    comp->xml += std::to_string(comp->spacing[i]) + " ";
                comp->xml += "\" dim=\"" + std::to_string(dim) + "\" radius=\"" + std::to_string(comp->radius) + "\" mass=\"";
                comp->xml += std::to_string(comp->mass) + "\" pos=\"";
                for (size_t i = 0; i < 3; i++)
                    comp->xml += std::to_string(comp->pose.X(i)) + " ";
                comp->xml += "\" quat=\"";
                for (size_t i = 0; i < 4; i++)
                    comp->xml += std::to_string(comp->pose.Q(i)) + " ";
                comp->xml += "\" ";
                comp->xml += comp->flexcompExtraXml + ">\n";
                comp->xml += comp->extraXml + "\n";
                if ((comp->pinned.size() > 0) && (!ignorePins))
                {
                    comp->xml += "<pin id=\"";
                    for (size_t i = 0; i < comp->pinned.size(); i++)
                        comp->xml += std::to_string(comp->pinned[i]) + " ";
                    comp->xml += "\"/>\n";
                }
                comp->xml += "</flexcomp>\n";

                if (comp->type == "grid")
                {
                    C3Vector off(-0.5 * comp->spacing[0] * double(comp->count[0] - 1), -0.5 * comp->spacing[1] * double(comp->count[1] - 1), -0.5 * comp->spacing[2] * double(comp->count[2] - 1));
                    for (size_t x = 0; x < comp->count[0]; x++)
                    {
                        for (size_t y = 0; y < comp->count[1]; y++)
                        {
                            for (size_t z = 0; z < comp->count[2]; z++)
                            {
                                C3Vector ip(comp->pose * (off + C3Vector(double(x) * comp->spacing[0], double(y) * comp->spacing[1], double(z) * comp->spacing[2])));
                                comp->initialPositions.push_back(ip);
                            }
                        }
                    }
                }
                xmlDoc->pushNewNode(ds.c_str());
                xmlDoc->pushNewNode("dummy");
                xmlDoc->popNode();
                xmlDoc->popNode();
                comp->xmlDummyString = ds;
            }
        }
    }
}

void CRigidBodyContainerDyn::_addGeneralInjections(CXmlSer* xmlDoc, int shapeHandle)
{
    for (size_t inj = 0; inj < _xmlGeneralInjections.size(); inj++)
    {
        SGeneralInject* comp = &_xmlGeneralInjections[inj];
        if (shapeHandle == -1)
        { // this function is called from the worldbody element. Fix situation where a parent shape was removed (in a world rebuild)
            if (comp->shapeHandle != -1)
            {
                bool present = false;
                for (size_t i = 0; i < _allShapes.size(); i++)
                {
                    if (_allShapes[i].objectHandle == comp->shapeHandle)
                    {
                        present = true;
                        break;
                    }
                }
                if (!present)
                {
                    comp->shapeHandle = -1;
                    comp->element = ""; // disable that injection
                }
            }
        }

        if (comp->xmlDummyString.size() == 0)
        {
            std::string ds;
            if (shapeHandle != -1)
            {
                if (comp->shapeHandle == shapeHandle)
                    ds = std::string("__xmlGeneralShapeInject__") + std::to_string(shapeHandle) + std::to_string(inj);
            }
            if (ds.size() > 0)
            {
                comp->updateCnt++;
                if ((comp->cbFunc.size() > 0) && (comp->cbScript != -1) && (comp->updateCnt > 1))
                { // possibly get updated injection data via callback:
                    int stack = simCreateStack();
                    CStackArray outArguments;
                    outArguments.pushInt(comp->injectionId);
                    outArguments.buildOntoStack(stack);
                    if (simCallScriptFunctionEx(comp->cbScript, comp->cbFunc.c_str(), stack) != -1)
                    {
                        // simDebugStack(stack, -1);
                        CStackArray inArguments;
                        inArguments.buildFromStack(stack);
                        addOrUpdateGeneralInjection(&inArguments, comp->injectionId, nullptr, -1);
                    }
                    simReleaseStack(stack);
                }

                xmlDoc->pushNewNode(ds.c_str());
                xmlDoc->pushNewNode("dummy");
                xmlDoc->popNode();
                xmlDoc->popNode();
                comp->xmlDummyString = ds;
            }
        }
    }
}

void CRigidBodyContainerDyn::_displayWarningAboutCPUCompatibility()
{
    static bool done = false;
    if (!done)
        simAddLog(LIBRARY_NAME, sim_verbosity_warnings | sim_verbosity_onlyterminal, "make sure your CPU supports AVX, SSE and similar, otherwise you might see a crash now...");
    done = true;
}

std::string CRigidBodyContainerDyn::_getObjectName(CXSceneObject* object)
{
    int objectHandle = _simGetObjectID(object);
    char* f = simGetObjectAlias(objectHandle, 4);
    std::string retVal(f);
    simReleaseBuffer(f);
    return retVal;
}

bool CRigidBodyContainerDyn::_addObjectBranch(CXSceneObject* object, CXSceneObject* parent, CXmlSer* xmlDoc, SInfo* info)
{
    size_t moreToExploreInitSize = info->moreToExplore.size();
    int dynProp = _simGetTreeDynamicProperty(object);
    if (dynProp & (sim_objdynprop_dynamic | sim_objdynprop_respondable))
    { // elements in tree could be dynamic and/or respondable
        int objType = _simGetObjectType(object);
        bool objectWasAdded = false;
        bool ignoreChildren = false;
        if (objType == sim_sceneobject_shape)
        {
            bool isStatic = ((dynProp & sim_objdynprop_dynamic) == 0) || (_simIsShapeDynamicallyStatic(object) != 0);
            if (isStatic)
            {
                if ((_simIsShapeDynamicallyRespondable(object) != 0) && info->isTreeDynamic)
                    _simMakeDynamicAnnouncement(sim_announce_containsstaticshapesondynamicconstruction);
            }
            if (isStatic && (parent != nullptr))
            { // static shapes should always be added to "worldbody"!
                info->moreToExplore.push_back(object);
                ignoreChildren = true;
            }
            else
            {
                int parentType = -1;
                if (parent != nullptr)
                    parentType = _simGetObjectType(parent);
                if (parentType == sim_sceneobject_shape)
                { // parent(shape) --> this(shape): those have to be added to "worldbody"
                    info->moreToExplore.push_back(object);
                    ignoreChildren = true;
                }
                else
                {
                    bool isNonRespondable = ((dynProp & sim_objdynprop_respondable) == 0) || (_simIsShapeDynamicallyRespondable(object) == 0);
                    bool addIt = (!isNonRespondable) || (!isStatic);
                    if (!addIt)
                    { // check if we want to add this static, non-respondable shape
                        int childrenCount;
                        CXSceneObject** childrenPointer = (CXSceneObject**)_simGetObjectChildren(object, &childrenCount);
                        for (int i = 0; i < childrenCount; i++)
                        {
                            CXSceneObject* child = childrenPointer[i];
                            if (_simGetObjectType(child) == sim_sceneobject_forcesensor)
                            {
                                addIt = true;
                                break;
                            }
                            if ((_simGetObjectType(child) == sim_sceneobject_joint) && (isJointInDynamicMode(child) && ((_simGetTreeDynamicProperty(child) & sim_objdynprop_dynamic) != 0)))
                            {
                                addIt = true;
                                break;
                            }
                        }
                    }
                    if (addIt)
                    {
                        _addShape(object, parent, xmlDoc, info);
                        objectWasAdded = true;
                    }
                }
            }
        }
        CXSceneObject* child = nullptr;
        if (objType == sim_sceneobject_joint)
        {
            /*
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
*/
            int t;
            CXSceneObject* cld = getJointOrFsensorChild(object, &t);
            if ((parent != nullptr) && ((_simGetObjectType(parent) == sim_sceneobject_shape) || (_simGetObjectType(parent) == sim_sceneobject_joint)) && (cld != nullptr) && (isJointInDynamicMode(object) || _simIsJointInHybridOperation(object)) && (dynProp & sim_objdynprop_dynamic))
            { // parent is shape (or joint, consecutive joints are allowed!), joint is in correct mode, and has one possible correct child
                if ((t == sim_sceneobject_shape) || (t == sim_sceneobject_joint))
                { // child is a dyn. shape (or a dyn. joint)
                    _addObjectBranch(cld, object, xmlDoc, info);
                    ignoreChildren = true;
                }
                else
                    child = cld; // check further down if that joint is involved in a loop closure
            }
        }
        if (objType == sim_sceneobject_forcesensor)
        {
            /*
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
*/
            int t;
            CXSceneObject* cld = getJointOrFsensorChild(object, &t);
            if ((parent != nullptr) && (_simGetObjectType(parent) == sim_sceneobject_shape) && (cld != nullptr) && (dynProp & sim_objdynprop_dynamic))
            { // parent is shape, and force sensor has an appropriate child
                if (t == sim_sceneobject_shape)
                { // child is a dyn. shape
                    _addObjectBranch(cld, object, xmlDoc, info);
                    ignoreChildren = true;
                }
                else
                    child = cld; // check further down if that force sensor is involved in a loop closure
            }
        }
        if (objType == sim_sceneobject_dummy)
        {
            int childrenCount;
            _simGetObjectChildren(object, &childrenCount);
            if ((parent != nullptr) && (_simGetObjectType(parent) == sim_sceneobject_shape) && (childrenCount == 0) && (dynProp & sim_objdynprop_dynamic))
            { // parent is shape, and dummy has no child
                int linkedDummyHandle = -1;
                int linkType = _simGetDummyLinkType(object, &linkedDummyHandle);
                if (linkedDummyHandle != -1)
                { // there is a linked dummy
                    CXDummy* linkedDummy = (CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent = (CXSceneObject*)_simGetParentObject(linkedDummy);
                    _simGetObjectChildren(linkedDummy, &childrenCount);
                    if ((linkedDummyParent != nullptr) && (_simGetObjectType(linkedDummyParent) == sim_sceneobject_shape) && (childrenCount == 0))
                    { // linked dummy has a shape parent, and no children on its own
                        if (_simIsShapeDynamicallyRespondable(linkedDummyParent) || (_simIsShapeDynamicallyStatic(linkedDummyParent) == 0))
                        { // the linked dummy's parent is dyn. or respondable
                            if (_simGetTreeDynamicProperty(linkedDummyParent) & (sim_objdynprop_dynamic | sim_objdynprop_respondable))
                            {
                                if (linkType == sim_dummylink_dynloopclosure)
                                    info->loopClosures.push_back(object); // this means a shape --> dummy -- dummy <-- shape loopClosure
                                if (linkType == sim_dummylink_dyntendon)
                                    info->tendons.push_back(object); // this means a shape --> dummy -- dummy <-- shape tendon
                                if ((xmlDoc != nullptr) && ((linkType == sim_dummylink_dynloopclosure) || (linkType == sim_dummylink_dyntendon)))
                                {
                                    _simSetDynamicSimulationIconCode(object, sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(object, 64);
                                    _simSetDynamicSimulationIconCode(linkedDummy, sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(linkedDummy, 64);
                                }
                            }
                        }
                    }
                }
                // for a dummy that has a shape parent, we always add a site, which can be used for
                // various purposes (some built-in like tendons, others set-up by users via script code)
                if (xmlDoc != nullptr)
                {
                    xmlDoc->pushNewNode("site");
                    xmlDoc->setAttr("name", _getObjectName(object).c_str());
                    //-------------------
                    double pos[3];
                    double quat[4];
                    C7Vector tr;
                    C7Vector pTrInv, objTr;
                    _simGetObjectCumulativeTransformation(parent, pTrInv.X.data, pTrInv.Q.data, true);
                    pTrInv.inverse();
                    _simGetObjectCumulativeTransformation(object, objTr.X.data, objTr.Q.data, true);
                    tr = pTrInv * objTr;
                    tr.X.getData(pos);
                    xmlDoc->setPosAttr("pos", pos);
                    tr.Q.getData(quat);
                    xmlDoc->setQuatAttr("quat", quat);
                    //-------------------
                    xmlDoc->popNode();
                }
            }
        }
        if (child != nullptr)
        { // joint or force sensor possibly involved in a loop closure
            if (_simGetObjectType(child) == sim_sceneobject_dummy)
            { // child is a dummy
                int linkedDummyHandle = -1;
                int linkType = _simGetDummyLinkType(child, &linkedDummyHandle);
                if ((linkType == sim_dummylink_dynloopclosure) && (linkedDummyHandle != -1))
                { // the dummy is linked to another dummy via a dyn. overlap constr.
                    CXDummy* linkedDummy = (CXDummy*)_simGetObject(linkedDummyHandle);
                    CXSceneObject* linkedDummyParent = (CXSceneObject*)_simGetParentObject(linkedDummy);
                    if ((linkedDummyParent != nullptr) && (_simGetObjectType(linkedDummyParent) == sim_sceneobject_shape))
                    { // the linked dummy's parent is a shape
                        if (_simIsShapeDynamicallyRespondable(linkedDummyParent) || (_simIsShapeDynamicallyStatic(linkedDummyParent) == 0))
                        { // the linked dummy's parent is dyn. or respondable
                            if (_simGetTreeDynamicProperty(linkedDummyParent) & (sim_objdynprop_dynamic | sim_objdynprop_respondable))
                            {
                                _addShape(child, object, xmlDoc, info);
                                objectWasAdded = true;
                                ignoreChildren = true;
                                if (xmlDoc != nullptr)
                                {
                                    _simSetDynamicSimulationIconCode(child, sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(child, 64);
                                    _simSetDynamicSimulationIconCode(linkedDummy, sim_dynamicsimicon_objectisdynamicallysimulated);
                                    _simSetDynamicObjectFlagForVisualization(linkedDummy, 64);
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
            CXSceneObject** childrenPointer = (CXSceneObject**)_simGetObjectChildren(object, &childrenCount);
            for (int i = 0; i < childrenCount; i++)
            {
                if (objectWasAdded)
                    _addObjectBranch(childrenPointer[i], object, xmlDoc, info);
                else
                    info->moreToExplore.push_back(childrenPointer[i]);
            }
        }
        if (objectWasAdded && (xmlDoc != nullptr))
            xmlDoc->popNode();
    }
    return (moreToExploreInitSize != info->moreToExplore.size());
}

void CRigidBodyContainerDyn::_addShape(CXSceneObject* object, CXSceneObject* parent, CXmlSer* xmlDoc, SInfo* info)
{ // object can also be a loop closure dummy! (as in shape1 --> joint/fsensor --> DUMMY1 -- dummy2 <-- shape2).
    // In that case we need to add a dummy body inside of shape1, which contains the joint/fsensor
    // That dummy body needs to be the same as shape2, and the mass needs to be balanced across all dummy bodies,
    // check out info->massDividers
    int objectHandle = _simGetObjectID(object);
    int dynProp = _simGetTreeDynamicProperty(object);
    bool forceStatic = ((dynProp & sim_objdynprop_dynamic) == 0);
    bool forceNonRespondable = ((dynProp & sim_objdynprop_respondable) == 0);
    int parentType = -1;
    if (parent != nullptr)
        parentType = _simGetObjectType(parent);
    int flag = 0;
    std::string objectName(_getObjectName(object));

    SMjShape g;
    g.objectHandle = objectHandle;
    g.name = objectName;
    if (parent == nullptr)
        g.shapeMode = shapeModes::freeMode; // might also be staticMode or kinematicMode. Decided further down
    else
        g.shapeMode = shapeModes::attachedMode;

    if (_simGetObjectType(object) == sim_sceneobject_dummy)
    { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
        g.adhesion = false;
        g.itemType = dummyShapeItem;
        if (xmlDoc == nullptr)
        { // We already know that dummy2 has a shape as parent, and the link type is overlap constr.
            int linkedDummyHandle = -1;
            _simGetDummyLinkType(object, &linkedDummyHandle);
            CXDummy* linkedDummy = (CXDummy*)_simGetObject(linkedDummyHandle);
            CXSceneObject* linkedDummyParent = (CXSceneObject*)_simGetParentObject(linkedDummy);
            info->massDividers[linkedDummyParent] = info->massDividers[linkedDummyParent] + 1;
        }
    }
    else
    {
        int bp = 0;
        simGetBoolProperty(objectHandle, "mujoco.adhesion", &bp);
        g.adhesion = bool(bp);
        g.itemType = shapeItem;
        if (parent == nullptr)
        {
            if (forceStatic || (_simIsShapeDynamicallyStatic(object) != 0))
            { // we have a static shape.
                int kin;
                simGetObjectInt32Param(objectHandle, sim_shapeintparam_kinematic, &kin);
                if (_overrideKinematicFlag == 1)
                    kin = 0;
                if (_overrideKinematicFlag == 2)
                    kin = 1;

                CXGeomWrap* geomInfo = (CXGeomWrap*)_simGetGeomWrapFromGeomProxy(object);
                if (_simGetPurePrimitiveType(geomInfo) == sim_primitiveshape_heightfield)
                    kin = 0;
                if (kin == 0)
                    g.shapeMode = shapeModes::staticMode;
                else
                { // The object is static AND kinematic (specific to Mujoco plugin): we need to add 2 bodies welded together: mocap and non-mocap
                    g.shapeMode = shapeModes::kinematicMode;
                    if (xmlDoc != nullptr)
                    {
                        info->staticWelds.push_back(object);
                        xmlDoc->pushNewNode("body");
                        xmlDoc->setAttr("name", (objectName + "staticCounterpart").c_str());
                        // -------------------
                        double pos[3];
                        double quat[4];
                        _simGetObjectCumulativeTransformation(object, pos, quat, 1);
                        xmlDoc->setPosAttr("pos", pos);
                        xmlDoc->setQuatAttr("quat", quat);
                        // -------------------
                        xmlDoc->setAttr("mocap", true);
                        xmlDoc->popNode();
                    }
                }
            }
        }
        if (g.shapeMode >= shapeModes::freeMode)
        {
            flag = flag | 2; // free or attached shapes
            info->isTreeDynamic = true;
        }
        if ((!forceNonRespondable) && _simIsShapeDynamicallyRespondable(object))
            flag = flag | 1;
    }

    if (xmlDoc != nullptr)
        xmlDoc->pushNewNode("body");
    if (_simGetObjectType(object) == sim_sceneobject_dummy)
    { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
        forceNonRespondable = true;
        info->loopClosures.push_back(object);
        if (xmlDoc != nullptr)
        {
            xmlDoc->setAttr("name", (objectName + "loop").c_str());
            xmlDoc->setAttr("mocap", forceStatic);
        }
    }
    else
    { // we have a shape
        if (xmlDoc != nullptr)
        {
            _addInjections(xmlDoc, objectHandle, "");
            _addComposites(xmlDoc, objectHandle, "");
            _addFlexcomps(xmlDoc, objectHandle, "");
            _addGeneralInjections(xmlDoc, objectHandle);
            xmlDoc->setAttr("name", objectName.c_str());
            double gravcomp = 0.0;
            simGetFloatProperty(objectHandle, "mujoco.gravcomp", &gravcomp);
            xmlDoc->setAttr("gravcomp", gravcomp);
            _allShapes.push_back(g);
            _simSetDynamicSimulationIconCode(object, sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(object, flag);
        }
    }

    CXSceneObject* containingShape = parent; //i.e. shape1 as in shape1 --> joint/fsensor --> shape2/dummy
    while (containingShape != nullptr)
    {
        if (_simGetObjectType(containingShape) == sim_sceneobject_shape)
            break;
        containingShape = (CXSceneObject*)_simGetParentObject(containingShape);
    }

    C7Vector objectPose; // pose of current shape (can also be a dummy shape!!)
    if (xmlDoc != nullptr)
    {
        if (_simGetObjectType(object) == sim_sceneobject_dummy)
        { // we have dummy -- linkedDummy <- shape.
            // the dummy shape we create here is very special
            C7Vector tr;
            _simGetObjectCumulativeTransformation(object, tr.X.data, tr.Q.data, 1);
            int linkedDummyHandle = -1;
            _simGetDummyLinkType(object, &linkedDummyHandle);
            CXDummy* linkedDummy = (CXDummy*)_simGetObject(linkedDummyHandle);
            C7Vector linkedDummyLocal;
            _simGetObjectLocalTransformation(linkedDummy, linkedDummyLocal.X.data, linkedDummyLocal.Q.data, 1);
            objectPose = tr * linkedDummyLocal.getInverse();
            // objectPose is the abs. pose for the dummy shape (that doesn't exist in CoppeliaSim)
        }
        else
            _simGetObjectCumulativeTransformation(object, objectPose.X.data, objectPose.Q.data, 1);

        // ------------------
        double pos[3];
        double quat[4];
        C7Vector tr;
        if (parent == nullptr)
            tr = objectPose;
        else
        {
            C7Vector pTr;
            _simGetObjectCumulativeTransformation(containingShape, pTr.X.data, pTr.Q.data, true);
            tr = pTr.getInverse() * objectPose;
        }
        tr.X.getData(pos);
        xmlDoc->setPosAttr("pos", pos);
        tr.Q.getData(quat);
        xmlDoc->setQuatAttr("quat", quat);
        // ------------------

        if (parent == nullptr)
        { // static or free shapes.
            if (g.shapeMode == shapeModes::staticMode)
                xmlDoc->setAttr("mocap", true);
            else
            {
                xmlDoc->pushNewNode("freejoint");
                xmlDoc->setAttr("name", (objectName + "freejoint").c_str());
                xmlDoc->popNode();
                SMjFreejoint fj;
                fj.objectHandle = objectHandle;
                fj.name = objectName + "freejoint";
                _allFreejoints.push_back(fj);
            }
        }
    }

    if (xmlDoc != nullptr)
    {
        if (parentType == sim_sceneobject_joint)
        { // dyn. shape attached to joint (and consecutive joints are allowed!)
            std::vector<CXSceneObject*> parentJoints;
            CXSceneObject* jointIterator = parent;
            while (_simGetObjectType(jointIterator) == sim_sceneobject_joint)
            {
                parentJoints.push_back(jointIterator);
                jointIterator = (CXSceneObject*)_simGetParentObject(jointIterator);
                if (jointIterator == nullptr) // should not happen
                    break;
            }
            for (int i = int(parentJoints.size()) - 1; i >= 0; i--)
            {
                CXSceneObject* joint = parentJoints[i];
                std::string jointName(_getObjectName(joint));
                int jt = _simGetJointType(joint);

                xmlDoc->pushNewNode("joint");
                double solrefLimit[2];
                double solrefFriction[2];
                double springdamper[2];
                for (size_t j = 0; j < 2; j++)
                {
                    solrefLimit[j] = simGetEngineFloatParam(sim_mujoco_joint_solreflimit1 + j, -1, joint, nullptr);
                    solrefFriction[j] = simGetEngineFloatParam(sim_mujoco_joint_solreffriction1 + j, -1, joint, nullptr);
                    springdamper[j] = simGetEngineFloatParam(sim_mujoco_joint_springdamper1 + j, -1, joint, nullptr);
                }
                double solimpLimit[5];
                double solimpFriction[5];
                for (size_t j = 0; j < 5; j++)
                {
                    solimpLimit[j] = simGetEngineFloatParam(sim_mujoco_joint_solimplimit1 + j, -1, joint, nullptr);
                    solimpFriction[j] = simGetEngineFloatParam(sim_mujoco_joint_solimpfriction1 + j, -1, joint, nullptr);
                }
                double stiffness = simGetEngineFloatParam(sim_mujoco_joint_stiffness, -1, joint, nullptr);
                double damping = simGetEngineFloatParam(sim_mujoco_joint_damping, -1, joint, nullptr);
                double springref = simGetEngineFloatParam(sim_mujoco_joint_springref, -1, joint, nullptr);
                double armature = simGetEngineFloatParam(sim_mujoco_joint_armature, -1, joint, nullptr);
                double margin = simGetEngineFloatParam(sim_mujoco_joint_margin, -1, joint, nullptr);
                double frictionLoss = simGetEngineFloatParam(sim_mujoco_joint_frictionloss, -1, joint, nullptr);
                xmlDoc->setAttr("name", jointName.c_str());
                xmlDoc->setAttr("solreflimit", solrefLimit, 2);
                xmlDoc->setAttr("solimplimit", solimpLimit, 5);
                xmlDoc->setAttr("frictionloss", frictionLoss);
                xmlDoc->setAttr("solreffriction", solrefFriction, 2);
                xmlDoc->setAttr("solimpfriction", solimpFriction, 5);
                xmlDoc->setAttr("stiffness", stiffness);
                xmlDoc->setAttr("damping", damping);
                xmlDoc->setAttr("springref", springref);
                xmlDoc->setAttr("armature", armature);
                xmlDoc->setAttr("margin", margin);

                SMjJoint gjoint;
                if (jt == sim_joint_spherical)
                {
                    xmlDoc->setAttr("type", "ball");
                    double p[7];
                    simGetObjectChildPose(_simGetObjectID(joint), p);
                    gjoint.initialBallQuat = C4Vector(p + 3, true);
                    double p2[7];
                    simGetObjectPose(_simGetObjectID(joint), _simGetObjectID(containingShape), p2);
                    gjoint.initialBallQuat2 = C4Vector(p2 + 3, true);
                    gjoint.dependencyJointHandle = -1;
                }
                else
                {
                    if (jt == sim_joint_revolute)
                        xmlDoc->setAttr("type", "hinge");
                    if (jt == sim_joint_prismatic)
                        xmlDoc->setAttr("type", "slide");
                    double minp, rangep;
                    bool limited = _simGetJointPositionInterval(joint, &minp, &rangep);
                    xmlDoc->setAttr("limited", limited);
                    if (limited)
                        xmlDoc->setAttr("range", minp, minp + rangep);
                    xmlDoc->setAttr("ref", _simGetJointPosition(joint));
                    double off, mult;
                    simGetJointDependency(_simGetObjectID(joint), &gjoint.dependencyJointHandle, &off, &mult);
                    //gjoint.dependencyJointHandle=simGetEngineInt32Param(sim_mujoco_joint_dependentobjectid,-1,joint,nullptr);
                    if (gjoint.dependencyJointHandle != -1)
                    {
                        gjoint.polycoef[0] = off;
                        gjoint.polycoef[1] = mult;
                        for (size_t j = 0; j < 3; j++)
                            gjoint.polycoef[2 + j] = simGetEngineFloatParam(sim_mujoco_joint_polycoef3 + j, -1, joint, nullptr);
                    }
                }

                // ---------------------
                double pos[3];
                C7Vector tr;
                C7Vector jointTr;
                _simGetObjectCumulativeTransformation(joint, jointTr.X.data, jointTr.Q.data, true);
                tr = objectPose.getInverse() * jointTr;
                tr.X.getData(pos);
                xmlDoc->setPosAttr("pos", pos);
                C4X4Matrix m(tr);
                xmlDoc->setPosAttr("axis", m.M.axis[2].data);
                // ---------------------

                xmlDoc->popNode();

                gjoint.objectHandle = _simGetObjectID(joint);
                gjoint.name = jointName;
                gjoint.jointType = jt;
                gjoint.tendonJoint = false;
                _allJoints.push_back(gjoint);

                _simSetDynamicSimulationIconCode(joint, sim_dynamicsimicon_objectisdynamicallysimulated);
                _simSetDynamicObjectFlagForVisualization(joint, 4);
            }
        }
        if (parentType == sim_sceneobject_forcesensor)
        { // dyn. shape attached to force sensor
            CXSceneObject* forceSensor = parent;

            std::string forceSensorName(_getObjectName(forceSensor));

            SMjForceSensor gfsensor;
            gfsensor.objectHandle = _simGetObjectID(forceSensor);
            gfsensor.name = forceSensorName;
            _allForceSensors.push_back(gfsensor);

            xmlDoc->pushNewNode("site");
            xmlDoc->setAttr("name", forceSensorName.c_str());

            // -----------------
            double pos[3];
            double quat[4];
            C7Vector tr;
            C7Vector sensorTr;
            _simGetObjectCumulativeTransformation(forceSensor, sensorTr.X.data, sensorTr.Q.data, true);
            tr = objectPose.getInverse() * sensorTr;
            tr.X.getData(pos);
            xmlDoc->setPosAttr("pos", pos);
            tr.Q.getData(quat);
            xmlDoc->setQuatAttr("quat", quat);
            // -----------------

            xmlDoc->popNode();

            _simSetDynamicSimulationIconCode(forceSensor, sim_dynamicsimicon_objectisdynamicallysimulated);
            _simSetDynamicObjectFlagForVisualization(forceSensor, 32);
        }
    }

    if (xmlDoc != nullptr)
    {
        if (_simGetObjectType(object) == sim_sceneobject_dummy)
        { // loop closure of type: shape1 --> joint/fsensor --> dummy1 -- dummy2 <-- shape2
            // We already know that dummy2 has a shape as parent, and the link type is overlap constr.
            // Keep in mind that dummy1 and dummy2 might not (yet) be overlapping!
            int linkedDummyHandle = -1;
            _simGetDummyLinkType(object, &linkedDummyHandle);
            CXDummy* linkedDummy = (CXDummy*)_simGetObject(linkedDummyHandle);
            CXSceneObject* linkedDummyParent = (CXSceneObject*)_simGetParentObject(linkedDummy);

            C7Vector tr;
            C3Vector im;
            double mass = _simGetLocalInertiaInfo(linkedDummyParent, tr.X.data, tr.Q.data, im.data); // im includes the mass!
            mass /= double(info->massDividers[linkedDummyParent]);
            im /= double(info->massDividers[linkedDummyParent]);
            _addInertiaElement(xmlDoc, mass, tr, im);
        }
        else
        {
            if (!_addMeshes(object, xmlDoc, info, &_allGeoms, forceStatic || (_simIsShapeDynamicallyStatic(object) != 0)))
                _simMakeDynamicAnnouncement(sim_announce_containsnonpurenonconvexshapes);
            if (g.shapeMode != shapeModes::staticMode)
            {
                C7Vector tr;
                C3Vector im;
                double mass = _simGetLocalInertiaInfo(object, tr.X.data, tr.Q.data, im.data); // im includes the mass!
                mass /= double(info->massDividers[object]);                                   // the mass is possibly shared with a loop closure of type shape1 --> joint/fsensor --> dummy1(becomes aux. body) -- dummy2 <-- shape2
                im /= double(info->massDividers[object]);
                if (g.shapeMode == shapeModes::kinematicMode)
                {
                    mass = simGetEngineFloatParam(sim_mujoco_global_kinmass, -1, nullptr, nullptr);
                    double inertia = simGetEngineFloatParam(sim_mujoco_global_kininertia, -1, nullptr, nullptr);
                    im = C3Vector(inertia, inertia, inertia);
                }
                _addInertiaElement(xmlDoc, mass, tr, im);
            }
            xmlDoc->pushNewNode("site");
            xmlDoc->setAttr("name", _getObjectName(object).c_str());
            xmlDoc->popNode(); // site
        }
    }
}

void CRigidBodyContainerDyn::_addInertiaElement(CXmlSer* xmlDoc, double mass, const C7Vector& tr, const C3Vector diagI)
{
    xmlDoc->pushNewNode("inertial");
    xmlDoc->setAttr("mass", mass);
    xmlDoc->setPosAttr("pos", tr.X.data);
    xmlDoc->setQuatAttr("quat", tr.Q.data);
    xmlDoc->setAttr("diaginertia", diagI(0), diagI(1), diagI(2));
    xmlDoc->popNode();
}

double CRigidBodyContainerDyn::computePMI(const double* vertices, int verticesL, const int* indices, int indicesL, C7Vector& tr, C3Vector& diagI)
{ // returns the diagonal mass-less inertia, for a density of 1000. Errors are silent here
    mju_user_warning = nullptr;
    mju_user_error = nullptr;

    double mass = 0.0;
    char* _dir = simGetStringParam(sim_stringparam_tempdir);
    std::string dir(_dir);
    std::string mjFile(_dir);
    simReleaseBuffer(_dir);
    mjFile += "/inertiaCalc.xml";
    CXmlSer* xmlDoc = new CXmlSer(mjFile.c_str());
    xmlDoc->pushNewNode("worldbody");

    xmlDoc->pushNewNode("body");
    xmlDoc->setAttr("name", "inertia");
    xmlDoc->setAttr("pos", 0.0, 0.0, 0.0);
    xmlDoc->setAttr("quat", 1.0, 0.0, 0.0, 0.0);
    xmlDoc->pushNewNode("freejoint");
    xmlDoc->popNode(); // freejoint

    xmlDoc->pushNewNode("geom");
    //xmlDoc->setAttr("name","inertia");
    xmlDoc->setAttr("type", "mesh");
    const char* tmpFileName = simGetStringParam(sim_stringparam_uniqueid);
    std::string fn(tmpFileName);
    simReleaseBuffer(tmpFileName);
    xmlDoc->setAttr("mesh", fn.c_str());
    fn += ".stl";
    std::string meshFile(fn);
    fn = dir + "/" + fn;
    simExportMesh(4, fn.c_str(), 0, 1.0, 1, (const double**)&vertices, &verticesL, (const int**)&indices, &indicesL, nullptr, nullptr);
    xmlDoc->popNode(); // geom

    xmlDoc->popNode(); // body
    xmlDoc->popNode(); // worldbody

    xmlDoc->pushNewNode("asset");
    xmlDoc->pushNewNode("mesh");
    xmlDoc->setAttr("file", meshFile.c_str());
    xmlDoc->popNode(); // mesh
    xmlDoc->popNode(); // asset
    xmlDoc->popNode(); // file

    delete xmlDoc; // saves the file

    _displayWarningAboutCPUCompatibility();

    char error[1000] = "could not load binary model";
    mjModel* m = mj_loadXML(mjFile.c_str(), 0, error, 1000);
    if (m != nullptr)
    {
        mjData* d = mj_makeData(m);
        int id = mj_name2id(m, mjOBJ_BODY, "inertia");
        mass = m->body_mass[id];
        diagI(0) = m->body_inertia[3 * id + 0];
        diagI(1) = m->body_inertia[3 * id + 1];
        diagI(2) = m->body_inertia[3 * id + 2];
        diagI /= mass;
        tr.X(0) = m->body_ipos[3 * id + 0];
        tr.X(1) = m->body_ipos[3 * id + 1];
        tr.X(2) = m->body_ipos[3 * id + 2];
        tr.Q(0) = m->body_iquat[4 * id + 0];
        tr.Q(1) = m->body_iquat[4 * id + 1];
        tr.Q(2) = m->body_iquat[4 * id + 2];
        tr.Q(3) = m->body_iquat[4 * id + 3];
        mj_deleteData(d);
        mj_deleteModel(m);
    }
    else
        mass = 0.0; // error
    mju_user_error = _errorCallback;
    mju_user_warning = _warningCallback;
    return (mass);
}

double CRigidBodyContainerDyn::computeInertia(int shapeHandle, C7Vector& tr, C3Vector& diagI, bool addRobustness)
{ // returns the diagonal mass-less inertia, for a density of 1000
    // Errors are silent here. Function can be reentrant (max. 1 time)
    mju_user_warning = nullptr;
    mju_user_error = nullptr;

    double mass = 0.0;
    CXSceneObject* shape = (CXSceneObject*)_simGetObject(shapeHandle);
    char* _dir = simGetStringParam(sim_stringparam_tempdir);
    std::string dir(_dir);
    std::string mjFile(_dir);
    simReleaseBuffer(_dir);
    mjFile += "/inertiaCalc.xml";
    CXmlSer* xmlDoc = new CXmlSer(mjFile.c_str());
    xmlDoc->pushNewNode("worldbody");
    xmlDoc->pushNewNode("body");
    xmlDoc->setAttr("name", "inertia");
    // ---------------
    double pos[3];
    double quat[4];
    _simGetObjectCumulativeTransformation(shape, pos, quat, 1);
    xmlDoc->setPosAttr("pos", pos);
    xmlDoc->setQuatAttr("quat", quat);
    // ---------------
    xmlDoc->pushNewNode("freejoint");
    xmlDoc->popNode();

    SInfo info;
    info.folder = dir;
    info.inertiaCalcRobust = addRobustness;
    _addMeshes(shape, xmlDoc, &info, nullptr, false);

    xmlDoc->popNode(); // body
    xmlDoc->popNode(); // worldbody

    xmlDoc->pushNewNode("asset");
    for (size_t i = 0; i < info.meshFiles.size(); i++)
    {
        xmlDoc->pushNewNode("mesh");
        xmlDoc->setAttr("file", info.meshFiles[i].c_str());
        xmlDoc->popNode();
    }
    xmlDoc->popNode(); // asset
    xmlDoc->popNode(); // file

    delete xmlDoc; // saves the file

    _displayWarningAboutCPUCompatibility();

    char error[1000] = "could not load binary model";
    mjModel* m = mj_loadXML(mjFile.c_str(), 0, error, 1000);
    if (m != nullptr)
    {
        mjData* d = mj_makeData(m);
        int id = mj_name2id(m, mjOBJ_BODY, "inertia");
        mass = m->body_mass[id];
        diagI(0) = m->body_inertia[3 * id + 0];
        diagI(1) = m->body_inertia[3 * id + 1];
        diagI(2) = m->body_inertia[3 * id + 2];
        diagI /= mass;
        tr.X(0) = m->body_ipos[3 * id + 0];
        tr.X(1) = m->body_ipos[3 * id + 1];
        tr.X(2) = m->body_ipos[3 * id + 2];
        tr.Q(0) = m->body_iquat[4 * id + 0];
        tr.Q(1) = m->body_iquat[4 * id + 1];
        tr.Q(2) = m->body_iquat[4 * id + 2];
        tr.Q(3) = m->body_iquat[4 * id + 3];
        mj_deleteData(d);
        mj_deleteModel(m);
    }
    else
    {
        if (!addRobustness)
            mass = computeInertia(shapeHandle, tr, diagI, true);
    }
    mju_user_error = _errorCallback;
    mju_user_warning = _warningCallback;
    return (mass);
}

bool CRigidBodyContainerDyn::_addMeshes(CXSceneObject* object, CXmlSer* xmlDoc, SInfo* info, std::vector<SMjGeom>* geoms, bool shapeIsStatic)
{ // retVal==false: display a warning if using non-pure non-convex shapes
    bool retVal = true;
    int objectHandle = _simGetObjectID(object);
    CXGeomWrap* geomInfo = (CXGeomWrap*)_simGetGeomWrapFromGeomProxy(object);

    if ((_simGetPurePrimitiveType(geomInfo) == sim_primitiveshape_none) && (_simIsGeomWrapConvex(geomInfo) == 0) && _simIsShapeDynamicallyRespondable(object) && (_simGetTreeDynamicProperty(object) & sim_objdynprop_respondable))
        retVal = false;

    double friction[3];
    friction[0] = simGetEngineFloatParam(sim_mujoco_body_friction1, -1, object, nullptr);
    friction[1] = simGetEngineFloatParam(sim_mujoco_body_friction2, -1, object, nullptr);
    friction[2] = simGetEngineFloatParam(sim_mujoco_body_friction3, -1, object, nullptr);
    double solref[2];
    solref[0] = simGetEngineFloatParam(sim_mujoco_body_solref1, -1, object, nullptr);
    solref[1] = simGetEngineFloatParam(sim_mujoco_body_solref2, -1, object, nullptr);
    double solimp[5];
    solimp[0] = simGetEngineFloatParam(sim_mujoco_body_solimp1, -1, object, nullptr);
    solimp[1] = simGetEngineFloatParam(sim_mujoco_body_solimp2, -1, object, nullptr);
    solimp[2] = simGetEngineFloatParam(sim_mujoco_body_solimp3, -1, object, nullptr);
    solimp[3] = simGetEngineFloatParam(sim_mujoco_body_solimp4, -1, object, nullptr);
    solimp[4] = simGetEngineFloatParam(sim_mujoco_body_solimp5, -1, object, nullptr);
    double solmix = simGetEngineFloatParam(sim_mujoco_body_solmix, -1, object, nullptr);
    int condim = simGetEngineInt32Param(sim_mujoco_body_condim, -1, object, nullptr);
    double margin = simGetEngineFloatParam(sim_mujoco_body_margin, -1, object, nullptr);
    double gap = 0.0;
    simGetFloatProperty(objectHandle, "mujoco.gap", &gap);
    int priority = simGetEngineInt32Param(sim_mujoco_body_priority, -1, object, nullptr);

    int componentListSize = _simGetGeometricCount(geomInfo);
    std::vector<CXGeometric*> componentList;
    componentList.resize(componentListSize);
    _simGetAllGeometrics(geomInfo, (void**)componentList.data());
    C7Vector objectPose;
    objectPose = C7Vector::identityTransformation;
    for (size_t i = 0; i < componentList.size(); i++)
    {
        xmlDoc->pushNewNode("geom");
        std::string nm(_getObjectName(object) + std::to_string(i));
        xmlDoc->setAttr("name", nm.c_str());
        xmlDoc->setAttr("friction", friction, 3);
        xmlDoc->setAttr("solref", solref, 2);
        xmlDoc->setAttr("solimp", solimp, 5);
        xmlDoc->setAttr("solmix", solmix);
        xmlDoc->setAttr("condim", condim);
        xmlDoc->setAttr("margin", margin);
        xmlDoc->setAttr("gap", gap);
        xmlDoc->setAttr("priority", priority);

        SMjGeom g;
        g.belongsToStaticItem = shapeIsStatic;
        g.itemType = shapeItem;
        g.objectHandle = objectHandle;
        g.name = nm;
        g.respondableMask = _simGetDynamicCollisionMask(object);

        if (geoms != nullptr)
            geoms->push_back(g);

        CXGeometric* sc = componentList[i];

        int pType = _simGetPurePrimitiveType(sc);
        C3Vector s;
        _simGetPurePrimitiveSizes(sc, s.data);

        C7Vector geomPose;
        geomPose.setIdentity();
        if (pType == sim_primitiveshape_heightfield)
        {
            xmlDoc->setAttr("type", "hfield");
            std::string fn(_getObjectName(object) + std::to_string(i));
            xmlDoc->setAttr("hfield", fn.c_str());
            fn += ".bin";

            int xCnt, yCnt;
            double minH, maxH;
            const double* hData = _simGetHeightfieldData(geomInfo, &xCnt, &yCnt, &minH, &maxH);

            double mmin = 1000.0;
            double mmax = -1000.0;
            std::vector<float> hDataF;
            hDataF.resize(xCnt * yCnt);
            for (size_t j = 0; j < size_t(xCnt * yCnt); j++)
            {
                hDataF[j] = float(hData[j]);
                double v = hData[j];
                if (v > mmax)
                    mmax = v;
                if (v < mmin)
                    mmin = v;
            }

            double mjhfbaseHeight = 0.1 * (s(0) + s(1));
            C3Vector pos(objectPose.X - objectPose.Q.getMatrix().axis[2] * (0.5 * (maxH - minH)));
            xmlDoc->setPosAttr("pos", pos.data);
            xmlDoc->setQuatAttr("quat", objectPose.Q.data);

            SHfield hf;
            hf.file = fn;
            hf.size[0] = 0.5 * s(0);
            hf.size[1] = 0.5 * s(1);
            hf.size[2] = maxH - minH;
            hf.size[3] = mjhfbaseHeight;
            info->heightfieldFiles.push_back(hf);

            fn = info->folder + "/" + fn;
            std::ofstream f(fn.c_str(), std::ios::out | std::ios::binary);
            f.write((char*)&yCnt, sizeof(int));
            f.write((char*)&xCnt, sizeof(int));
            f.write((char*)hDataF.data(), sizeof(float) * xCnt * yCnt);
        }
        else
            _simGetVerticesLocalFrame(object, sc, geomPose.X.data, geomPose.Q.data);
        geomPose = objectPose * geomPose;

        if ((pType == sim_primitiveshape_plane) || (pType == sim_primitiveshape_cuboid))
        {
            double z = s(2);
            if (z < 0.0001)
                z = 0.0001;
            xmlDoc->setAttr("type", "box");
            xmlDoc->setAttr("size", s(0) * 0.5, s(1) * 0.5, z * 0.5);
            xmlDoc->setPosAttr("pos", geomPose.X.data);
            xmlDoc->setQuatAttr("quat", geomPose.Q.data);
        }
        if ((pType == sim_primitiveshape_disc) || (pType == sim_primitiveshape_cylinder))
        {
            double z = s(2);
            if (z < 0.0001)
                z = 0.0001;
            xmlDoc->setAttr("type", "cylinder");
            xmlDoc->setAttr("size", s(0) * 0.5, z * 0.5);
            xmlDoc->setPosAttr("pos", geomPose.X.data);
            xmlDoc->setQuatAttr("quat", geomPose.Q.data);
        }
        if (pType == sim_primitiveshape_cone)
        {
            pType = sim_primitiveshape_none; // handle it as a mesh
        }
        if (pType == sim_primitiveshape_spheroid)
        {
            if ((fabs((s(0) - s(1)) / s(0)) < 0.001) && (fabs((s(0) - s(2)) / s(0)) < 0.001))
            {
                xmlDoc->setAttr("type", "sphere");
                xmlDoc->setAttr("size", s(0) * 0.5);
                xmlDoc->setPosAttr("pos", geomPose.X.data);
                xmlDoc->setQuatAttr("quat", geomPose.Q.data);
            }
            else
            { // We have a spheroid
                xmlDoc->setAttr("type", "ellipsoid");
                xmlDoc->setAttr("size", s(0) * 0.5, s(1) * 0.5, s(2) * 0.5);
                xmlDoc->setPosAttr("pos", geomPose.X.data);
                xmlDoc->setQuatAttr("quat", geomPose.Q.data);
            }
        }
        if (pType == sim_primitiveshape_capsule)
        {
            xmlDoc->setAttr("type", "capsule");
            double avgR = (s(0) + s(1)) * 0.25;
            xmlDoc->setAttr("size", avgR, s(2) * 0.5 - avgR);
            xmlDoc->setPosAttr("pos", geomPose.X.data);
            xmlDoc->setQuatAttr("quat", geomPose.Q.data);
        }
        if (pType == sim_primitiveshape_none)
        {
            xmlDoc->setAttr("type", "mesh");
            std::string fn(_getObjectName(object) + std::to_string(i)); // ideally get the hash of the local mesh for the filename. This would avoid creating file duplicates with identical meshes
            xmlDoc->setAttr("mesh", fn.c_str());
            fn += ".stl";
            info->meshFiles.push_back(fn);
            double* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(object, sc, &allVertices, &allVerticesSize, &allIndices, &allIndicesSize);
            double* mv = new double[allVerticesSize * 4];
            int* mi = new int[allIndicesSize * 4];
            double xmm[2] = {9999.0, -9999.0};
            double ymm[2] = {9999.0, -9999.0};
            double zmm[2] = {9999.0, -9999.0};
            for (int j = 0; j < allVerticesSize / 3; j++)
            { // relative to the shape's frame
                C3Vector v(allVertices + 3 * j + 0);
                v *= objectPose; //geomPose;
                mv[3 * j + 0] = v(0);
                mv[3 * j + 1] = v(1);
                mv[3 * j + 2] = v(2);
                if (v(0) < xmm[0])
                    xmm[0] = v(0);
                if (v(0) > xmm[1])
                    xmm[1] = v(0);
                if (v(1) < ymm[0])
                    ymm[0] = v(1);
                if (v(1) > ymm[1])
                    ymm[1] = v(1);
                if (v(2) < zmm[0])
                    zmm[0] = v(2);
                if (v(2) > zmm[1])
                    zmm[1] = v(2);
            }
            for (int j = 0; j < allIndicesSize; j++)
                mi[j] = allIndices[j];
            if (info->inertiaCalcRobust)
            { // give mesh an artifical volume (is grown in 3 directions by 10%)
                double dxyz = 0.1 * (xmm[1] - xmm[0] + ymm[1] - ymm[0] + zmm[1] - zmm[0]) / 3;
                int voff = allVerticesSize;
                int ioff = allIndicesSize;
                int ioi = allVerticesSize / 3;
                int io = ioi;
                ;
                for (int k = 0; k < 3; k++)
                {
                    C3Vector w(C3Vector::zeroVector);
                    w(k) = dxyz;
                    for (int j = 0; j < allVerticesSize / 3; j++)
                    { // relative to the shape's frame
                        C3Vector v(allVertices + 3 * j + 0);
                        v *= objectPose; //geomPose;
                        v += w;
                        mv[voff++] = v(0);
                        mv[voff++] = v(1);
                        mv[voff++] = v(2);
                    }
                    for (int j = 0; j < allIndicesSize; j++)
                        mi[ioff++] = allIndices[j] + io;
                    io += ioi;
                }
                allVerticesSize *= 4;
                allIndicesSize *= 4;
            }
            fn = info->folder + "/" + fn;
            if (simDoesFileExist(fn.c_str()) == 0)
                simExportMesh(4, fn.c_str(), 0, 1.0, 1, (const double**)&mv, &allVerticesSize, (const int**)&mi, &allIndicesSize, nullptr, nullptr);
            delete[] mi;
            delete[] mv;
            simReleaseBuffer((char*)allVertices);
            simReleaseBuffer((char*)allIndices);
        }

        xmlDoc->popNode();
    }
    return (retVal);
}

int CRigidBodyContainerDyn::_hasContentChanged()
{
    int occ, odc, hcc;
    simGetInt32Param(sim_intparam_objectcreationcounter, &occ);
    simGetInt32Param(sim_intparam_objectdestructioncounter, &odc);
    simGetInt32Param(sim_intparam_hierarchychangecounter, &hcc);
    int retVal = 0;
    if (_objectCreationCounter == -1)
        retVal = 1; // there is no content yet
    else
    { // in here we could eventually look a bit closer to see if the change is really involving pyhsics-simulated objects...
        if ((_objectCreationCounter != occ) && (_rebuildTrigger & 1))
            retVal = 2;
        if ((_objectDestructionCounter != odc) && (_rebuildTrigger & 2))
            retVal = 2;
        if ((_hierarchyChangeCounter != hcc) && (_rebuildTrigger & 4))
            retVal = 2;
        if (_xmlInjectionChanged && (_rebuildTrigger & 16))
            retVal = 2;
        if (_particleChanged && (_rebuildTrigger & 32))
            retVal = 2;
        if ((retVal == 0) && (_rebuildTrigger & 8))
        {
            _dynamicallyResetObjects.clear();
            int lSize = _simGetObjectListSize(sim_handle_all);
            for (int i = 0; i < lSize; i++)
            {
                CXSceneObject* it = (CXSceneObject*)_simGetObjectFromIndex(sim_handle_all, i);
                if (_simGetDynamicsFullRefreshFlag(it) != 0)
                {
                    retVal = 2;
                    _simSetDynamicsFullRefreshFlag(it, false);
                    _dynamicallyResetObjects[_getObjectName(it)];
                }
            }
        }
    }
    _objectCreationCounter = occ;
    _objectDestructionCounter = odc;
    _hierarchyChangeCounter = hcc;
    return (retVal);
}

bool CRigidBodyContainerDyn::hasSimulationHalted() const
{
    return _simulationHalted;
}

void CRigidBodyContainerDyn::handleDynamics(double dt, double simulationTime)
{
    if (!_simulationHalted)
    {
        double maxDynStep;
        simGetFloatParam(sim_floatparam_physicstimestep, &maxDynStep);

        _dynamicsCalculationPasses = int((dt / maxDynStep) + 0.5);
        if (_dynamicsCalculationPasses < 1)
            _dynamicsCalculationPasses = 1;
        _dynamicsInternalStepSize = dt / double(_dynamicsCalculationPasses);
        int contentChanged = _hasContentChanged();
        if (contentChanged > 0)
        {
            std::string err = _buildMujocoWorld(_dynamicsInternalStepSize, simulationTime, contentChanged > 1);
            if (err.size() > 0)
                simAddLog(LIBRARY_NAME, sim_verbosity_errors, err.c_str());
        }
        if (_mjModel != nullptr)
        {
            _simulationTime = simulationTime;
            _updateWorldFromCoppeliaSim();
            _contactInfo.clear();
            _contactPoints.clear();

            if (isDynamicContentAvailable())
            {
                for (int i = 0; i < _dynamicsCalculationPasses; i++)
                {
                    _currentPass = i + 1;
                    int integers[4] = {0, i + 1, _dynamicsCalculationPasses, 0};
                    double floats[1] = {_dynamicsInternalStepSize};
                    _simDynCallback(integers, floats);
                    _handleKinematicBodies_step(double(i + 1) / double(_dynamicsCalculationPasses), dt);
                    _stepDynamics(_dynamicsInternalStepSize, i);
                    _handleContactPoints(i);
                    _simulationTime += _dynamicsInternalStepSize;
                    _reportWorldToCoppeliaSim(_simulationTime, i, _dynamicsCalculationPasses);
                    integers[3] = 1;
                    _simDynCallback(integers, floats);
                }
            }
            _clearAdditionalForcesAndTorques();
        }
    }
}

int CRigidBodyContainerDyn::_contactCallback(const mjModel* m, mjData* d, int geom1, int geom2)
{
    return (_dynWorld->_handleContact(m, d, geom1, geom2));
}

int CRigidBodyContainerDyn::_handleContact(const mjModel* m, mjData* d, int geom1, int geom2)
{
    int retVal = 1; // ignore this contact
    int ind1 = _geomIdIndex[geom1];
    int ind2 = _geomIdIndex[geom2];
    if ((ind1 >= 0) && (ind2 >= 0))
    {
        SMjGeom* gm1 = &_allGeoms[ind1];
        SMjGeom* gm2 = &_allGeoms[ind2];
        int body1Handle = gm1->objectHandle;
        int body2Handle = gm2->objectHandle;
        bool canCollide = false;
        if ((gm1->itemType == particleItem) && (gm2->itemType == particleItem))
        { // particle-particle
            canCollide = (gm1->respondableMask & gm2->respondableMask & 0x00ff);
        }
        if ((gm1->itemType == compositeItem) && (gm2->itemType == compositeItem))
        { // composite-composite
            if (gm1->objectHandle == gm2->objectHandle)
                canCollide = (gm1->respondableMask & gm2->respondableMask & 0x000f); // same composite
            else
                canCollide = (gm1->respondableMask & gm2->respondableMask & 0x00f0); // other composites
        }
        if (((gm1->itemType == shapeItem) && (gm2->itemType > shapeItem)) || ((gm1->itemType > shapeItem) && (gm2->itemType == shapeItem)))
        { // shape-particle or shape-composite
            canCollide = true;
            unsigned int collFA = gm1->respondableMask & 0xff00;
            if (gm1->itemType == shapeItem)
            {
                CXSceneObject* shapeA = (CXSceneObject*)_simGetObject(body1Handle);
                canCollide = _simIsShapeDynamicallyRespondable(shapeA) != 0;
            }
            unsigned int collFB = gm2->respondableMask & 0xff00;
            if (gm2->itemType == shapeItem)
            {
                CXSceneObject* shapeB = (CXSceneObject*)_simGetObject(body2Handle);
                canCollide = _simIsShapeDynamicallyRespondable(shapeB) != 0;
            }
            canCollide = (canCollide && (collFA & collFB & 0xff00)); // we are global
        }
        if ((gm1->itemType == shapeItem) && (gm2->itemType == shapeItem))
        { // shape-shape
            CXSceneObject* shapeA = (CXSceneObject*)_simGetObject(body1Handle);
            CXSceneObject* shapeB = (CXSceneObject*)_simGetObject(body2Handle);
            unsigned int collFA = gm1->respondableMask;
            unsigned int collFB = gm2->respondableMask;
            canCollide = (_simIsShapeDynamicallyRespondable(shapeA) && _simIsShapeDynamicallyRespondable(shapeB));

            // Following used to not have the expected effect: it added strangely more contacts. But somehow now that works just fine...
            if (gm1->belongsToStaticItem && gm2->belongsToStaticItem)
                canCollide = false;

            if (canCollide)
            {
                CXSceneObject* lastPA = (CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeA);
                CXSceneObject* lastPB = (CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeB);
                if (lastPA == lastPB)
                    canCollide = (collFA & collFB & 0x00ff); // we are local
                else
                    canCollide = (collFA & collFB & 0xff00); // we are global
            }
        }
        if (canCollide)
        {
            int dataInt[3] = {0, 0, 0};
            double dataFloat[14] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            int customHandleRes = _simHandleCustomContact(body1Handle, body2Handle, sim_physics_mujoco, dataInt, dataFloat);
            if (customHandleRes != 0)
                retVal = 0; // should collide
        }
    }
    return (retVal);
}

void CRigidBodyContainerDyn::_controlCallback(const mjModel* m, mjData* d)
{
    _dynWorld->_handleControl(m, d);
}

void CRigidBodyContainerDyn::_handleControl(const mjModel* m, mjData* d)
{
    // Handle joint control here mainly. But also handle application of additional forces/torques
    // Also, we handle item "deactivation" when outside of the "_dynamicActivityRange" by:
    // 1. Disabling gravity, i.e. effectively adding a counter force
    // 2. Adding damping to the involved bodies
    // 3. Setting the velocity to 0 for involved freejoints
    // 4. Disable collision response for involved bodies

    C3Vector gravity;
    _simGetGravity(gravity.data);
    _particleCont->handleAntiGravityForces_andFluidFrictionForces(gravity);

    struct SAdhesion
    {
        int mjIdAdhesion;
        double ctrl;
    };

    std::vector<SAdhesion> adhesionItems;

    // handle additional forces/torques:
    for (size_t i = 0; i < _allShapes.size(); i++)
    {
        if (_allShapes[i].itemType == shapeItem)
        { // exclude shapes that do not exist in CoppeliaSim
            CXSceneObject* shape = (CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
            if (shape != nullptr)
            {
                if (_allShapes[i].shapeMode >= shapeModes::freeMode)
                {
                    int bodyId = _allShapes[i].mjId;
                    C3Vector vf, vt;
                    _simGetAdditionalForceAndTorque(shape, vf.data, vt.data);
                    d->xfrc_applied[6 * bodyId + 0] = vf(0);
                    d->xfrc_applied[6 * bodyId + 1] = vf(1);
                    d->xfrc_applied[6 * bodyId + 2] = vf(2);
                    d->xfrc_applied[6 * bodyId + 3] = vt(0);
                    d->xfrc_applied[6 * bodyId + 4] = vt(1);
                    d->xfrc_applied[6 * bodyId + 5] = vt(2);
                    // handle objects lower than _dynamicActivityRange: (if they continue to fall, they will reset the Mujoco simulation eventually)
                    if (d->xpos[3 * bodyId + 2] < -_dynamicActivityRange)
                    {
                        double mass = m->body_mass[bodyId];
                        double avgI = (m->body_inertia[bodyId + 0] + m->body_inertia[bodyId + 1] + m->body_inertia[bodyId + 2]) / 3.0;
                        // Disable gravity:
                        d->xfrc_applied[6 * bodyId + 2] = mass * gravity(2) * -1.0;
                        // Apply add. force inv. prop. to the velocity:
                        d->xfrc_applied[6 * bodyId + 0] -= mass * _mjData->cvel[6 * bodyId + 3] * 1.0;
                        d->xfrc_applied[6 * bodyId + 1] -= mass * _mjData->cvel[6 * bodyId + 4] * 1.0;
                        d->xfrc_applied[6 * bodyId + 2] -= mass * _mjData->cvel[6 * bodyId + 5] * 1.0;
                        // Apply add. torque inv. prop. to the velocity:
                        d->xfrc_applied[6 * bodyId + 3] -= avgI * _mjData->cvel[6 * bodyId + 0] * 0.00001;
                        d->xfrc_applied[6 * bodyId + 4] -= avgI * _mjData->cvel[6 * bodyId + 1] * 0.00001;
                        d->xfrc_applied[6 * bodyId + 5] -= avgI * _mjData->cvel[6 * bodyId + 2] * 0.00001;
                        // Disable collision response:
                        for (size_t j = 0; j < _allShapes[i].geomIndices.size(); j++)
                            _allGeoms[_allShapes[i].geomIndices[j]].respondableMask = 0;
                    }
                }
                if (_allShapes[i].mjIdAdhesion >= 0)
                {
                    SAdhesion item;
                    item.mjIdAdhesion = _allShapes[i].mjIdAdhesion;
                    simGetFloatProperty(_allShapes[i].objectHandle, "mujoco.adhesionctrl", &item.ctrl);
                    adhesionItems.push_back(item);
                }
            }
        }
    }

    // Joints:
    // Order them from leaf to root (important for later, when we need to access force/torque data for child joints):
    std::vector<SMjJoint*> jointMujocoItems;
    std::vector<size_t> allCtrlJoints;
    std::set<CXSceneObject*> allCtrlJoints_set;
    for (size_t i = 0; i < _allJoints.size(); i++)
    {
        CXSceneObject* joint = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if ((joint != nullptr) && (_allJoints[i].actMode > 0) && (_allJoints[i].jointType != sim_joint_spherical))
        {
            _allJoints[i].object = joint;
            allCtrlJoints.push_back(i);
            allCtrlJoints_set.insert(joint);
        }
    }
    while (allCtrlJoints.size() > 0)
    {
        size_t ind = 0;
        while (ind < allCtrlJoints.size())
        {
            size_t i = allCtrlJoints[ind];
            CXSceneObject* joint =  _allJoints[i].object;
            CXSceneObject* p = joint;
            bool hasJointParent = false;
            while (true)
            {
                p = (CXSceneObject*)_simGetParentObject(p);
                if (p == nullptr)
                    break;
                else
                {
                    if (allCtrlJoints_set.find(p) != allCtrlJoints_set.end())
                    {
                        hasJointParent = true;
                        break;
                    }
                }
            }
            if (hasJointParent)
                ind++;
            else
            {
                allCtrlJoints.erase(allCtrlJoints.begin() + ind);
                allCtrlJoints_set.erase(joint);
                jointMujocoItems.insert(jointMujocoItems.begin(), &_allJoints[i]);
            }
        }
    }
/*
    // Following is without ordering the joints from leaf to root:
    std::vector<SMjJoint*> jointMujocoItems;
    for (size_t i = 0; i < _allJoints.size(); i++)
    {
        CXSceneObject* joint = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if ((joint != nullptr) && (_allJoints[i].actMode > 0) && (_allJoints[i].jointType != sim_joint_spherical))
        {
            _allJoints[i].object = joint;
            jointMujocoItems.push_back(&_allJoints[i]);
        }
    }
    */

    for (size_t i = 0; i < _allFreejoints.size(); i++)
    {
        int padr = _mjModel->jnt_qposadr[_allFreejoints[i].mjId];
        if ((fabs(_mjData->qpos[padr + 0]) > _dynamicActivityRange) || (fabs(_mjData->qpos[padr + 1]) > _dynamicActivityRange) || (fabs(_mjData->qpos[padr + 2]) > _dynamicActivityRange))
        { // make sure that this joint doesn't start making the rest of the simulation unstable
            int vadr = _mjModel->jnt_dofadr[_allFreejoints[i].mjId];
            for (size_t j = 0; j < 6; j++)
                _mjData->qvel[vadr + j] = 0.0;
        }
    }

    // First get control values:
    for (size_t i = 0; i < jointMujocoItems.size(); i++)
        _handleMotorControl(jointMujocoItems[i]);

    // Compute inverse dynamics to figure out force/torque values for a perfect velocity control:
    // mj_copyData(_mjDataCopy,m,d); // very slow
    _mjDataCopy->time = d->time;
    mju_copy(_mjDataCopy->qpos, d->qpos, m->nq);
    mju_copy(_mjDataCopy->qvel, d->qvel, m->nv);
    mju_copy(_mjDataCopy->act, d->act, m->na);
    mju_copy(_mjDataCopy->mocap_pos, d->mocap_pos, 3 * m->nmocap);
    mju_copy(_mjDataCopy->mocap_quat, d->mocap_quat, 4 * m->nmocap);
    mju_copy(_mjDataCopy->userdata, d->userdata, m->nuserdata);
    mju_copy(_mjDataCopy->qacc_warmstart, d->qacc_warmstart, m->nv);
    for (size_t i = 0; i < jointMujocoItems.size(); i++)
    {
        SMjJoint* mujocoItem = jointMujocoItems[i];
        if ((mujocoItem->actMode == 1) || (mujocoItem->actMode == 3))
            _mjDataCopy->ctrl[mujocoItem->mjIdActuator] = mujocoItem->jointCtrlToApply;
        if (mujocoItem->actMode == 2)
        {
            if (mujocoItem->tendonJoint)
            { // TODO
            }
            else
            {
                int vadr = m->jnt_dofadr[mujocoItem->mjId];
                _mjDataCopy->qacc[vadr] = mujocoItem->jointCtrlDv / m->opt.timestep;
            }
        }
    }
    for (size_t i = 0; i < adhesionItems.size(); i++)
        _mjDataCopy->ctrl[adhesionItems[i].mjIdAdhesion] = adhesionItems[i].ctrl;

    mj_inverse(m, _mjDataCopy);

    // Now apply joint forces/torques:
    for (size_t i = 0; i < jointMujocoItems.size(); i++)
    {
        SMjJoint* mujocoItem = jointMujocoItems[i];
        if ((mujocoItem->actMode == 1) || (mujocoItem->actMode == 3))
            d->ctrl[mujocoItem->mjIdActuator] = mujocoItem->jointCtrlToApply;
        if (mujocoItem->actMode == 2)
        {
            if (mujocoItem->tendonJoint)
            { // TODO
            }
            else
            {
                int vadr = m->jnt_dofadr[mujocoItem->mjId];
                double f = _mjDataCopy->qfrc_inverse[vadr];
                if (f > fabs(mujocoItem->jointCtrlToApply))
                    f = fabs(mujocoItem->jointCtrlToApply);
                else if (f < -fabs(mujocoItem->jointCtrlToApply))
                    f = -fabs(mujocoItem->jointCtrlToApply);
                d->ctrl[mujocoItem->mjIdActuator] = f;
            }
        }
    }
    for (size_t i = 0; i < adhesionItems.size(); i++)
        d->ctrl[adhesionItems[i].mjIdAdhesion] = adhesionItems[i].ctrl;

    _firstCtrlPass = false;
    if (_rg4Cnt > 0)
    { // we have Runge-Kutta4 integrator
        _rg4Cnt++;
        if (_rg4Cnt > 4)
            _rg4Cnt = 1;
    }
}

void CRigidBodyContainerDyn::_handleMotorControl(SMjJoint* mujocoItem)
{
    CXSceneObject* joint = mujocoItem->object;
    int ctrlMode = _simGetJointDynCtrlMode(joint);
    double dynStepSize = CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    double e = 0.0;
    double currentPos, currentVel, currentAccel;
    int auxV = 0;
    if (mujocoItem->tendonJoint)
    { // TODO tendon accel?
        currentPos = _mjData->ten_length[mujocoItem->mjId];
        currentVel = _mjData->ten_velocity[mujocoItem->mjId];
        auxV = 2; // we provide vel info too
    }
    else
    {
        int padr = _mjModel->jnt_qposadr[mujocoItem->mjId];
        currentPos = _mjData->qpos[padr];
        int vadr = _mjModel->jnt_dofadr[mujocoItem->mjId];
        currentVel = _mjData->qvel[vadr];
        currentAccel = _mjData->qacc[vadr];
        auxV = 2 + 4; // we provide vel and accel info too
    }
    if (mujocoItem->jointType == sim_joint_revolute)
    {
        if (ctrlMode >= sim_jointdynctrl_position)
        {
            if (_simGetJointPositionInterval(joint, nullptr, nullptr) == 0)
                e = _getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint), currentPos);
            else
                e = _simGetDynamicMotorTargetPosition(joint) - currentPos;
        }
    }
    else
    {
        if (ctrlMode >= sim_jointdynctrl_position)
            e = _simGetDynamicMotorTargetPosition(joint) - currentPos;
    }

    if (_firstCtrlPass)
        auxV |= 1;
    int inputValuesInt[5] = {0, 0, 0, 0, 0};
    inputValuesInt[0] = _currentPass;
    inputValuesInt[1] = _dynamicsCalculationPasses;
    inputValuesInt[2] = _rg4Cnt;
    double inputValuesFloat[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    inputValuesFloat[0] = currentPos;
    double eff = _mjData->actuator_force[mujocoItem->mjIdActuator];
    inputValuesFloat[1] = -eff;
    if (_rg4Cnt == 0)
        inputValuesFloat[2] = dynStepSize;
    else
    {
        if (_rg4Cnt > 1)
        {
            if (_rg4Cnt < 4)
                inputValuesFloat[2] = dynStepSize * 0.5;
            else
                inputValuesFloat[2] = dynStepSize;
        }
        else
            inputValuesFloat[2] = 0.0;
    }
    inputValuesFloat[3] = e;
    inputValuesFloat[4] = currentVel;
    inputValuesFloat[5] = currentAccel;
    double outputValues[5];
    int res = _simHandleJointControl(joint, auxV, inputValuesInt, inputValuesFloat, outputValues);

    //    if ((res&2)==0)
    { // motor is not locked
        if ((res & 1) == 1)
        { // motor on
            mujocoItem->jointCtrlDv = outputValues[0] - currentVel;
            mujocoItem->jointCtrlToApply = outputValues[1]; // force (or ctrl) to apply
        }
        else
        { // motor off
            mujocoItem->jointCtrlDv = 0.0;
            mujocoItem->jointCtrlToApply = 0.0;
        }
    }
    //    else
    //    { // motor is locked. Not supported by Mujoco?!
    //    }
}

void CRigidBodyContainerDyn::_errorCallback(const char* err)
{
    std::string msg("Engine error: ");
    msg += err;
    simAddLog(LIBRARY_NAME, sim_verbosity_errors, msg.c_str());
    _simulationHalted = true;
    simPauseSimulation();
}

void CRigidBodyContainerDyn::_warningCallback(const char* warn)
{
    std::string msg("Engine warning: ");
    msg += warn;
    simAddLog(LIBRARY_NAME, sim_verbosity_warnings, msg.c_str());
    if (msg.find("simulation is unstable") != std::string::npos)
        _simulationHalted = true;
    if (msg.find("constraint buffer is full") != std::string::npos)
        _simulationHalted = true;
    if (msg.find("Nan, Inf or huge value") != std::string::npos)
        _simulationHalted = true;
    if (msg.find("Inertia matrix is too close to singular") != std::string::npos)
        _simulationHalted = true;
    if (msg.find("Insufficient arena memory") != std::string::npos)
        _simulationHalted = true;
    if (_simulationHalted)
        simPauseSimulation();
}

void CRigidBodyContainerDyn::_handleContactPoints(int dynPass)
{
    _contactPoints.clear();
    for (int i = 0; i < _mjData->ncon; i++)
    {
        C3Vector pos(_mjData->contact[i].pos[0], _mjData->contact[i].pos[1], _mjData->contact[i].pos[2]); // midpoint
        _contactPoints.push_back(pos(0));
        _contactPoints.push_back(pos(1));
        _contactPoints.push_back(pos(2));
        // _mjData->contact[i]->dist // dist between pts, neg=penetration

        SContactInfo ci;
        ci.subPassNumber = dynPass;
        ci.objectID1 = -1; // unknown item
        ci.objectID2 = -1; // unknown item
        if ((_mjData->contact[i].geom1 < int(_geomIdIndex.size())) && (_mjData->contact[i].geom1 >= 0))
            ci.objectID1 = _allGeoms[_geomIdIndex[_mjData->contact[i].geom1]].objectHandle;
        if ((_mjData->contact[i].geom2 < int(_geomIdIndex.size())) && (_mjData->contact[i].geom2 >= 0))
            ci.objectID2 = _allGeoms[_geomIdIndex[_mjData->contact[i].geom2]].objectHandle;
        ci.position = pos;

        // _mjData->contact[i].frame is a transposed rotation matrix. Axis X is the contact normal vector
        mjtNum* frame = _mjData->contact[i].frame;
        ci.surfaceNormal = C3Vector(frame[0], frame[1], frame[2]);
        mjtNum ft[6];
        mj_contactForce(_mjModel, _mjData, i, ft); // forceTorque expressed in contact frame
        ci.directionAndAmplitude(0) = ft[0] * frame[0] + ft[1] * frame[3] + ft[2] * frame[6];
        ci.directionAndAmplitude(1) = ft[0] * frame[1] + ft[1] * frame[4] + ft[2] * frame[7];
        ci.directionAndAmplitude(2) = ft[0] * frame[2] + ft[1] * frame[5] + ft[2] * frame[8];
        // following 2 fixed on 06.01.2025:
        ci.surfaceNormal *= -1.0;
        ci.directionAndAmplitude *= -1.0;
        _contactInfo.push_back(ci);
    }
}

double CRigidBodyContainerDyn::_getAngleMinusAlpha(double angle, double alpha)
{ // Returns angle-alpha. Angle and alpha are cyclic angles!!
    double sinAngle0 = sinf(angle);
    double sinAngle1 = sinf(alpha);
    double cosAngle0 = cosf(angle);
    double cosAngle1 = cosf(alpha);
    double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
    double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
    double angle_da = atan2(sin_da, cos_da);
    return angle_da;
}

std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    std::string v("Mujoco v");
    v += mj_versionString();
    return (v);
}

bool CRigidBodyContainerDyn::_updateWorldFromCoppeliaSim()
{
    for (size_t i = 0; i < _allShapes.size(); i++)
    {
        if (_allShapes[i].itemType == shapeItem)
        { // only shapes that exist in CoppeliaSim
            if (_allShapes[i].shapeMode <= shapeModes::kinematicMode)
            { // prepare for static shape motion interpol.
                int mid;
                if (_allShapes[i].shapeMode == shapeModes::kinematicMode)
                    mid = _allShapes[i].mjIdStatic;
                if (_allShapes[i].shapeMode == shapeModes::staticMode)
                    mid = _allShapes[i].mjId;
                int bodyId = _mjModel->body_mocapid[mid];
                _allShapes[i].staticShapeStart.X = C3Vector(_mjData->mocap_pos[3 * bodyId + 0], _mjData->mocap_pos[3 * bodyId + 1], _mjData->mocap_pos[3 * bodyId + 2]);
                _allShapes[i].staticShapeStart.Q = C4Vector(_mjData->mocap_quat[4 * bodyId + 0], _mjData->mocap_quat[4 * bodyId + 1], _mjData->mocap_quat[4 * bodyId + 2], _mjData->mocap_quat[4 * bodyId + 3]);
                _allShapes[i].staticShapeGoal = _allShapes[i].staticShapeStart;
                CXSceneObject* shape = (CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (shape != nullptr)
                {
                    /*
                    // Trying initial velocity for static shapes (e.g. simple conveyor)
                    C3Vector lv,av;
                    _simGetInitialDynamicVelocity(_allShapes[i].object,lv.data);
                    _simGetInitialDynamicAngVelocity(_allShapes[i].object,av.data);
                    if ( (lv.getLength()>0.0)||(av.getLength()>0.0) )
                    { // when a static shape has an initial velocity (e.g. simple conveyor)
                        _simSetInitialDynamicVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                        _simSetInitialDynamicAngVelocity(_allShapes[i].object,C3Vector::zeroVector.data); // important to reset it
                        double dt=_dynamicsInternalStepSize*double(_dynamicsCalculationPasses);
                        _allShapes[i].staticShapeGoal.X(0)+=lv(0)*dt;
                        _allShapes[i].staticShapeGoal.X(1)+=lv(1)*dt;
                        _allShapes[i].staticShapeGoal.X(2)+=lv(2)*dt;
                        int nvadr=_mjModel->jnt_dofadr[_allShapes[i].mjIdJoint];
                        _mjData->qvel[nvadr+0]=lv(0);
                        _mjData->qvel[nvadr+1]=lv(1);
                        _mjData->qvel[nvadr+2]=lv(2);
                    }
                    else
                    //*/
                    _simGetObjectCumulativeTransformation(shape, _allShapes[i].staticShapeGoal.X.data, _allShapes[i].staticShapeGoal.Q.data, false);
                }
            }
        }
    }

    return (true);
}

void CRigidBodyContainerDyn::_handleKinematicBodies_step(double t, double cumulatedTimeStep)
{
    for (size_t i = 0; i < _allShapes.size(); i++)
    {
        if (_allShapes[i].itemType == shapeItem)
        { // only shapes that exist in CoppeliaSim
            if (_allShapes[i].shapeMode <= shapeModes::kinematicMode)
            {
                CXSceneObject* shape = (CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
                if (shape != nullptr)
                {
                    int mid;
                    if (_allShapes[i].shapeMode == shapeModes::kinematicMode)
                        mid = _allShapes[i].mjIdStatic;
                    if (_allShapes[i].shapeMode == shapeModes::staticMode)
                        mid = _allShapes[i].mjId;
                    int bodyId = _mjModel->body_mocapid[mid];
                    C7Vector tr;
                    tr.buildInterpolation(_allShapes[i].staticShapeStart, _allShapes[i].staticShapeGoal, t);
                    _mjData->mocap_pos[3 * bodyId + 0] = tr.X(0);
                    _mjData->mocap_pos[3 * bodyId + 1] = tr.X(1);
                    _mjData->mocap_pos[3 * bodyId + 2] = tr.X(2);
                    _mjData->mocap_quat[4 * bodyId + 0] = tr.Q(0);
                    _mjData->mocap_quat[4 * bodyId + 1] = tr.Q(1);
                    _mjData->mocap_quat[4 * bodyId + 2] = tr.Q(2);
                    _mjData->mocap_quat[4 * bodyId + 3] = tr.Q(3);
                }
            }
        }
    }
}

void CRigidBodyContainerDyn::_reportWorldToCoppeliaSim(double simulationTime, int currentPass, int totalPasses)
{
    // First joints:
    for (size_t i = 0; i < _allJoints.size(); i++)
    {
        CXSceneObject* joint = (CXSceneObject*)_simGetObject(_allJoints[i].objectHandle);
        if (joint != nullptr)
        {
            if (_allJoints[i].tendonJoint)
            { // TODO
                _simSetJointPosition(joint, _mjData->actuator_length[_allJoints[i].mjIdActuator]);
                /*
                int totalPassesCount=0;
                if (currentPass==totalPasses-1)
                    totalPassesCount=totalPasses;
                _simAddJointCumulativeForcesOrTorques(joint,-_mjData->actuator_force[_allJoints[i].mjIdActuator],totalPassesCount,simulationTime);
                _simSetJointVelocity(joint,_mjData->qvel[vadr]);
                */
            }
            else
            {
                int padr = _mjModel->jnt_qposadr[_allJoints[i].mjId];
                int vadr = _mjModel->jnt_dofadr[_allJoints[i].mjId];
                if (_allJoints[i].jointType == sim_joint_spherical)
                {
                    C4Vector q(_mjData->qpos[padr + 0], _mjData->qpos[padr + 1], _mjData->qpos[padr + 2], _mjData->qpos[padr + 3]);
                    q = _allJoints[i].initialBallQuat2.getInverse() * (q * _allJoints[i].initialBallQuat) * _allJoints[i].initialBallQuat2;
                    // Something is still not right here, in situations where the joint is involved in loop closure, and has an initialBallQuat different from the identity quaternion
                    // But this as only a visual effect on the spherical joint only (involved shapes are correctly placed)
                    q.normalize();
                    _simSetJointSphericalTransformation(joint, q.data, simulationTime);
                }
                else
                {
                    _simSetJointPosition(joint, _mjData->qpos[padr]);
                    int totalPassesCount = 0;
                    if (currentPass == totalPasses - 1)
                        totalPassesCount = totalPasses;
                    _simAddJointCumulativeForcesOrTorques(joint, -_mjData->actuator_force[_allJoints[i].mjIdActuator], totalPassesCount, simulationTime);
                    _simSetJointVelocity(joint, _mjData->qvel[vadr]);
                }
            }
        }
    }

    // Then force sensors:
    for (size_t i = 0; i < _allForceSensors.size(); i++)
    {
        CXSceneObject* forceSens = (CXSceneObject*)_simGetObject(_allForceSensors[i].objectHandle);
        if (forceSens != nullptr)
        {
            int fadr = _mjModel->sensor_adr[_allForceSensors[i].mjId];
            int tadr = _mjModel->sensor_adr[_allForceSensors[i].mjId2];
            int totalPassesCount = 0;
            if (currentPass == totalPasses - 1)
                totalPassesCount = totalPasses;
            double f[3] = {-_mjData->sensordata[fadr + 0], -_mjData->sensordata[fadr + 1], -_mjData->sensordata[fadr + 2]};
            double t[3] = {-_mjData->sensordata[tadr + 0], -_mjData->sensordata[tadr + 1], -_mjData->sensordata[tadr + 2]};
            _simAddForceSensorCumulativeForcesAndTorques(forceSens, f, t, totalPassesCount, simulationTime);
        }
    }

    // Then shapes:
    for (size_t i = 0; i < _allShapes.size(); i++)
    {
        if (_allShapes[i].itemType == shapeItem)
        { // only shapes that exist in CoppeliaSim (we want to exclude dummyShapeItem, particleItem, etc.)
            CXSceneObject* shape = (CXSceneObject*)_simGetObject(_allShapes[i].objectHandle);
            if (shape != nullptr)
            {
                int bodyId = _allShapes[i].mjId;
                if (_allShapes[i].shapeMode == shapeModes::freeMode)
                {
                    C7Vector tr;
                    tr.X = C3Vector(_mjData->xpos[3 * bodyId + 0], _mjData->xpos[3 * bodyId + 1], _mjData->xpos[3 * bodyId + 2]);
                    tr.Q = C4Vector(_mjData->xquat[4 * bodyId + 0], _mjData->xquat[4 * bodyId + 1], _mjData->xquat[4 * bodyId + 2], _mjData->xquat[4 * bodyId + 3]);
                    _simDynReportObjectCumulativeTransformation(shape, tr.X.data, tr.Q.data, simulationTime);
                }

                double av[3] = {_mjData->cvel[6 * bodyId + 0], _mjData->cvel[6 * bodyId + 1], _mjData->cvel[6 * bodyId + 2]};
                double lv[3] = {_mjData->cvel[6 * bodyId + 3], _mjData->cvel[6 * bodyId + 4], _mjData->cvel[6 * bodyId + 5]};
                // Above is COM vel. in spatial vector
                double comPos[3] = {_mjData->xipos[3 * bodyId + 0], _mjData->xipos[3 * bodyId + 1], _mjData->xipos[3 * bodyId + 2]};
                // Above com is relative to the origin. We however need the com relative to the last parent body.
                // This is strange and not the "spatial velocity" that I know of:
                int parentId = bodyId;
                while (true)
                {
                    int p = _mjModel->body_parentid[parentId];
                    if (p > 0)
                        parentId = p;
                    else
                        break;
                }
                /*
                printf("body:      %s\n",_mjModel->names+_mjModel->name_bodyadr[bodyId]);
                printf("parent:    %s\n",_mjModel->names+_mjModel->name_bodyadr[parentId]);
                printf("av:        %.4, %.4, %.4\n",av[0],av[1],av[2]);
                printf("lv:        %.4, %.4, %.4\n",lv[0],lv[1],lv[2]);
                //*/
                double llv[3];
                if (parentId != bodyId)
                {
                    comPos[0] -= _mjData->xipos[3 * parentId + 0];
                    comPos[1] -= _mjData->xipos[3 * parentId + 1];
                    comPos[2] -= _mjData->xipos[3 * parentId + 2];
                    double x[3];
                    x[0] = comPos[1] * av[2] - comPos[2] * av[1];
                    x[1] = comPos[2] * av[0] - comPos[0] * av[2];
                    x[2] = comPos[0] * av[1] - comPos[1] * av[0];
                    llv[0] = lv[0] - x[0];
                    llv[1] = lv[1] - x[1];
                    llv[2] = lv[2] - x[2];
                    /*
                    printf("comPos:    %.4, %.4, %.4\n",comPos[0],comPos[1],comPos[2]);
                    printf("comPos^av: %.4, %.4, %.4\n",x[0],x[1],x[2]);
                    //*/
                }
                else
                {
                    llv[0] = lv[0];
                    llv[1] = lv[1];
                    llv[2] = lv[2];
                }
                _simSetShapeDynamicVelocity(shape, llv, av, simulationTime);
            }
        }
    }

    // Finally particles:
    _particleCont->updateParticlesPosition(simulationTime);
}

bool CRigidBodyContainerDyn::isDynamicContentAvailable() const
{
    return (_allShapes.size() > 0);
}

void CRigidBodyContainerDyn::_stepDynamics(double dt, int pass)
{
    mj_step(_mjModel, _mjData);
}

bool CRigidBodyContainerDyn::removeInjection(int injectionId)
{
    bool retVal = false;
    for (size_t i = 0; i < _xmlInjections.size(); i++)
    { // old
        if (_xmlInjections[i].injectionId == injectionId)
        {
            _xmlInjections.erase(_xmlInjections.begin() + i);
            retVal = true;
            break;
        }
    }
    if (!retVal)
    {
        for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
        {
            if (_xmlCompositeInjections[i].injectionId == injectionId)
            {
                _xmlCompositeInjections.erase(_xmlCompositeInjections.begin() + i);
                retVal = true;
                break;
            }
        }
    }
    if (!retVal)
    {
        for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
        {
            if (_xmlFlexcompInjections[i].injectionId == injectionId)
            {
                _xmlFlexcompInjections.erase(_xmlFlexcompInjections.begin() + i);
                retVal = true;
                break;
            }
        }
    }
    if (!retVal)
    {
        for (size_t i = 0; i < _xmlGeneralInjections.size(); i++)
        {
            if (_xmlGeneralInjections[i].injectionId == injectionId)
            {
                _xmlGeneralInjections.erase(_xmlGeneralInjections.begin() + i);
                retVal = true;
                break;
            }
        }
    }
    if (retVal)
        _dynWorld->_xmlInjectionChanged = true;
    return (retVal);
}

int CRigidBodyContainerDyn::injectXml(const char* xml, const char* element, int objectHandle, const char* cbFunc, int cbScript, const char* cbId)
{ // old
    SInject inf;
    inf.injectionId = _nextInjectionId++;
    inf.cbFunc = cbFunc;
    inf.cbScript = cbScript;
    inf.cbId = cbId;
    inf.xml = xml;
    inf.element = element;
    inf.objectHandle = objectHandle;
    _xmlInjections.push_back(inf);
    _dynWorld->_xmlInjectionChanged = true;
    return (inf.injectionId);
}

int CRigidBodyContainerDyn::injectCompositeXml(const char* xml, int shapeHandle, const char* element, const char* prefix, const size_t* count, const char* type, int respondableMask, double grow, const char* cbFunc, int cbScript)
{
    SCompositeInject inf;
    inf.injectionId = _nextInjectionId++;
    inf.cbFunc = cbFunc;
    inf.cbScript = cbScript;
    inf.xml = xml;
    inf.shapeHandle = shapeHandle;
    inf.element = element;
    inf.prefix = prefix;
    inf.type = type;
    inf.respondableMask = respondableMask;
    inf.grow = grow;
    for (size_t i = 0; i < 3; i++)
        inf.count[i] = count[i];
    _xmlCompositeInjections.push_back(inf);
    _dynWorld->_xmlInjectionChanged = true;
    return (inf.injectionId);
}

int CRigidBodyContainerDyn::getCompositeIndexFromPrefix(const char* prefix)
{
    int index = -1;
    for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
    {
        if (_xmlCompositeInjections[i].prefix == prefix)
        {
            index = int(i);
            break;
        }
    }
    return index;
}

int CRigidBodyContainerDyn::getFlexcompIndexFromPrefix(const char* prefix)
{
    int index = -1;
    for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
    {
        if (_xmlFlexcompInjections[i].prefix == prefix)
        {
            index = int(i);
            break;
        }
    }
    return index;
}

int CRigidBodyContainerDyn::getCompositeIndexFromInjectionId(int id)
{
    int index = -1;
    for (size_t i = 0; i < _xmlCompositeInjections.size(); i++)
    {
        if (_xmlCompositeInjections[i].injectionId == id)
        {
            index = int(i);
            break;
        }
    }
    return (index);
}

int CRigidBodyContainerDyn::addOrUpdateFlexcompInjection(CStackArray* inArguments, int flexcompIdToUpdate /*= -1*/, std::string* errString /*= nullptr*/, int scriptId /*= -1*/)
{
    int retVal = -1;
    int flexcompIndex = -1;
    if (flexcompIdToUpdate != -1)
    {
        flexcompIndex = getFlexcompIndexFromInjectionId(flexcompIdToUpdate);
        if (flexcompIndex < 0)
        {
            if (errString != nullptr)
                errString[0] = "invalid flexcomp id.";
        }
    }
    else
    {
        flexcompIndex = _xmlFlexcompInjections.size();
        SFlexcompInject flexcomp;
        _xmlFlexcompInjections.push_back(flexcomp);
    }

    if (flexcompIndex != -1)
    {
        SFlexcompInject* flexcomp = &_xmlFlexcompInjections[flexcompIndex];
        if ((inArguments->getSize() >= 1) && inArguments->isMap(0))
        { // we expect 1 argument: an info map
            CStackMap* map = inArguments->getMap(0);
            if ((map->isNumber("shapeHandle")) || (map->isString("element")) || (flexcompIdToUpdate != -1))
            {
                if ((map->isString("name") && map->isArray("count") && map->isString("type")) || (flexcompIdToUpdate != -1))
                {
                    std::string flexcompPrefix;
                    if (flexcompIdToUpdate == -1)
                    {
                        flexcomp->shapeHandle = -1;
                        if (map->isNumber("shapeHandle"))
                            flexcomp->shapeHandle = map->getInt("shapeHandle");
                        flexcomp->element = map->getString("element");
                        if ((flexcomp->element == "") && (flexcomp->shapeHandle == -1))
                            flexcomp->element = "worldbody";
                        if (flexcomp->shapeHandle != -1)
                            flexcomp->element = "";
                        flexcompPrefix = map->getString("name");
                    }
                    if ((flexcompIdToUpdate != -1) || (CRigidBodyContainerDyn::getFlexcompIndexFromPrefix(flexcompPrefix.c_str()) == -1))
                    {
                        if (flexcompPrefix != "")
                            flexcomp->prefix = flexcompPrefix;
                        CStackArray* arr = nullptr;
                        if (flexcompIdToUpdate == -1)
                        {
                            flexcomp->type = map->getString("type");

                            flexcomp->count[0] = 1;
                            flexcomp->count[1] = 1;
                            flexcomp->count[2] = 1;
                            arr = map->getArray("count");
                            for (size_t i = 0; i < std::min<size_t>(3, arr->getSize()); i++)
                                flexcomp->count[i] = size_t(arr->getInt(i));

                            flexcomp->spacing[0] = 0.01;
                            flexcomp->spacing[1] = 0.01;
                            flexcomp->spacing[2] = 0.01;
                        }

                        arr = map->getArray("spacing");
                        if (arr != nullptr)
                        {
                            for (size_t i = 0; i < std::min<size_t>(3, arr->getSize()); i++)
                                flexcomp->spacing[i] = arr->getDouble(i);
                        }
                        arr = map->getArray("pin");
                        if (arr != nullptr)
                            flexcomp->pinned = arr->getInts()[0];

                        if (flexcompIdToUpdate == -1)
                            flexcomp->radius = 0.001;
                        if (map->isNumber("radius"))
                            flexcomp->radius = map->getDouble("radius");
                        if (flexcompIdToUpdate == -1)
                            flexcomp->mass = 1.0;
                        if (map->isNumber("mass"))
                            flexcomp->mass = map->getDouble("mass");

                        if (flexcompIdToUpdate == -1)
                        {
                            flexcomp->pose = C7Vector::identityTransformation;
                            arr = map->getArray("pose");
                            if ((arr != nullptr) && (arr->getSize() == 7))
                            {
                                double p[7];
                                for (size_t i = 0; i < 7; i++)
                                    p[i] = arr->getDouble(i);
                                flexcomp->pose.setData(p, true);
                            }
                        }

                        if (map->isString("extraFlexcompXml"))
                            flexcomp->flexcompExtraXml = map->getString("extraFlexcompXml");
                        if (map->isString("extraXml"))
                            flexcomp->extraXml = map->getString("extraXml");

                        if (flexcomp->type == "grid")
                        {
                            if (flexcompIdToUpdate == -1)
                            {
                                if (map->isString("cbFunc"))
                                    flexcomp->cbFunc = map->getString("cbFunc");
                                flexcomp->cbScript = scriptId;
                                flexcomp->updateCnt = 0;
                                retVal = _nextInjectionId++;
                            }
                            else
                                retVal = flexcompIdToUpdate;
                        }
                        else
                        {
                            if (errString != nullptr)
                                errString[0] = "invalid flexcomp type.";
                        }
                    }
                    else
                    {
                        if (errString != nullptr)
                            errString[0] = "invalid prefix.";
                    }
                }
                else
                {
                    if (errString != nullptr)
                        errString[0] = "info map does not contain all required items.";
                }
            }
            else
            {
                if (errString != nullptr)
                    errString[0] = "info map should contain either shapeHandle(int) or element(string).";
            }
        }
        else
        {
            if (errString != nullptr)
                errString[0] = "not enough arguments or wrong arguments.";
        }
    }

    if (retVal != -1)
    {
        if (flexcompIdToUpdate == -1)
        {
            _xmlFlexcompInjections[_xmlFlexcompInjections.size() - 1].injectionId = retVal; // we actually added a new flexcomp. Set its id
            _dynWorld->_xmlInjectionChanged = true;                                         // a callback update is called already from within a _xmlInjectionChanged action!
        }
    }
    else
    { // error
        if (flexcompIdToUpdate == -1)
            _xmlFlexcompInjections.erase(_xmlFlexcompInjections.end() - 1); // we actually added a new flexcomp. Remove it again
    }

    return retVal;
}

int CRigidBodyContainerDyn::getFlexcompIndexFromInjectionId(int id)
{
    int index = -1;
    for (size_t i = 0; i < _xmlFlexcompInjections.size(); i++)
    {
        if (_xmlFlexcompInjections[i].injectionId == id)
        {
            index = int(i);
            break;
        }
    }
    return index;
}

int CRigidBodyContainerDyn::addOrUpdateGeneralInjection(CStackArray* inArguments, int injectionIdToUpdate /*= -1*/, std::string* errString /*= nullptr*/, int scriptId /*= -1*/)
{
    int retVal = -1;
    int injectionIndex = -1;
    if (injectionIdToUpdate != -1)
    {
        injectionIndex = getGeneralInjectionIndexFromInjectionId(injectionIdToUpdate);
        if (injectionIndex < 0)
        {
            if (errString != nullptr)
                errString[0] = "invalid injection id.";
        }
    }
    else
    {
        injectionIndex = _xmlGeneralInjections.size();
        SGeneralInject injection;
        _xmlGeneralInjections.push_back(injection);
    }

    if (injectionIndex != -1)
    {
        SGeneralInject* injection = &_xmlGeneralInjections[injectionIndex];
        if ((inArguments->getSize() >= 1) && inArguments->isMap(0))
        { // we expect 1 argument: an info map
            CStackMap* map = inArguments->getMap(0);
            if ((map->isNumber("shapeHandle")) || (map->isString("element")) || (injectionIdToUpdate != -1))
            {
                if (injectionIdToUpdate == -1)
                {
                    injection->shapeHandle = -1;
                    if (map->isNumber("shapeHandle"))
                        injection->shapeHandle = map->getInt("shapeHandle");
                    injection->element = map->getString("element");
                    if ((injection->element == "") && (injection->shapeHandle == -1))
                        injection->element = "worldbody";
                    if (injection->shapeHandle != -1)
                        injection->element = "";
                }

                if (map->isString("xml"))
                    injection->xml = map->getString("xml");

                if (injectionIdToUpdate == -1)
                {
                    if (map->isString("cbFunc"))
                        injection->cbFunc = map->getString("cbFunc");
                    injection->cbScript = scriptId;
                    injection->updateCnt = 0;
                    retVal = _nextInjectionId++;
                }
                else
                    retVal = injectionIdToUpdate;
            }
            else
            {
                if (errString != nullptr)
                    errString[0] = "info map should contain either shapeHandle(int) or element(string).";
            }
        }
        else
        {
            if (errString != nullptr)
                errString[0] = "not enough arguments or wrong arguments.";
        }
    }

    if (retVal != -1)
    {
        if (injectionIdToUpdate == -1)
        {
            _xmlGeneralInjections[_xmlGeneralInjections.size() - 1].injectionId = retVal; // we actually added a new injection. Set its id
            _dynWorld->_xmlInjectionChanged = true;                                       // a callback update is called already from within a _xmlInjectionChanged action!
        }
    }
    else
    { // error
        if (injectionIdToUpdate == -1)
            _xmlGeneralInjections.erase(_xmlGeneralInjections.end() - 1); // we actually added a new injection. Remove it again
    }

    return retVal;
}

int CRigidBodyContainerDyn::getGeneralInjectionIndexFromInjectionId(int id)
{
    int index = -1;
    for (size_t i = 0; i < _xmlGeneralInjections.size(); i++)
    {
        if (_xmlGeneralInjections[i].injectionId == id)
        {
            index = int(i);
            break;
        }
    }
    return index;
}
