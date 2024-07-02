#include <simDyn.h>
#include <RigidBodyContainerDyn.h>
#include <simLib/simLib.h>
#include <iostream>
#include <cstdio>
#include <simStack/stackArray.h>
#include <simStack/stackMap.h>

static LIBRARY simLib;

#ifdef INCLUDE_MUJOCO_CODE
void LUA_MUJOCOREMOVEXML_CALLBACK(SScriptCallBack* p)
{
    if (simGetSimulationState()!=sim_simulation_stopped)
    {
        int eng;
        simGetInt32Param(sim_intparam_dynamic_engine,&eng);
        if (eng==sim_physics_mujoco)
        {
            if (CRigidBodyContainerDyn::getDynWorld() != nullptr)
            {
                int stack=p->stackID;
                CStackArray inArguments;
                inArguments.buildFromStack(stack);
                if ( (inArguments.getSize()>=1)&&inArguments.isNumber(0) )
                { // we expect 1 arguments: the injection ID
                    int injectionId=inArguments.getInt(0);
                    if (!CRigidBodyContainerDyn::removeInjection(injectionId))
                        simSetLastError(nullptr,"invalid injection ID.");
                }
                else
                    simSetLastError(nullptr,"not enough arguments or wrong arguments.");

                CStackArray outArguments;
                outArguments.buildOntoStack(stack);
            }
            else
                simSetLastError(nullptr,"engine not initialized.");
        }
        else
            simSetLastError(nullptr,"current engine is not MuJoCo.");
    }
    else
        simSetLastError(nullptr,"simulation is not yet running.");
}

void LUA_MUJOCOINJECTXML_CALLBACK(SScriptCallBack* p)
{
    if (simGetSimulationState()!=sim_simulation_stopped)
    {
        int eng;
        simGetInt32Param(sim_intparam_dynamic_engine,&eng);
        if (eng==sim_physics_mujoco)
        {
            if (CRigidBodyContainerDyn::getDynWorld() != nullptr)
            {
                int injectionId=-1;
                int stack=p->stackID;
                CStackArray inArguments;
                inArguments.buildFromStack(stack);
                if ( (inArguments.getSize()>=2)&&inArguments.isString(0)&&(inArguments.isString(1)||inArguments.isNumber(1)) )
                { // we expect 2 arguments: an xml string and an element name or objectHandle
                    std::string xml(inArguments.getString(0));
                    std::string element;
                    int handle=-1;
                    if (inArguments.isString(1))
                        element=inArguments.getString(1);
                    else
                        handle=inArguments.getInt(1);
                    std::string cbFunc;
                    std::string cbId;
                    int cbScript=p->scriptID;
                    if ( (inArguments.getSize()>2)&&inArguments.isMap(2) )
                    {
                        CStackMap* map=inArguments.getMap(2);
                        if (map->isString("cbFunc"))
                            cbFunc=map->getString("cbFunc");
                        if (map->isString("cbId"))
                            cbId=map->getString("cbId");
                    }
                    injectionId=CRigidBodyContainerDyn::injectXml(xml.c_str(),element.c_str(),handle,cbFunc.c_str(),cbScript,cbId.c_str());
                }
                else
                    simSetLastError(nullptr,"not enough arguments or wrong arguments.");

                CStackArray outArguments;
                outArguments.pushInt(injectionId);
                outArguments.buildOntoStack(stack);
            }
            else
                simSetLastError(nullptr,"engine not initialized.");
        }
        else
            simSetLastError(nullptr,"current engine is not MuJoCo.");
    }
    else
        simSetLastError(nullptr,"simulation is not yet running.");
}

void LUA_MUJOCOCOMPOSITE_CALLBACK(SScriptCallBack* p)
{
    if (simGetSimulationState()!=sim_simulation_stopped)
    {
        int eng;
        simGetInt32Param(sim_intparam_dynamic_engine,&eng);
        if (eng==sim_physics_mujoco)
        {
            int injectionId=-1;
            int stack=p->stackID;
            CStackArray inArguments;
            inArguments.buildFromStack(stack);
            if ( (inArguments.getSize()>=2)&&inArguments.isString(0)&&inArguments.isMap(1) )
            { // we expect 2 arguments: an xml string and an info map
                std::string xml(inArguments.getString(0));
                CStackMap* map=inArguments.getMap(1);
                if ( (map->isNumber("shapeHandle"))||(map->isString("element")) )
                {
                    if ( (map->isString("prefix"))&&(map->isArray("count"))&&(map->isString("type")) )
                    {
                        if (CRigidBodyContainerDyn::getDynWorld() != nullptr)
                        {
                            int shapeHandle=map->getInt("shapeHandle");
                            std::string element(map->getString("element"));
                            std::string prefix(map->getString("prefix"));
                            if (CRigidBodyContainerDyn::getCompositeIndexFromPrefix(prefix.c_str())==-1)
                            {
                                std::string type(map->getString("type"));
                                int respondableMask=0xffff;
                                if ( (type=="box")||(type=="cylinder")||(type=="ellipsoide") )
                                     respondableMask=0xff00;   // do not collide with other composite elements
                                if (map->isNumber("respondableMask"))
                                    respondableMask=map->getInt("respondableMask");
                                double grow=0.0;
                                if (map->isNumber("grow"))
                                    grow=map->getDouble("grow");
                                size_t c[3]={1,1,1};
                                CStackArray* arr=map->getArray("count");
                                for (size_t i=0;i<std::min<size_t>(3,arr->getSize());i++)
                                    c[i]=size_t(arr->getInt(i));
                                // We do not support particles via composites, since they can't be named, thus, they can't be identified later on as particles
                                if ( /*(type=="particle")||*/(type=="grid")||(type=="rope")||(type=="loop")||(type=="cloth")||(type=="box")||(type=="cylinder")||(type=="ellipsoid") )
                                {
                                    std::string cbFunc;
                                    if (map->isString("cbFunc"))
                                        cbFunc=map->getString("cbFunc");
                                    int cbScript=p->scriptID;
                                    injectionId=CRigidBodyContainerDyn::injectCompositeXml(xml.c_str(),shapeHandle,element.c_str(),prefix.c_str(),c,type.c_str(),respondableMask,grow,cbFunc.c_str(),cbScript);
                                }
                                else
                                    simSetLastError(nullptr,"invalid composite type.");
                            }
                            else
                                simSetLastError(nullptr,"invalid prefix.");
                        }
                        else
                            simSetLastError(nullptr,"engine not initialized.");
                    }
                    else
                        simSetLastError(nullptr,"info map does not contain all required items.");
                }
                else
                    simSetLastError(nullptr,"info map should contain either shapeHandle(int) or element(string).");
            }
            else
                simSetLastError(nullptr,"not enough arguments or wrong arguments.");

            CStackArray outArguments;
            outArguments.pushInt(injectionId);
            outArguments.buildOntoStack(stack);
        }
        else
            simSetLastError(nullptr,"current engine is not MuJoCo.");
    }
    else
        simSetLastError(nullptr,"simulation is not yet running.");
}

void LUA_MUJOCOGETCOMPOSITEINFO_CALLBACK(SScriptCallBack* p)
{
    if (simGetSimulationState()!=sim_simulation_stopped)
    {
        int eng;
        simGetInt32Param(sim_intparam_dynamic_engine,&eng);
        if (eng==sim_physics_mujoco)
        {
            if (CRigidBodyContainerDyn::getDynWorld() != nullptr)
            {
                int stack=p->stackID;
                std::vector<double> info;
                std::string type;
                int count[3]={0,0,0};
                CStackArray inArguments;
                inArguments.buildFromStack(stack);
                if ( (inArguments.getSize()>=2)&&(inArguments.isNumber(0)||inArguments.isString(0))&&inArguments.isNumber(1) )
                { // we expect 2 argument: a prefix string and an int
                    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
                    if (dynWorld!=nullptr)
                    {
                        int compIndex=-1;
                        if (inArguments.isNumber(0))
                            compIndex=dynWorld->getCompositeIndexFromInjectionId(inArguments.getInt(0));
                        else
                            compIndex=dynWorld->getCompositeIndexFromPrefix(inArguments.getString(0).c_str()); // for backward compatibility
                        int what=inArguments.getInt(1);
                        type=dynWorld->getCompositeInfo(compIndex,what,info,count);
                    }
                }
                else
                    simSetLastError(nullptr,"not enough arguments or wrong arguments.");

                CStackArray outArguments;
                CStackMap* map=new CStackMap();
                CStackArray* infoArray=new CStackArray();
                if (info.size()>0)
                    infoArray->setDoubleArray(&info[0],info.size());
                map->setArray("info",infoArray);
                map->setString("type",type);
                CStackArray* countArray=new CStackArray();
                for (size_t i=0;i<3;i++)
                    countArray->pushInt(count[i]);
                map->setArray("count",countArray);
                outArguments.pushMap(map);
                outArguments.buildOntoStack(stack);
            }
            else
                simSetLastError(nullptr,"engine not initialized.");
        }
        else
            simSetLastError(nullptr,"current engine is not MuJoCo.");
    }
    else
        simSetLastError(nullptr,"simulation is not yet running.");
}
#endif


SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib=loadSimLibrary(info->coppeliaSimLibPath);
     if (simLib==nullptr)
    {
         simAddLog(info->pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
         return(0);
    }
     if (getSimProcAddresses(simLib)==0)
    {
         simAddLog(info->pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
         unloadSimLibrary(simLib);
         return(0);
    }

#ifdef INCLUDE_MUJOCO_CODE
    simRegisterScriptCallbackFunction("removeXML",nullptr,LUA_MUJOCOREMOVEXML_CALLBACK);
    simRegisterScriptCallbackFunction("_injectXML",nullptr,LUA_MUJOCOINJECTXML_CALLBACK);
    simRegisterScriptCallbackFunction("_composite",nullptr,LUA_MUJOCOCOMPOSITE_CALLBACK);
    simRegisterScriptCallbackFunction("getCompositeInfo",nullptr,LUA_MUJOCOGETCOMPOSITEINFO_CALLBACK);
#endif

    return(DYNAMICS_PLUGIN_VERSION);
}

SIM_DLLEXPORT void simCleanup()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void simMsg(SSimMsg*)
{
}

SIM_DLLEXPORT char dynPlugin_startSimulation_D(const double floatParams[20],const int intParams[20])
{
    char retVal=0;
    simAddLog(LIBRARY_NAME,sim_verbosity_infos,"initializing the physics engine...");
    CRigidBodyContainerDyn* dynWorld=new CRigidBodyContainerDyn();
    CRigidBodyContainerDyn::setDynWorld(dynWorld);
#ifdef SIM_MATH_DOUBLE
    std::string err(dynWorld->init(floatParams,intParams));
#else
    sReal fParams[20];
    for (size_t i=0;i<20;i++)
        fParams[i]=(sReal)floatParams[i];
    std::string err(dynWorld->init(fParams,intParams));
#endif
    std::string tmp("engine: ");
    tmp+=dynWorld->getEngineInfo();
    tmp+=", plugin version: ";
    tmp+=std::to_string(DYNAMICS_PLUGIN_VERSION);
    simAddLog(LIBRARY_NAME,sim_verbosity_infos,tmp.c_str());
    if (err.size()!=0)
        simAddLog(LIBRARY_NAME,sim_verbosity_errors,err.c_str());
    else
        simAddLog(LIBRARY_NAME,sim_verbosity_infos,"initialization successful.");
    retVal=1;
    return(retVal);
}

SIM_DLLEXPORT void dynPlugin_endSimulation()
{
    delete CRigidBodyContainerDyn::getDynWorld();
    CRigidBodyContainerDyn::setDynWorld(nullptr);
}

SIM_DLLEXPORT void dynPlugin_step_D(double timeStep,double simulationTime)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        dynWorld->handleDynamics((sReal)timeStep,(sReal)simulationTime);
}

SIM_DLLEXPORT char dynPlugin_isDynamicContentAvailable()
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        return(dynWorld->isDynamicContentAvailable());
    return(0);
}

SIM_DLLEXPORT void dynPlugin_serializeDynamicContent(const char* filenameAndPath,int bulletSerializationBuffer)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        dynWorld->serializeDynamicContent(filenameAndPath,bulletSerializationBuffer);
}

SIM_DLLEXPORT int dynPlugin_addParticleObject_D(int objectType,double size,double massOverVolume,const void* params,double lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        CParticleObject_base* it=new CParticleObject_base(objectType,(sReal)size,(sReal)massOverVolume,params,(sReal)lifeTime,maxItemCount);
        for (int i=0;i<9;i++)
            it->color[i]=0.25f;
        for (int i=9;i<12;i++)
            it->color[i]=0.0f;
        for (int i=0;i<3;i++)
        {
            if (ambient!=nullptr)
                it->color[0+i]=ambient[i];
            if (specular!=nullptr)
                it->color[6+i]=specular[i];
            if (emission!=nullptr)
                it->color[9+i]=emission[i];
        }
        return(dynWorld->getParticleCont()->addObject(it));
    }
    return(-1); // error
}

SIM_DLLEXPORT char dynPlugin_removeParticleObject(int objectHandle)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        if (objectHandle==sim_handle_all)
            dynWorld->getParticleCont()->removeAllObjects();
        else
        {
            CParticleObject_base* it=dynWorld->getParticleCont()->getObject(objectHandle,false);
            if (it==nullptr)
                return(false); // error
            dynWorld->getParticleCont()->removeObject(objectHandle);
        }
        return(true);
    }
    return(false); // error
}

SIM_DLLEXPORT char dynPlugin_addParticleObjectItem_D(int objectHandle,const double* itemData,double simulationTime)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        CParticleObject_base* it=dynWorld->getParticleCont()->getObject(objectHandle,false);
        if (it==nullptr)
            return(false); // error
#ifdef SIM_MATH_DOUBLE
        it->addParticle(simulationTime,itemData);
#else
        sReal iData[20];
        sReal* _iData=nullptr;
        if (itemData!=nullptr)
        {
            for (size_t i=0;i<20;i++)
                iData[i]=(sReal)itemData[i];
            _iData=iData;
        }
        it->addParticle((sReal)simulationTime,_iData);
#endif
#ifdef INCLUDE_MUJOCO_CODE
        dynWorld->particlesAdded();
#endif
        return(true);
    }
    return(false); // error
}

SIM_DLLEXPORT int dynPlugin_getParticleObjectOtherFloatsPerItem(int objectHandle)
{
    int retVal=0;
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        CParticleObject_base* it=dynWorld->getParticleCont()->getObject(objectHandle,false);
        if (it!=nullptr)
            retVal=it->getOtherFloatsPerItem();
        else if (objectHandle==-131183)
            retVal=61855195;
    }
    return(retVal);
}

SIM_DLLEXPORT double* dynPlugin_getContactPoints_D(int* count)
{
    double* retVal=nullptr;
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    count[0]=0;
    if (dynWorld!=nullptr)
    {
#ifdef SIM_MATH_DOUBLE
        retVal=dynWorld->getContactPoints(count);
#else
        static std::vector<double> pp;
        sReal* p=dynWorld->getContactPoints(count);
        pp.resize(count[0]*3);
        for (int i=0;i<count[0]*3;i++)
            pp[i]=(double)p[i];
        retVal=pp.data();
#endif
    }
    return(retVal);
}

SIM_DLLEXPORT void** dynPlugin_getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    void** retVal=nullptr;
    if (dynWorld!=nullptr)
        retVal=dynWorld->getParticleCont()->getParticles(index,particlesCount,objectType,cols);
    else
        particlesCount[0]=-1;
    return(retVal);
}

SIM_DLLEXPORT char dynPlugin_getParticleData_D(const void* particle,double* pos,double* size,int* objectType,float** additionalColor)
{
    if (particle==nullptr)
        return(0);
    char retVal=0;
#ifdef SIM_MATH_DOUBLE
    retVal=((CParticleDyn*)particle)->getRenderData(pos,size,objectType,additionalColor);
#else
    sReal fpos[3];
    sReal s;
    retVal=((CParticleDyn*)particle)->getRenderData(fpos,&s,objectType,additionalColor);
    pos[0]=(double)fpos[0];
    pos[1]=(double)fpos[1];
    pos[2]=(double)fpos[2];
    size[0]=(double)s;
#endif
    return(retVal);
}

SIM_DLLEXPORT char dynPlugin_getContactForce_D(int dynamicPass,int objectHandle,int index,int objectHandles[2],double* contactInfo)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    char retVal=0;
    if (dynWorld!=nullptr)
    {
#ifdef SIM_MATH_DOUBLE
        retVal=dynWorld->getContactForce(dynamicPass,objectHandle,index,objectHandles,contactInfo);
#else
        sReal ci[9];
        retVal=dynWorld->getContactForce(dynamicPass,objectHandle,index,objectHandles,ci);
        size_t cnt=6;
        if ((index&sim_handleflag_extended)!=0)
            cnt=9;
        for (size_t i=0;i<cnt;i++)
            contactInfo[i]=(double)ci[i];
#endif
    }
    return(retVal);
}

SIM_DLLEXPORT int dynPlugin_getDynamicStepDivider()
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        return(dynWorld->getDynamicsCalculationPasses());
    return(0);
}

#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT double mujocoPlugin_computeInertia(int shapeHandle,double* relPos,double* relQuat,double* diagI)
{ // returns the diagonal massless inertia (and mass, for a density of 1000)
    C7Vector tr;
    C3Vector diag;
    double mass=CRigidBodyContainerDyn::computeInertia(shapeHandle,tr,diag);
    tr.X.getData(relPos);
    tr.Q.getData(relQuat);
    diag.getData(diagI);
    return(mass);
}

SIM_DLLEXPORT double mujocoPlugin_computePMI(const double* vertices,int verticesL,const int* indices,int indicesL,double* relPos,double* relQuat,double* diagI)
{ // returns the diagonal massless inertia (and mass, for a density of 1000)
    C7Vector tr;
    C3Vector diag;
    double mass=CRigidBodyContainerDyn::computePMI(vertices,verticesL,indices,indicesL,tr,diag);
    tr.X.getData(relPos);
    tr.Q.getData(relQuat);
    diag.getData(diagI);
    return(mass);
}

#endif

#ifdef INCLUDE_BULLET_2_78_CODE
SIM_DLLEXPORT void dynPlugin_bullet278(){}
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
SIM_DLLEXPORT void dynPlugin_bullet283(){}
#endif
#ifdef INCLUDE_ODE_CODE
SIM_DLLEXPORT void dynPlugin_ode(){}
#endif
#ifdef INCLUDE_VORTEX_CODE
SIM_DLLEXPORT void dynPlugin_vortex(){}
#endif
#ifdef INCLUDE_NEWTON_CODE
SIM_DLLEXPORT void dynPlugin_newton(){}
#endif
#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT void dynPlugin_mujoco(){}
#endif
#ifdef INCLUDE_PHYSX_CODE
SIM_DLLEXPORT void dynPlugin_physx(){}
#endif
