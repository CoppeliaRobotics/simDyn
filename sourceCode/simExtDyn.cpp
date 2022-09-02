#include "simExtDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"
#include <iostream>
#include <cstdio>
#include "stackArray.h"
#include "stackMap.h"

#ifdef _WIN32
#include <direct.h>
#endif

#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif

static LIBRARY simLib;

#ifdef INCLUDE_MUJOCO_CODE
#define LUA_MUJOCOINJECTXML_COMMAND "simMujoco.injectXML"
void LUA_MUJOCOINJECTXML_CALLBACK(SScriptCallBack* p)
{
    int eng;
    simGetInt32Param(sim_intparam_dynamic_engine,&eng);
    if (eng==sim_physics_mujoco)
    {
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
            CRigidBodyContainerDyn::injectXml(xml.c_str(),element.c_str(),handle);
        }
        else
            simSetLastError(LUA_MUJOCOINJECTXML_COMMAND,"not enough arguments or wrong arguments.");

        CStackArray outArguments;
        outArguments.buildOntoStack(stack);
    }
    else
        simSetLastError(LUA_MUJOCOINJECTXML_COMMAND,"current engine is not Mujoco.");
}

#define LUA_MUJOCOCOMPOSITE_COMMAND "simMujoco.composite"
void LUA_MUJOCOCOMPOSITE_CALLBACK(SScriptCallBack* p)
{
    int eng;
    simGetInt32Param(sim_intparam_dynamic_engine,&eng);
    if (eng==sim_physics_mujoco)
    {
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
                            CRigidBodyContainerDyn::injectCompositeXml(xml.c_str(),shapeHandle,element.c_str(),prefix.c_str(),c,type.c_str(),respondableMask,grow);
                        else
                            simSetLastError(LUA_MUJOCOCOMPOSITE_COMMAND,"invalid composite type.");
                    }
                    else
                        simSetLastError(LUA_MUJOCOCOMPOSITE_COMMAND,"invalid prefix.");
                }
                else
                    simSetLastError(LUA_MUJOCOCOMPOSITE_COMMAND,"info map does not contain all required items.");
            }
            else
                simSetLastError(LUA_MUJOCOCOMPOSITE_COMMAND,"info map should contain either shapeHandle(int) or element(string).");
        }
        else
            simSetLastError(LUA_MUJOCOCOMPOSITE_COMMAND,"not enough arguments or wrong arguments.");

        CStackArray outArguments;
        outArguments.buildOntoStack(stack);
    }
    else
        simSetLastError(LUA_MUJOCOINJECTXML_COMMAND,"current engine is not Mujoco.");
}

#define LUA_MUJOCOGETCOMPOSITEINFO_COMMAND "simMujoco.getCompositeInfo"
void LUA_MUJOCOGETCOMPOSITEINFO_CALLBACK(SScriptCallBack* p)
{
    int eng;
    simGetInt32Param(sim_intparam_dynamic_engine,&eng);
    if (eng==sim_physics_mujoco)
    {
        int stack=p->stackID;
        std::vector<double> info;
        std::string type;
        int count[3]={0,0,0};
        CStackArray inArguments;
        inArguments.buildFromStack(stack);
        if ( (inArguments.getSize()>=2)&&inArguments.isString(0)&&inArguments.isNumber(1) )
        { // we expect 2 argument: a prefix string and an int
            std::string prefix(inArguments.getString(0));
            int what=inArguments.getInt(1);
            CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
            if (dynWorld!=nullptr)
                type=dynWorld->getCompositeInfo(prefix.c_str(),what,info,count);
        }
        else
            simSetLastError(LUA_MUJOCOGETCOMPOSITEINFO_COMMAND,"not enough arguments or wrong arguments.");

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
        simSetLastError(LUA_MUJOCOINJECTXML_COMMAND,"current engine is not Mujoco.");
}
#endif


SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    char curDirAndFile[1024];
 #ifdef _WIN32
    _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
    std::string currentDirAndPath(curDirAndFile);

    std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/coppeliaSim.dll";
 #elif defined (__linux)
     temp+="/libcoppeliaSim.so";
 #elif defined (__APPLE__)
     temp+="/libcoppeliaSim.dylib";
 #endif /* __linux || __APPLE__ */

     simLib=loadSimLibrary(temp.c_str());
     if (simLib==nullptr)
    {
         printf("simExt%s: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n",LIBRARY_NAME); // cannot use simAddLog here.
         return(0);
    }
     if (getSimProcAddresses(simLib)==0)
    {
         printf("simExt%s: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n",LIBRARY_NAME); // cannot use simAddLog here.
         unloadSimLibrary(simLib);
         return(0);
    }

#ifdef INCLUDE_MUJOCO_CODE
    simRegisterScriptVariable("simMujoco","require('simMujoco')",0);

    simRegisterScriptCallbackFunction("simMujoco.injectXML@Mujoco","simMujoco.injectXML(string xml,string element)",LUA_MUJOCOINJECTXML_CALLBACK);
    simRegisterScriptCallbackFunction("simMujoco.composite@Mujoco","simMujoco.composite(string xml,map info)",LUA_MUJOCOCOMPOSITE_CALLBACK);
    simRegisterScriptCallbackFunction("simMujoco.getCompositeInfo@Mujoco","simMujoco.getCompositeInfo(string prefix,int what)",LUA_MUJOCOGETCOMPOSITEINFO_CALLBACK);
#endif

    return(DYNAMICS_PLUGIN_VERSION);
}

SIM_DLLEXPORT void simEnd()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    return(nullptr);
}

SIM_DLLEXPORT char dynPlugin_startSimulation(int engine,int version,const float floatParams[20],const int intParams[20])
{
    char retVal=0;
#ifdef INCLUDE_BULLET_2_78_CODE
    if ( (engine==sim_physics_bullet)&&(version==0) )
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
    if ( (engine==sim_physics_bullet)&&(version==283) )
#endif
#ifdef INCLUDE_ODE_CODE
    if (engine==sim_physics_ode)
#endif
#ifdef INCLUDE_NEWTON_CODE
    if (engine==sim_physics_newton)
#endif
#ifdef INCLUDE_VORTEX_CODE
    if (engine==sim_physics_vortex)
#endif
#ifdef INCLUDE_MUJOCO_CODE
    if (engine==sim_physics_mujoco)
#endif
    {
        simAddLog(LIBRARY_NAME,sim_verbosity_infos,"initializing the physics engine...");
        CRigidBodyContainerDyn* dynWorld=new CRigidBodyContainerDyn();
        CRigidBodyContainerDyn::setDynWorld(dynWorld);
        std::string err(dynWorld->init(floatParams,intParams));
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
    }
    return(retVal);
}

SIM_DLLEXPORT void dynPlugin_endSimulation()
{
    delete CRigidBodyContainerDyn::getDynWorld();
    CRigidBodyContainerDyn::setDynWorld(nullptr);
}

SIM_DLLEXPORT void dynPlugin_step(float timeStep,float simulationTime)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        dynWorld->handleDynamics(timeStep,simulationTime);
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

SIM_DLLEXPORT int dynPlugin_addParticleObject(int objectType,float size,float massOverVolume,const void* params,float lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        CParticleObject_base* it=new CParticleObject_base(objectType,size,massOverVolume,params,lifeTime,maxItemCount);
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

SIM_DLLEXPORT char dynPlugin_addParticleObjectItem(int objectHandle,const float* itemData,float simulationTime)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
    {
        CParticleObject_base* it=dynWorld->getParticleCont()->getObject(objectHandle,false);
        if (it==nullptr)
            return(false); // error
        it->addParticle(simulationTime,itemData);
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

SIM_DLLEXPORT float* dynPlugin_getContactPoints(int* count)
{
    float* retVal=nullptr;
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    count[0]=0;
    if (dynWorld!=nullptr)
        retVal=dynWorld->getContactPoints(count);
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

SIM_DLLEXPORT char dynPlugin_getParticleData(const void* particle,float* pos,float* size,int* objectType,float** additionalColor)
{
    if (particle==nullptr)
        return(0);
    return(((CParticleDyn*)particle)->getRenderData(pos,size,objectType,additionalColor));
}

SIM_DLLEXPORT char dynPlugin_getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float* contactInfo)
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        return(dynWorld->getContactForce(dynamicPass,objectHandle,index,objectHandles,contactInfo));
    return(false);
}

SIM_DLLEXPORT int dynPlugin_getDynamicStepDivider()
{
    CRigidBodyContainerDyn* dynWorld=CRigidBodyContainerDyn::getDynWorld();
    if (dynWorld!=nullptr)
        return(dynWorld->getDynamicsCalculationPasses());
    return(0);
}

#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT float mujocoPlugin_computeInertia(int shapeHandle,float* relPos,float* relQuat,float* diagI)
{ // returns the diagonal massless inertia (and mass, for a density of 1000)
    C7Vector tr;
    C3Vector diag;
    float mass=CRigidBodyContainerDyn::computeInertia(shapeHandle,tr,diag);
    tr.X.getInternalData(relPos);
    tr.Q.getInternalData(relQuat);
    diag.getInternalData(diagI);
    return(mass);
}
#endif

