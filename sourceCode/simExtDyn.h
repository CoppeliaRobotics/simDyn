#ifndef SIMEXTDYN_H
#define SIMEXTDYN_H

#ifdef _WIN32
#define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif

#if defined (__linux) || defined (__APPLE__)
#define SIM_DLLEXPORT extern "C"
#endif

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);

SIM_DLLEXPORT char dynPlugin_startSimulation(int engine,int version,const float floatParams[20],const int intParams[20]);
SIM_DLLEXPORT void dynPlugin_endSimulation();
SIM_DLLEXPORT void dynPlugin_step(float timeStep,float simulationTime);
SIM_DLLEXPORT char dynPlugin_isDynamicContentAvailable();
SIM_DLLEXPORT void dynPlugin_serializeDynamicContent(const char* filenameAndPath,int bulletSerializationBuffer);
SIM_DLLEXPORT int dynPlugin_addParticleObject(int objectType,float size,float massOverVolume,const void* params,float lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission);
SIM_DLLEXPORT char dynPlugin_removeParticleObject(int objectHandle);
SIM_DLLEXPORT char dynPlugin_addParticleObjectItem(int objectHandle,const float* itemData,float simulationTime);
SIM_DLLEXPORT int dynPlugin_getParticleObjectOtherFloatsPerItem(int objectHandle);
SIM_DLLEXPORT float* dynPlugin_getContactPoints(int* count);
SIM_DLLEXPORT void** dynPlugin_getParticles(int index,int* particlesCount,int* objectType,float** cols);
SIM_DLLEXPORT char dynPlugin_getParticleData(const void* particle,float* pos,float* size,int* objectType,float** additionalColor);
SIM_DLLEXPORT char dynPlugin_getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float* contactInfo);
SIM_DLLEXPORT int dynPlugin_getDynamicStepDivider();
#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT float mujocoPlugin_computeInertia(int shapeHandle,float* relPos,float* relQuat,float* diagI);
#endif
#endif
