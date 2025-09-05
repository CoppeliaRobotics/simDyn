#pragma once

#include <simLib/simTypes.h>
#include <simLib/simExp.h>

SIM_DLLEXPORT int simInit(SSimInit*);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(SSimMsg*);

SIM_DLLEXPORT char dynPlugin_startSimulation_D(const double floatParams[20], const int intParams[20]);
SIM_DLLEXPORT void dynPlugin_endSimulation();
SIM_DLLEXPORT void dynPlugin_step_D(double timeStep, double simulationTime);
SIM_DLLEXPORT char dynPlugin_isDynamicContentAvailable();
SIM_DLLEXPORT void dynPlugin_serializeDynamicContent(const char* filenameAndPath, int bulletSerializationBuffer);
SIM_DLLEXPORT int dynPlugin_addParticleObject_D(int objectType, double size, double massOverVolume, const void* params, double lifeTime, int maxItemCount, const float* ambient, const float* diffuse, const float* specular, const float* emission);
SIM_DLLEXPORT char dynPlugin_removeParticleObject(int objectHandle);
SIM_DLLEXPORT char dynPlugin_addParticleObjectItem_D(int objectHandle, const double* itemData, double simulationTime);
SIM_DLLEXPORT int dynPlugin_getParticleObjectOtherFloatsPerItem(int objectHandle);
SIM_DLLEXPORT double* dynPlugin_getContactPoints_D(int* count);
SIM_DLLEXPORT void** dynPlugin_getParticles(int index, int* particlesCount, int* objectType, float** cols);
SIM_DLLEXPORT char dynPlugin_getParticleData_D(const void* particle, double* pos, double* size, int* objectType, float** additionalColor);
SIM_DLLEXPORT char dynPlugin_getContactForce_D(int dynamicPass, int objectHandle, int index, int objectHandles[2], double* contactInfo);
SIM_DLLEXPORT int dynPlugin_getDynamicStepDivider();
#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT double mujocoPlugin_computeInertia(int shapeHandle, double* relPos, double* relQuat, double* diagI);
SIM_DLLEXPORT double mujocoPlugin_computePMI(const double* vertices, int verticesL, const int* indices, int indicesL, double* relPos, double* relQuat, double* diagI);
SIM_DLLEXPORT char mujocoPlugin_generateMjcfFile();
#endif

#ifdef INCLUDE_BULLET_2_78_CODE
SIM_DLLEXPORT void dynPlugin_bullet278();
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
SIM_DLLEXPORT void dynPlugin_bullet283();
#endif
#ifdef INCLUDE_ODE_CODE
SIM_DLLEXPORT void dynPlugin_ode();
#endif
#ifdef INCLUDE_VORTEX_CODE
SIM_DLLEXPORT void dynPlugin_vortex();
#endif
#ifdef INCLUDE_NEWTON_CODE
SIM_DLLEXPORT void dynPlugin_newton();
#endif
#ifdef INCLUDE_MUJOCO_CODE
SIM_DLLEXPORT void dynPlugin_mujoco();
#endif
#ifdef INCLUDE_DRAKE_CODE
SIM_DLLEXPORT void dynPlugin_drake();
#endif
