#pragma once

#include <simMath/3Vector.h>

class CParticleDyn_base
{
  public:
    CParticleDyn_base(const C3Vector& position, const C3Vector& velocity, int objType, sReal size, sReal massOverVolume, sReal killTime, float addColor[3]);

    virtual ~CParticleDyn_base();

    virtual bool addToEngineIfNeeded(sReal parameters[18], int objectID);
    virtual void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity, sReal linearFluidFrictionCoeff, sReal quadraticFluidFrictionCoeff, sReal linearAirFrictionCoeff, sReal quadraticAirFrictionCoeff);
    virtual void updatePosition();
    virtual void removeFromEngine();

    bool didTimeOut(sReal simulationTime);
    int getInitializationState();
    int getUniqueID();
    void setUniqueID(int id);
    bool getRenderData(sReal* pos, sReal* size, int* objType, float** additionalColor);

  protected:
    int _uniqueID;
    char _initializationState;       // 0=not initialized, 1=initialized, 2=desinitialized (to be killed)
    C3Vector _currentPosition;       // Not scaled!
    C3Vector _initialVelocityVector; // not scaled!
    int _objectType;
    sReal _size;
    sReal _massOverVolume;
    sReal _killTime;
    float _additionalColor[3];
};
