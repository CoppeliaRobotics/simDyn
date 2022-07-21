#pragma once

#include "3Vector.h"

class CParticleDyn_base
{
public:
    CParticleDyn_base(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]);

    virtual ~CParticleDyn_base();

    virtual bool addToEngineIfNeeded(float parameters[18],int objectID);
    virtual void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff);
    virtual void updatePosition();
    virtual void removeFromEngine();

    bool didTimeOut(float simulationTime);
    int getInitializationState();
    int getUniqueID();
    void setUniqueID(int id);
    bool getRenderData(float* pos,float* size,int* objType,float** additionalColor);

protected:    
    int _uniqueID;
    char _initializationState; // 0=not initialized, 1=initialized, 2=desinitialized (to be killed)
    C3Vector _currentPosition; // Not scaled!
    C3Vector _initialVelocityVector; // not scaled!
    int _objectType;
    float _size;
    float _massOverVolume;
    float _killTime;
    float _additionalColor[3];
};
