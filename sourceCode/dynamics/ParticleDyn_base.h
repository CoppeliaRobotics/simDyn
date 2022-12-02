#pragma once

#include "3Vector.h"

class CParticleDyn_base
{
public:
    CParticleDyn_base(const C3Vector& position,const C3Vector& velocity,int objType,double size,double massOverVolume,double killTime,float addColor[3]);

    virtual ~CParticleDyn_base();

    virtual bool addToEngineIfNeeded(double parameters[18],int objectID);
    virtual void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,double linearFluidFrictionCoeff,double quadraticFluidFrictionCoeff,double linearAirFrictionCoeff,double quadraticAirFrictionCoeff);
    virtual void updatePosition();
    virtual void removeFromEngine();

    bool didTimeOut(double simulationTime);
    int getInitializationState();
    int getUniqueID();
    void setUniqueID(int id);
    bool getRenderData(double* pos,double* size,int* objType,float** additionalColor);

protected:    
    int _uniqueID;
    char _initializationState; // 0=not initialized, 1=initialized, 2=desinitialized (to be killed)
    C3Vector _currentPosition; // Not scaled!
    C3Vector _initialVelocityVector; // not scaled!
    int _objectType;
    double _size;
    double _massOverVolume;
    double _killTime;
    float _additionalColor[3];
};
