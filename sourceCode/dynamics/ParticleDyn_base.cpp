#include "ParticleDyn_base.h"
#include "RigidBodyContainerDyn_base.h"
#include "4Vector.h"
#include "simLib.h"

CParticleDyn_base::CParticleDyn_base(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3])
{
    _initializationState=0;
    _currentPosition=position;
    _initialVelocityVector=velocity;
    _objectType=objType;
    _size=size;
    _massOverVolume=massOverVolume;
    _killTime=killTime;
    if (addColor!=nullptr)
    {
        _additionalColor[0]=addColor[0];
        _additionalColor[1]=addColor[1];
        _additionalColor[2]=addColor[2];
    }
}

CParticleDyn_base::~CParticleDyn_base()
{
}

bool CParticleDyn_base::addToEngineIfNeeded(float parameters[18],int objectID)
{
    return(false);
}

void CParticleDyn_base::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
}

void CParticleDyn_base::removeFromEngine()
{
}

void CParticleDyn_base::updatePosition()
{
}

bool CParticleDyn_base::didTimeOut(float simulationTime)
{
    return(simulationTime>_killTime);
}

int CParticleDyn_base::getInitializationState()
{
    return(_initializationState);
}

int CParticleDyn_base::getUniqueID()
{
    return(_uniqueID);
}

void CParticleDyn_base::setUniqueID(int id)
{
    _uniqueID=id;
}

bool CParticleDyn_base::getRenderData(float* pos,float* size,int* objType,float** additionalColor)
{
    if (_initializationState==1)
    {
        _currentPosition.getInternalData(pos);
        objType[0]=_objectType;
        size[0]=_size;
        additionalColor[0]=_additionalColor;
        return(true);
    }
    return(false);
}
