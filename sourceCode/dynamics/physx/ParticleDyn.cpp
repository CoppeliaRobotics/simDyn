#include <ParticleDyn.h>
#include <RigidBodyContainerDyn.h>
#include <simLib/simLib.h>

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,double size,double massOverVolume,double killTime,float addColor[3]) : CParticleDyn_base(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(double parameters[18],int objectID)
{
    if (_initializationState>1)
        return(false);
    _initializationState=1;

    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,double linearFluidFrictionCoeff,double quadraticFluidFrictionCoeff,double linearAirFrictionCoeff,double quadraticAirFrictionCoeff)
{
    if (_initializationState==1)
    {
    }
}

void CParticleDyn::removeFromEngine()
{
    if (_initializationState==1)
        _initializationState=2;
}

void CParticleDyn::updatePosition()
{
    if (_initializationState==1)
    {
    }
}
