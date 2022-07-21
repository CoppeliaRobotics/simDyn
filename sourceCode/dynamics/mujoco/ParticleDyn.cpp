#include "ParticleDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]) : CParticleDyn_base(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(float parameters[18],int objectID)
{
    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
}

void CParticleDyn::removeFromEngine()
{
}

void CParticleDyn::updatePosition()
{
}
