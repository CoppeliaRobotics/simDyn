#pragma once

#include <ParticleDyn_base.h>
#include <Newton.h>
#include <dMatrix.h>
#include <CustomJoint.h>
#include <CustomHinge.h>
#include <CustomSlider.h>
#include <CustomBallAndSocket.h>
#include <CustomHingeActuator.h>
#include <CustomSliderActuator.h>

class CParticleDyn : public CParticleDyn_base
{
  public:
    CParticleDyn(const C3Vector& position, const C3Vector& velocity, int objType, sReal size, sReal massOverVolume, sReal killTime, float addColor[3]);
    virtual ~CParticleDyn();

    bool addToEngineIfNeeded(sReal parameters[18], int objectID);
    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity, sReal linearFluidFrictionCoeff, sReal quadraticFluidFrictionCoeff, sReal linearAirFrictionCoeff, sReal quadraticAirFrictionCoeff);
    void updatePosition();
    void removeFromEngine();

    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    static void ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex);

  protected:
    NewtonBody* _newtonBody;
    C3Vector m_externForce;
    bool _ignoreGravity;
    sReal _particleMass;
    int _particleObjectID_withOffset;
    void* _newtonParticleUserData[5]; // particleObjectID,this,stat. friction,kin. friction, restitution
    sReal statFric, kinFric, rest;
};
