#include "ParticleDyn.h"
#include "RigidBodyContainerDyn.h"
#include "NewtonConvertUtil.h"
#include "simLib.h"

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,sReal size,sReal massOverVolume,sReal killTime,float addColor[3]) : CParticleDyn_base(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(sReal parameters[18],int objectID)
{
    if (_initializationState!=0)
        return(_initializationState==1);
    _initializationState=1;

    CRigidBodyContainerDyn* rbc=(CRigidBodyContainerDyn*)(CRigidBodyContainerDyn::getDynWorld());
    NewtonWorld* const world=rbc->getWorld();

    C7Vector tr;
    tr.setIdentity();
    tr.X=_currentPosition;
    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(tr));

    NewtonCollision* const collision = NewtonCreateSphere(world,_size*0.5,0,nullptr);
    _newtonBody = NewtonCreateDynamicBody (world,collision,&matrix[0][0]);
    NewtonDestroyCollision (collision);
    
    _particleMass=_massOverVolume*(4.0*piValue*(_size*0.5)*(_size*0.5)*(_size*0.5)/3.0);
    _ignoreGravity=false;
    sReal I=2.0*(_size*0.5)*(_size*0.5)/5.0;
    C3Vector im(I,I,I);
    im*=_particleMass;
    NewtonBodySetMassMatrix(_newtonBody,_particleMass,im(0),im(1),im(2));

    // Set the user-data to that particle:
    _particleObjectID_withOffset=objectID+CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart();
    _newtonParticleUserData[0]=&_particleObjectID_withOffset;
    _newtonParticleUserData[1]=this;
    statFric=parameters[15];
    kinFric=parameters[16];
    rest=parameters[17];
    _newtonParticleUserData[2]=&statFric;
    _newtonParticleUserData[3]=&kinFric;
    _newtonParticleUserData[4]=&rest;
    NewtonBodySetUserData(_newtonBody, _newtonParticleUserData);

    // disable auto sleep at all time
    // Julio this should no be necessary, but does no hurt
    NewtonBodySetAutoSleep(_newtonBody, 0);

    m_externForce.clear();

//    // attach the CXGeomWrap* as user dat of the collsion shape, this is so that we can apply material propertes in the collision callacks
//    NewtonCollision* const collision = NewtonBodyGetCollision(_newtonBody);
//    NewtonCollisionSetUserData(collision, geomInfo);

    NewtonBodySetTransformCallback(_newtonBody, TransformCallback);
    NewtonBodySetForceAndTorqueCallback(_newtonBody, ApplyExtenalForceCallback);

    NewtonBodySetVelocity(_newtonBody, _initialVelocityVector.data);

// Julio: this could be an option, as continue collision make it slower.   
//    NewtonBodySetContinuousCollisionMode(_newtonBody,1); // fast moving
    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,sReal linearFluidFrictionCoeff,sReal quadraticFluidFrictionCoeff,sReal linearAirFrictionCoeff,sReal quadraticAirFrictionCoeff)
{
    bool isWaterButInAir=false;
    if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
    {
        sReal mass=_massOverVolume*((piValue*_size*_size*_size)/6.0);
        C3Vector f=gravity*-mass;
        bool reallyIgnoreGravity=true;
        if (_objectType&sim_particle_water)
        { // We ignore gravity only if we are in the water (z<0) (New since 27/6/2011):
            C3Vector pos;
            NewtonBodyGetPosition(_newtonBody,pos.data);
            if (pos(2)>=0.0)
            {
                reallyIgnoreGravity=false;
                isWaterButInAir=true;
            }
        }

        _ignoreGravity=reallyIgnoreGravity;
        if (reallyIgnoreGravity)
        {

        }
    }
    if ((linearFluidFrictionCoeff!=0.0)||(quadraticFluidFrictionCoeff!=0.0)||(linearAirFrictionCoeff!=0.0)||(quadraticAirFrictionCoeff!=0.0))
    { // New since 27/6/2011
        sReal lfc=linearFluidFrictionCoeff;
        sReal qfc=quadraticFluidFrictionCoeff;
        if (isWaterButInAir)
        {
            lfc=linearAirFrictionCoeff;
            qfc=quadraticAirFrictionCoeff;
        }
        C3Vector vVect;

        NewtonBodyGetVelocity(_newtonBody,vVect.data);

        sReal v=vVect.getLength();
        if (v!=0.0)
        {
            C3Vector nv(vVect.getNormalized()*-1.0);
            C3Vector f(nv*(v*lfc+v*v*qfc));
            m_externForce=f;
        }
    }
}

void CParticleDyn::removeFromEngine()
{
    if (_initializationState==1)
    {
        NewtonDestroyBody (_newtonBody);
        _initializationState=2;
    }
}

void CParticleDyn::updatePosition()
{
    if (_initializationState==1)
        NewtonBodyGetPosition(_newtonBody,_currentPosition.data);
}

void CParticleDyn::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
    //Julio: I am no sure if CoppeliaSim Collect the transforms after the update but iteration ov e bodies in a loop
}

void CParticleDyn::ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
{
    void** userData=(void**)NewtonBodyGetUserData(body);
    CParticleDyn* const me = (CParticleDyn*)userData[1];

    C3Vector gravity;
    gravity.clear();

    if (!me->_ignoreGravity)
    {
        _simGetGravity(gravity.data);
        gravity *= me->_particleMass;
    }


    gravity += me->m_externForce;
    NewtonBodyAddForce(body, &gravity.data[0]);
/*
// for testing codename system alignments
if (mass != 0.0) {
    dVector omega;
    NewtonBodyGetOmega (body, &omega[0]);
    dVector torque (dVector (0, 0, 1, 0) - omega.Scale (0.1));
    NewtonBodyAddTorque(body, &torque[0]);
}
*/
}
