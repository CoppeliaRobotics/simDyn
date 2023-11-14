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
{ // return value indicates if there are particles that need to be simulated
    if (_initializationState!=0)
        return(_initializationState==1);
    _initializationState=1;

    CRigidBodyContainerDyn* rbc=(CRigidBodyContainerDyn*)(CRigidBodyContainerDyn::getDynWorld());
    dWorldID odeWorld=rbc->getWorld();
    dSpaceID space=rbc->getOdeSpace();

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    double massScaling=CRigidBodyContainerDyn::getDynWorld()->getMassScalingFactorDyn();
    double masslessInertiaScaling=CRigidBodyContainerDyn::getDynWorld()->getMasslessInertiaScalingFactorDyn();
    double velScaling=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();

    double mass=_massOverVolume*(4.0*piValue*(_size*0.5)*(_size*0.5)*(_size*0.5)/3.0)*massScaling; // ********** SCALING

    dMass m;
    dMassSetZero(&m);
    double I=2.0*(_size*0.5)*(_size*0.5)/5.0;
    C3Vector im(I,I,I);
    im*=masslessInertiaScaling; // ********** SCALING
    dMassSetParameters(&m,mass,0.0,0.0,0.0,im(0)*mass,im(1)*mass,im(2)*mass,0.0,0.0,0.0);

    _odeRigidBody=dBodyCreate(odeWorld);
    dBodySetMass(_odeRigidBody,&m);

    dBodySetPosition(_odeRigidBody,_currentPosition(0)*linScaling,_currentPosition(1)*linScaling,_currentPosition(2)*linScaling); // ********** SCALING
    C4Vector q;
    q.setIdentity();
    dQuaternion dQ;
    dQ[0]=q.data[0];
    dQ[1]=q.data[1];
    dQ[2]=q.data[2];
    dQ[3]=q.data[3];
    dBodySetQuaternion(_odeRigidBody,dQ);

    dBodySetData(_odeRigidBody,(void*)(CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart()+objectID));

    _odeGeom=dCreateSphere(space,_size*linScaling/2.0);  // ********** SCALING
    dGeomSetBody(_odeGeom,_odeRigidBody);

    // For now, we disable the auto-disable functionality (because there are problems when removing a kinematic object during simulation(e.g. removing the floor, nothing falls)):
    dBodySetAutoDisableFlag(_odeRigidBody,0);

    dBodySetLinearVel(_odeRigidBody,_initialVelocityVector(0)*velScaling,_initialVelocityVector(1)*velScaling,_initialVelocityVector(2)*velScaling); // ********** SCALING
    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,double linearFluidFrictionCoeff,double quadraticFluidFrictionCoeff,double linearAirFrictionCoeff,double quadraticAirFrictionCoeff)
{
    bool isWaterButInAir=false;
    if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
    {
        double mass=_massOverVolume*((piValue*_size*_size*_size)/6.0)*CRigidBodyContainerDyn::getDynWorld()->getMassScalingFactorDyn(); // ********** SCALING
        C3Vector f=gravity*-mass*CRigidBodyContainerDyn::getDynWorld()->getGravityScalingFactorDyn(); // ********** SCALING
        bool reallyIgnoreGravity=true;
        if (_objectType&sim_particle_water)
        { // We ignore gravity only if we are in the water (z<0) (New since 27/6/2011):
            double ps=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING
            const dReal* pos=dBodyGetPosition(_odeRigidBody);
            if (pos[2]/ps>=0.0) // ********** SCALING (not really needed here!)
            {
                reallyIgnoreGravity=false;
                isWaterButInAir=true;
            }
        }

        if (reallyIgnoreGravity)
            dBodyAddForce(_odeRigidBody,f(0),f(1),f(2));
    }
    if ((linearFluidFrictionCoeff!=0.0)||(quadraticFluidFrictionCoeff!=0.0)||(linearAirFrictionCoeff!=0.0)||(quadraticAirFrictionCoeff!=0.0))
    { // New since 27/6/2011
        double lfc=linearFluidFrictionCoeff;
        double qfc=quadraticFluidFrictionCoeff;
        if (isWaterButInAir)
        {
            lfc=linearAirFrictionCoeff;
            qfc=quadraticAirFrictionCoeff;
        }
        C3Vector vVect;

        const dReal* lvel=dBodyGetLinearVel(_odeRigidBody);
        vVect.setData((double)lvel[0],(double)lvel[1],(double)lvel[2]);

        double v=vVect.getLength();
        if (v!=0.0)
        {
            double vs=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn(); // ********** SCALING
            double fs=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn(); // ********** SCALING
            v/=vs; // ********** SCALING
            C3Vector nv(vVect.getNormalized()*-1.0);
            C3Vector f(nv*(v*lfc+v*v*qfc));
            f*=fs; // ********** SCALING

            dBodyAddForce(_odeRigidBody,f(0),f(1),f(2));
        }
    }
}

void CParticleDyn::removeFromEngine()
{
    if (_initializationState==1)
    {
        dGeomDestroy(_odeGeom);
        dBodyDestroy(_odeRigidBody);
        _initializationState=2;
    }
}

void CParticleDyn::updatePosition()
{
    if (_initializationState==1)
    {
        double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();

        const dReal* pos=dBodyGetPosition(_odeRigidBody);
        _currentPosition(0)=pos[0]/linScaling; // ********** SCALING
        _currentPosition(1)=pos[1]/linScaling; // ********** SCALING
        _currentPosition(2)=pos[2]/linScaling; // ********** SCALING
    }
}
