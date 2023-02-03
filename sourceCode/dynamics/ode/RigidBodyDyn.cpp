#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include <simLib.h>

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
    delete _collisionShapeDyn;
    dBodyDestroy(_odeRigidBody);
}

void CRigidBodyDyn::init(CXShape* shape,bool forceStatic,bool forceNonRespondable)
{
    CRigidBodyDyn_base::init(shape,forceStatic,forceNonRespondable);

    dWorldID odeWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    double linVelScaling=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();

    C7Vector cumulPart1_scaled;
    _simGetObjectCumulativeTransformation(shape,cumulPart1_scaled.X.data,cumulPart1_scaled.Q.data,true);


    cumulPart1_scaled.X*=linScaling; // ********** SCALING
    C7Vector tr(cumulPart1_scaled*_localInertiaFrame_scaled);
    CXGeomProxy* geomData=(CXGeomProxy*)_simGetGeomProxyFromShape(shape);
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);
    double mass=_mass_scaled;

    dMass m;
    dMassSetZero(&m);

    if (!_isStatic)
        dMassSetParameters(&m,mass,0.0,0.0,0.0,_diagonalInertia_scaled(0),_diagonalInertia_scaled(1),_diagonalInertia_scaled(2),0.0,0.0,0.0);

    _odeRigidBody=dBodyCreate(odeWorld);
    if (!_isStatic)
    {
        dBodySetMass(_odeRigidBody,&m);

        // Following 6 lines added on 11/03/2011:
        C3Vector v;
        _simGetInitialDynamicVelocity(shape,v.data);
        if (v.getLength()>0.0)
        {
            dBodySetLinearVel(_odeRigidBody,v(0)*linVelScaling,v(1)*linVelScaling,v(2)*linVelScaling); // ********** SCALING
            _simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        }
        // Following 6 lines added on 25/03/2013:
        _simGetInitialDynamicAngVelocity(shape,v.data);
        if (v.getLength()>0.0)
        {
            dBodySetAngularVel(_odeRigidBody,v(0),v(1),v(2));
            _simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        }
        // Kinematic objects are handled elsewhere (at the end of the file)
    }
    else
    {
        dBodySetKinematic(_odeRigidBody);
//        // Following instruction doesn't have any effect (check the Bullet equivalent), so I temporarily disabled the auto-disable function for ALL bodies (see just above)
//        dBodySetAutoDisableFlag(_odeRigidBody,0); // Important, because if we remove the floor and this is not set, nothing falls!
    }
    dBodySetPosition(_odeRigidBody,tr.X(0),tr.X(1),tr.X(2));
    dQuaternion dQ;
    dQ[0]=tr.Q.data[0];
    dQ[1]=tr.Q.data[1];
    dQ[2]=tr.Q.data[2];
    dQ[3]=tr.Q.data[3];
    dBodySetQuaternion(_odeRigidBody,dQ);
    dBodySetData(_odeRigidBody,(void*)_shapeHandle);
    int index=0;
    while (true)
    {
        dGeomID odeGeomID=_collisionShapeDyn->getOdeGeoms(index++);
        if (odeGeomID==nullptr)
            break;
        dGeomSetBody(odeGeomID,_odeRigidBody);
    }

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double linD,angD;
    _simGetDamping(geomInfo,&linD,&angD);
    dBodySetLinearDamping(_odeRigidBody,linD);
    dBodySetAngularDamping(_odeRigidBody,angD);

    // For now, we disable the auto-disable functionality (because there are problems when removing a kinematic object during simulation(e.g. removing the floor, nothing falls)):
    dBodySetAutoDisableFlag(_odeRigidBody,0);

    if (_simGetStartSleeping(shape))
    {
        if (_simGetWasPutToSleepOnce(shape)==0) // this will set the flag to true!!
            dBodyDisable(_odeRigidBody); // Make sure that when the shape is added/removed because we are simply shifting it, that it doesn't stay deactivated!
    }
}

C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;

    const dReal* pos=dBodyGetPosition(_odeRigidBody);
    const dReal* quat=dBodyGetQuaternion(_odeRigidBody);
    tr.X(0)=pos[0];
    tr.X(1)=pos[1];
    tr.X(2)=pos[2];
    tr.Q(0)=quat[0];
    tr.Q(1)=quat[1];
    tr.Q(2)=quat[2];
    tr.Q(3)=quat[3];

    tr.X/=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING
    return(tr);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    if (_collisionShapeDyn!=nullptr)
    {

        const dReal* pos=dBodyGetPosition(_odeRigidBody);
        const dReal* quat=dBodyGetQuaternion(_odeRigidBody);
        tr.X(0)=pos[0];
        tr.X(1)=pos[1];
        tr.X(2)=pos[2];
        tr.Q(0)=quat[0];
        tr.Q(1)=quat[1];
        tr.Q(2)=quat[2];
        tr.Q(3)=quat[3];

        tr*=_inverseLocalInertiaFrame_scaled;
        tr.X/=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING
    }
    else
        tr.setIdentity();
    return(tr);
}

dBodyID CRigidBodyDyn::getOdeRigidBody()
{
    return(_odeRigidBody);
}

void CRigidBodyDyn::reportVelocityToShape(double simulationTime)
{
    double vs=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn(); // ********** SCALING
    C3Vector lv,av;

    const dReal* lvd=dBodyGetLinearVel(_odeRigidBody);
    lv.setData(lvd[0]/vs,lvd[1]/vs,lvd[2]/vs);
    const dReal* avd=dBodyGetAngularVel(_odeRigidBody);
    av.setData(avd[0],avd[1],avd[2]);

    _simSetShapeDynamicVelocity(_shape,lv.data,av.data,simulationTime);
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques()
{
    double fs=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn(); // ********** SCALING
    double ts=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn(); // ********** SCALING
    C3Vector vf,vt;
    _simGetAdditionalForceAndTorque(_shape,vf.data,vt.data);

    // In ODE bodies are never sleeping!
    if ((vf.getLength()!=0.0)||(vt.getLength()!=0.0))
    { // We should wake the body!!
        dBodyEnable(_odeRigidBody);
        dBodyAddForce(_odeRigidBody,vf(0)*fs,vf(1)*fs,vf(2)*fs);
        dBodyAddTorque(_odeRigidBody,vt(0)*ts,vt(1)*ts,vt(2)*ts);
    }
}

void CRigidBodyDyn::handleKinematicBody_step(double t,double cumulatedTimeStep)
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr;
        tr.buildInterpolation(_bodyStart_kinematicBody,_bodyEnd_kinematicBody,t);
        tr.X*=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING

        dBodySetPosition(_odeRigidBody,tr.X(0),tr.X(1),tr.X(2));
        dQuaternion dQ;
        dQ[0]=tr.Q.data[0];
        dQ[1]=tr.Q.data[1];
        dQ[2]=tr.Q.data[2];
        dQ[3]=tr.Q.data[3];
        dBodySetQuaternion(_odeRigidBody,dQ);
        dBodyEnable(_odeRigidBody);
        // Important to set the velocity here, so that collisions react correctly:
        C3Vector dx((_bodyEnd_kinematicBody.X-_bodyStart_kinematicBody.X)*CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn()); // ********** SCALING
        dBodySetLinearVel(_odeRigidBody,dx(0)/cumulatedTimeStep,dx(1)/cumulatedTimeStep,dx(2)/cumulatedTimeStep);

        C4Vector q(_bodyEnd_kinematicBody.Q*_bodyStart_kinematicBody.Q.getInverse());
        C3Vector dEuler(q.getEulerAngles());
        dBodySetAngularVel(_odeRigidBody,dEuler(0)/cumulatedTimeStep,dEuler(1)/cumulatedTimeStep,dEuler(2)/cumulatedTimeStep);
    }
}

void CRigidBodyDyn::handleKinematicBody_end()
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr(_bodyEnd_kinematicBody);
        tr.X*=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING

        dBodySetPosition(_odeRigidBody,tr.X(0),tr.X(1),tr.X(2));
        dQuaternion dQ;
        dQ[0]=tr.Q.data[0];
        dQ[1]=tr.Q.data[1];
        dQ[2]=tr.Q.data[2];
        dQ[3]=tr.Q.data[3];
        dBodySetQuaternion(_odeRigidBody,dQ);
        dBodyEnable(_odeRigidBody);
        // Important to set the velocity to 0 here:
        dBodySetLinearVel(_odeRigidBody,0.0,0.0,0.0);
        dBodySetAngularVel(_odeRigidBody,0.0,0.0,0.0);
    }
}

