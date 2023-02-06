#include "CollShapeDyn.h"
#include "RigidBodyContainerDyn.h"
#include <simLib/simLib.h>

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
    delete _collisionShapeDyn;
    CRigidBodyContainerDyn* rbc=(CRigidBodyContainerDyn*)(CRigidBodyContainerDyn::getDynWorld());
    rbc->getWorld()->removeCollisionObject(_rigidBody);
    if (_rigidBody->getMotionState()!=nullptr)
        delete _rigidBody->getMotionState();
    delete _rigidBody;
}

void CRigidBodyDyn::init(CXShape* shape,bool forceStatic,bool forceNonRespondable)
{
    CRigidBodyDyn_base::init(shape,forceStatic,forceNonRespondable);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    double linVelScaling=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();

    C7Vector cumulPart1_scaled;
    _simGetObjectCumulativeTransformation(shape,cumulPart1_scaled.X.data,cumulPart1_scaled.Q.data,true);

    cumulPart1_scaled.X*=linScaling; // ********** SCALING
    C7Vector tr(cumulPart1_scaled*_localInertiaFrame_scaled);
    CXGeomProxy* geomData=(CXGeomProxy*)_simGetGeomProxyFromShape(shape);
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);
    double mass=_mass_scaled;

    btVector3 localInertia(0,0,0);
    if (!_isStatic)
    {
        localInertia.setX(_diagonalInertia_scaled(0));
        localInertia.setY(_diagonalInertia_scaled(1));
        localInertia.setZ(_diagonalInertia_scaled(2));
    }
    else
        mass=0.0;

    btDefaultMotionState* myMotionState = new btDefaultMotionState(btTransform(btQuaternion(tr.Q(1),tr.Q(2),tr.Q(3),tr.Q(0)),btVector3(tr.X(0),tr.X(1),tr.X(2))));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,((CCollShapeDyn*)_collisionShapeDyn)->getBtCollisionShape(),localInertia);
    _rigidBody = new btRigidBody(rbInfo);
    _rigidBody->setUserPointer((void*)_shapeHandle);

    if (_isStatic)
    { // this is for kinematic objects only:
        _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags()|btCollisionObject::CF_KINEMATIC_OBJECT);
        _rigidBody->setActivationState(DISABLE_DEACTIVATION); // Important, because if we remove the floor and this is not set, nothing falls!
        // Following is important otherwise kinematic bodies make dynamic bodies jump at the first movement!
        _rigidBody->setInterpolationWorldTransform(_rigidBody->getWorldTransform());
        _rigidBody->setInterpolationLinearVelocity(btVector3(0,0,0));
        _rigidBody->setInterpolationAngularVelocity(btVector3(0,0,0));
    }
    else
    {
        // Following few lines added on 11/03/2011:
        C3Vector v;
        _simGetInitialDynamicVelocity(shape,v.data);
        if (v.getLength()>0.0)
        {
            _rigidBody->setLinearVelocity(btVector3(v(0)*linVelScaling,v(1)*linVelScaling,v(2)*linVelScaling)); // ********** SCALING
            _simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        }
        // Following few lines added on 25/03/2013:
        _simGetInitialDynamicAngVelocity(shape,v.data);
        if (v.getLength()>0.0)
        {
            _rigidBody->setAngularVelocity(btVector3(v(0),v(1),v(2)));
            _simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        }
        // Kinematic objects are handled elsewhere (at the end of the file)
    }

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double linD,angD;
    _simGetDamping(geomInfo,&linD,&angD);
    _rigidBody->setDamping(linD,angD);
    _rigidBody->setRestitution(_simGetBulletRestitution(geomInfo));
    _rigidBody->setFriction(_simGetFriction(geomInfo));

    _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK); // 22/02/2011: To allow the contact callback to be called!!

    if ((_simIsShapeDynamicallyRespondable(shape)==0)||forceNonRespondable)
        _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);

    // Put body into sleep mode if needed:
    bool deactivate=false;
    if (_simGetStartSleeping(shape))
    {
        if (_simGetWasPutToSleepOnce(shape)==0) // this will set the flag to true!!
        { // Make sure that when the shape is added/removed because we are simply shifting it, that it doesn't stay deactivated!
            deactivate=true;
        }
    }
    if (deactivate)
        _rigidBody->setActivationState(ISLAND_SLEEPING);
    bulletWorld->addRigidBody(_rigidBody);
    if (deactivate)
        _rigidBody->setActivationState(ISLAND_SLEEPING);
    else
        _rigidBody->setActivationState(DISABLE_DEACTIVATION); // 11.05.2021
}

C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;

    btTransform wt;
    btMotionState* ms=_rigidBody->getMotionState();
    if (ms!=nullptr)
        ms->getWorldTransform(wt);
    else
        wt=_rigidBody->getWorldTransform();
    btQuaternion wtq(wt.getRotation());
    btVector3 wtx(wt.getOrigin());
    tr.X(0)=wtx.getX();
    tr.X(1)=wtx.getY();
    tr.X(2)=wtx.getZ();
    tr.Q(0)=wtq.getW();
    tr.Q(1)=wtq.getX();
    tr.Q(2)=wtq.getY();
    tr.Q(3)=wtq.getZ();

    tr.X/=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING
    return(tr);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    if (_collisionShapeDyn!=nullptr)
    {
        btTransform wt;
        btMotionState* ms=_rigidBody->getMotionState();
        if (ms!=nullptr)
            ms->getWorldTransform(wt);
        else
            wt=_rigidBody->getWorldTransform();
        btQuaternion wtq(wt.getRotation());
        btVector3 wtx(wt.getOrigin());
        tr.X(0)=wtx.getX();
        tr.X(1)=wtx.getY();
        tr.X(2)=wtx.getZ();
        tr.Q(0)=wtq.getW();
        tr.Q(1)=wtq.getX();
        tr.Q(2)=wtq.getY();
        tr.Q(3)=wtq.getZ();

        tr*=_inverseLocalInertiaFrame_scaled;
        tr.X/=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING
    }
    else
        tr.setIdentity();
    return(tr);
}

btRigidBody* CRigidBodyDyn::getBtRigidBody()
{
    return(_rigidBody);
}


void CRigidBodyDyn::reportVelocityToShape(double simulationTime)
{
    double vs=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();
    C3Vector lv,av;
    btVector3 btlv(_rigidBody->getLinearVelocity());
    btVector3 btav(_rigidBody->getAngularVelocity());
    lv.setData(btlv.x()/vs,btlv.y()/vs,btlv.z()/vs);
    av.setData(btav.x(),btav.y(),btav.z());
    _simSetShapeDynamicVelocity(_shape,lv.data,av.data,simulationTime);
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques()
{
    double fs=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn(); // ********** SCALING
    double ts=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn(); // ********** SCALING
    C3Vector vf,vt;
    _simGetAdditionalForceAndTorque(_shape,vf.data,vt.data);

    if ((vf.getLength()!=0.0)||(vt.getLength()!=0.0))
    { // We should wake the body!!
        _rigidBody->activate(false);
        btVector3 f;
        btVector3 t;
        f.setX(vf(0)*fs);
        f.setY(vf(1)*fs);
        f.setZ(vf(2)*fs);
        t.setX(vt(0)*ts);
        t.setY(vt(1)*ts);
        t.setZ(vt(2)*ts);
        _rigidBody->applyCentralForce(f);
        _rigidBody->applyTorque(t);
    }
}

void CRigidBodyDyn::handleKinematicBody_step(double t,double cumulatedTimeStep)
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr;
        tr.buildInterpolation(_bodyStart_kinematicBody,_bodyEnd_kinematicBody,t);
        tr.X*=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING

        btQuaternion wtq(tr.Q(1),tr.Q(2),tr.Q(3),tr.Q(0));
        btVector3 wtx(tr.X(0),tr.X(1),tr.X(2));
        btMotionState* ms=_rigidBody->getMotionState();
        if (ms!=nullptr)
            ms->setWorldTransform(btTransform(wtq,wtx));
        else
            _rigidBody->setWorldTransform(btTransform(wtq,wtx));
    }
}

void CRigidBodyDyn::handleKinematicBody_end()
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr(_bodyEnd_kinematicBody);
        tr.X*=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING

        btQuaternion wtq(tr.Q(1),tr.Q(2),tr.Q(3),tr.Q(0));
        btVector3 wtx(tr.X(0),tr.X(1),tr.X(2));
        btMotionState* ms=_rigidBody->getMotionState();

        if (ms!=nullptr)
            ms->setWorldTransform(btTransform(wtq,wtx));
        else
            _rigidBody->setWorldTransform(btTransform(wtq,wtx));
    }
}

