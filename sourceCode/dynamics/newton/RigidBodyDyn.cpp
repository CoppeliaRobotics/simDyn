#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "NewtonConvertUtil.h"
#include "4X4FullMatrix.h"
#include "simLib.h"

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
    delete _collisionShapeDyn;
    if (NewtonBodyGetSkeleton(_newtonBody))
    {
        NewtonWorld* const world = NewtonBodyGetWorld(_newtonBody);
        CRigidBodyContainerDyn* const rigidBodyContainerDyn = (CRigidBodyContainerDyn*) NewtonWorldGetUserData(world);
        rigidBodyContainerDyn->_notifySekeletonRebuild();
    }
    NewtonDestroyBody (_newtonBody);
}

void CRigidBodyDyn::init(CXShape* shape,bool forceStatic,bool forceNonRespondable)
{
    CRigidBodyDyn_base::init(shape,forceStatic,forceNonRespondable);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector cumulPart1_scaled;
    _simGetObjectCumulativeTransformation(shape, cumulPart1_scaled.X.data, cumulPart1_scaled.Q.data, true);

    C7Vector tr(cumulPart1_scaled*_localInertiaFrame_scaled);
    CXGeomProxy* geomData = (CXGeomProxy*)_simGetGeomProxyFromShape(shape);
    CXGeomWrap* geomInfo = (CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);

    float mass = _isStatic ? 0.0f : _mass_scaled;

    _setNewtonParameters(shape);

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(tr));
    _newtonBody = NewtonCreateDynamicBody (world,((CCollShapeDyn*)_collisionShapeDyn)->getNewtonCollision(), &matrix[0][0]);
    _newtonBodyUserData[0]=&_shapeHandle;
    _newtonBodyUserData[1]=this;
    _newtonBodyUserData[2]=&_newtonStaticFriction;
    _newtonBodyUserData[3]=&_newtonKineticFriction;
    _newtonBodyUserData[4]=&_newtonRestitution;

    NewtonBodySetUserData(_newtonBody, _newtonBodyUserData);

    // disable auto sleep at all time
    NewtonBodySetAutoSleep(_newtonBody, 0);

    // set linear and angular damping
    NewtonBodySetLinearDamping(_newtonBody, _newtonLinearDrag);
    dVector angularDamp (_newtonAngularDrag, _newtonAngularDrag, _newtonAngularDrag, 0.0f);
    NewtonBodySetAngularDamping(_newtonBody, &angularDamp[0]);

    // attach the CXGeomWrap* as user dat of the collsion shape, thsi si so that we can apply material propertes in the collision callacks 
    NewtonCollision* const collision = NewtonBodyGetCollision(_newtonBody);
    NewtonCollisionSetUserData(collision, geomInfo);

    // initialize uninitialized variables
    m_externForce.clear();
    m_externTorque.clear();

    if (!_isStatic)
    {
        // this function will set a full inertia matrix with origin at the center of the collision shape
        NewtonBodySetMassProperties(_newtonBody, mass,((CCollShapeDyn*)_collisionShapeDyn)->getNewtonCollision());

        // here inertia will be set to the principal axis, but the origin is still at the center of collision shape.
        NewtonBodySetMassMatrix(_newtonBody,mass,_diagonalInertia_scaled(0),_diagonalInertia_scaled(1),_diagonalInertia_scaled(2));

        // Here we simply need to reset the COM to the origin of the rigid body:
        C3Vector com;
        com.clear();
        NewtonBodySetCentreOfMass (_newtonBody, com.data);

        NewtonBodySetTransformCallback(_newtonBody, TransformCallback);
        NewtonBodySetForceAndTorqueCallback(_newtonBody, ApplyExtenalForceCallback);

        C3Vector v;
        _simGetInitialDynamicVelocity(shape, v.data);
        NewtonBodySetVelocity(_newtonBody, v.data);

        _simGetInitialDynamicAngVelocity(shape, v.data);
        NewtonBodySetOmega(_newtonBody, v.data);
        _simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it
        _simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it

        if (_newtonFastMoving)
            NewtonBodySetContinuousCollisionMode(_newtonBody,1); // fast moving

    }

    CRigidBodyContainerDyn* const rigidBodyContainerDyn = (CRigidBodyContainerDyn*) NewtonWorldGetUserData(world);
    rigidBodyContainerDyn->_notifySekeletonRebuild();
}

C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    tr = getNewtonMatrix();
    return(tr);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{ // return value is unscaled (relative to CoppeliaSim)
    C7Vector tr;
    if (_collisionShapeDyn!=nullptr)
    {
        tr = getNewtonMatrix();
        tr*=_inverseLocalInertiaFrame_scaled;
    }
    else
        tr.setIdentity();
    return(tr);
}

NewtonBody* CRigidBodyDyn::getNewtonRigidBody() const
{
    return _newtonBody;
}

void CRigidBodyDyn::reportVelocityToShape(float simulationTime)
{
    C3Vector lv,av;

    NewtonBodyGetOmega(_newtonBody, &av(0));
    NewtonBodyGetVelocity(_newtonBody, &lv(0));

    _simSetShapeDynamicVelocity(_shape,lv.data,av.data,simulationTime);
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques()
{
    C3Vector vf,vt;
    _simGetAdditionalForceAndTorque(_shape,vf.data,vt.data);

    m_externTorque.clear();
    m_externForce.clear();
    // In Newton bodies are never sleeping!
    if ((vf.getLength() != 0.0f) || (vt.getLength() != 0.0f))
    { // We should wake the body!!
        m_externTorque=vt;
        m_externForce=vf;
    }
}

void CRigidBodyDyn::handleKinematicBody_step(float t,float cumulatedTimeStep)
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr;
        tr.buildInterpolation(_bodyStart_kinematicBody,_bodyEnd_kinematicBody,t);

        dMatrix posit;
        dQuaternion rot;
        NewtonBodyGetRotation(_newtonBody, &rot.m_q0);
        NewtonBodyGetMatrix(_newtonBody, &posit[0][0]);
        C4X4FullMatrix transform(tr.getMatrix());
        dMatrix matrix(&transform(0, 0));
        matrix = matrix.Transpose4X4();
        dQuaternion rot1 (matrix);

        float timestep = CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
        dVector veloc ((matrix.m_posit - posit.m_posit).Scale (1.0f / timestep));
        dVector omega (rot.CalcAverageOmega(rot1, 1.0f / timestep));

        NewtonBodySetVelocity(_newtonBody, &veloc[0]);
        NewtonBodySetOmega(_newtonBody, &omega[0]);
        NewtonBodyIntegrateVelocity (_newtonBody, timestep);
        NewtonBodySetMatrix(_newtonBody, &matrix[0][0]);
    }
}

void CRigidBodyDyn::handleKinematicBody_end()
{
    if (_isStatic&&_applyBodyToShapeTransf_kinematicBody)
    { // static & moved (the desired movement was large enough)
        C7Vector tr(_bodyEnd_kinematicBody);

        dVector zeroVeloc (0.0f, 0.0f, 0.0f, 0.0f);
        NewtonBodySetVelocity(_newtonBody, &zeroVeloc[0]);
        NewtonBodySetOmega(_newtonBody, &zeroVeloc[0]);
    }
}

C7Vector CRigidBodyDyn::getNewtonMatrix() const
{
    dMatrix matrix;
    NewtonBodyGetMatrix(_newtonBody, &matrix[0][0]);
    matrix = matrix.Transpose4X4();
    float tmp[4][4];
    memcpy(tmp, &matrix[0][0], sizeof (tmp));
    return C7Vector(tmp);
}

void CRigidBodyDyn::_setNewtonParameters(CXShape* shape)
{
    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float floatParams[5];
    int intParams[1];
    int parVer=0;
    _simGetNewtonParameters(shape,&parVer,floatParams,intParams);

    const float staticFriction=floatParams[0];
    const float kineticFriction=floatParams[1];
    const float restitution=floatParams[2];
    const float linearDrag=floatParams[3];
    const float angularDrag=floatParams[4];

    const bool fastMoving=(intParams[0]&1)!=false;

    _newtonStaticFriction=staticFriction;
    _newtonKineticFriction=kineticFriction;
    _newtonRestitution=restitution;
    _newtonLinearDrag=linearDrag;
    _newtonAngularDrag=angularDrag;
    _newtonFastMoving=fastMoving;
}

void CRigidBodyDyn::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
    //Julio: I am no sure if CoppeliaSim Collect the transforms after the update but iteration ov e bodies in a loop
}

void CRigidBodyDyn::ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
{
    float mass;
    float Ixx;
    float Iyy;
    float Izz;
    C3Vector gravity;

    _simGetGravity(gravity.data);
    NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);
    gravity *= mass;

    void** userData=(void**)NewtonBodyGetUserData(body);
    CRigidBodyDyn* const me = (CRigidBodyDyn*)userData[1];
    gravity += ((CRigidBodyDyn*)me)->m_externForce;
    NewtonBodyAddForce(body, &gravity.data[0]);
    NewtonBodyAddTorque(body, &((CRigidBodyDyn*)me)->m_externTorque.data[0]);
}

