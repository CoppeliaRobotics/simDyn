#include <RigidBodyDyn_base.h>
#include <RigidBodyContainerDyn.h>
#ifdef INCLUDE_MUJOCO_CODE
    #include <simLib/simLib.h>
#else
    #include <simLib/simLib.h>
#endif

CRigidBodyDyn_base::CRigidBodyDyn_base()
{
}

CRigidBodyDyn_base::~CRigidBodyDyn_base()
{
}

void CRigidBodyDyn_base::init(CXShape* shape, bool forceStatic, bool forceNonRespondable)
{
    _shapeHandle = _simGetObjectID(shape);
    _shape = shape;

    _collisionShapeDyn = new CCollShapeDyn();
    bool willBeStatic = (_simIsShapeDynamicallyStatic(shape) || forceStatic);

    _mass_scaled = _simGetLocalInertiaInfo(shape, _localInertiaFrame_scaled.X.data, _localInertiaFrame_scaled.Q.data, _diagonalInertia_scaled.data);
    _mass_scaled *= CRigidBodyContainerDyn::getDynWorld()->getMassScalingFactorDyn();
    _localInertiaFrame_scaled.X *= CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    _inverseLocalInertiaFrame_scaled = _localInertiaFrame_scaled.getInverse();
    _diagonalInertia_scaled *= CRigidBodyContainerDyn::getDynWorld()->getMasslessInertiaScalingFactorDyn();
    _diagonalInertia_scaled *= CRigidBodyContainerDyn::getDynWorld()->getMassScalingFactorDyn();

    _collisionShapeDyn->init(shape, willBeStatic, _inverseLocalInertiaFrame_scaled); // even non-respondable shapes have a collision object

    _isStatic = willBeStatic;
    _isNeverRespondable = forceNonRespondable;
    _simGetObjectLocalTransformation(shape, _localTransformation_old.X.data, _localTransformation_old.Q.data, false); // needed for the "parent follows"-thing!
    _bodyStart_kinematicBody.setIdentity();
    _bodyEnd_kinematicBody.setIdentity();
}

CCollShapeDyn* CRigidBodyDyn_base::getCollisionShapeDyn() const
{
    return (_collisionShapeDyn);
}

C7Vector CRigidBodyDyn_base::getInertiaFrameTransformation()
{
    return (C7Vector::identityTransformation);
}

C7Vector CRigidBodyDyn_base::getShapeFrameTransformation()
{
    return (C7Vector::identityTransformation);
}

void CRigidBodyDyn_base::reportVelocityToShape(sReal simulationTime)
{
}

void CRigidBodyDyn_base::handleAdditionalForcesAndTorques()
{
}

void CRigidBodyDyn_base::handleKinematicBody_step(sReal t, sReal cumulatedTimeStep)
{
}

void CRigidBodyDyn_base::handleKinematicBody_end()
{
}

bool CRigidBodyDyn_base::isStatic() const
{
    return (_isStatic);
}

bool CRigidBodyDyn_base::isNeverRespondable() const
{
    return (_isNeverRespondable);
}

int CRigidBodyDyn_base::getShapeHandle() const
{
    return (_shapeHandle);
}

CXShape* CRigidBodyDyn_base::getShape() const
{
    return (_shape);
}

void CRigidBodyDyn_base::reportConfigurationToShape(sReal simulationTime)
{
    if (!_isStatic)
    { // dynamic
        // set the pose of non-static bodies, and set intrinsic dynamic errors of joints/force sensors linked to that shape
        C7Vector tr(getShapeFrameTransformation());
        _simDynReportObjectCumulativeTransformation(_shape, tr.X.data, tr.Q.data, simulationTime);
    }
}

void CRigidBodyDyn_base::handleKinematicBody_init(sReal dt)
{
    if (_isStatic)
    {
        _bodyStart_kinematicBody = getInertiaFrameTransformation();
        C7Vector aax(_localInertiaFrame_scaled);
        aax.X /= CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn(); // ********** SCALING

        C7Vector cumulTrp1;
        _simGetObjectCumulativeTransformation(_shape, cumulTrp1.X.data, cumulTrp1.Q.data, true);
        _bodyEnd_kinematicBody = cumulTrp1 * aax;
        _applyBodyToShapeTransf_kinematicBody = (((_bodyStart_kinematicBody.X - _bodyEnd_kinematicBody.X).getLength() > 0.00001) || (_bodyStart_kinematicBody.Q.getAngleBetweenQuaternions(_bodyEnd_kinematicBody.Q) > 0.05 * degToRad));

        C3Vector dx;
        _simGetInitialDynamicVelocity(_shape, dx.data);
        dx *= dt;
        C3Vector dxa;
        _simGetInitialDynamicAngVelocity(_shape, dxa.data);
        if (dx.getLength() + dxa.getLength() > 0.0)
        {
            _simSetInitialDynamicVelocity(_shape, C3Vector::zeroVector.data);    // important to reset it!
            _simSetInitialDynamicAngVelocity(_shape, C3Vector::zeroVector.data); // important to reset it!
            C7Vector tr;
            _simGetObjectCumulativeTransformation(_shape, tr.X.data, tr.Q.data, true);
            tr.X += dx;
            if (dxa.getLength() > 0.0)
            { // following new since 25/03/2013
                C4Vector q(dxa(0), dxa(1), dxa(2));
                C4Vector angleAndAxis(q.getAngleAndAxis());
                q.setAngleAndAxis(angleAndAxis(0) * dt, C3Vector(angleAndAxis(1), angleAndAxis(2), angleAndAxis(3)));
                tr.Q = tr.Q * q;
            }
            _bodyEnd_kinematicBody = tr * aax;
            _applyBodyToShapeTransf_kinematicBody = true;
        }
    }
}
