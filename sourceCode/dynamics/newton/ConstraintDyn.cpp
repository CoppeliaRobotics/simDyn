#include "ConstraintDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"
#include "4X4Matrix.h"
#include "NewtonConvertUtil.h"

class CConstraintDyn::csimNewtonForceSensorJoint: public CustomHinge
{
    public:
    csimNewtonForceSensorJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomHinge(pinAndPivotFrame, child, parent)
        , m_broken(false)
    {
        EnableLimits(true);
        CustomHinge::SetLimits(0.0, 0.0);
    }

    void SubmitConstraints(dFloat timestep, int threadIndex)
    {
        if (!m_broken) {
            CustomHinge::SubmitConstraints(timestep, threadIndex);
        }
    }

    C3Vector GetForce() const
    {
        return C3Vector(-NewtonUserJointGetRowForce(m_joint, 0), -NewtonUserJointGetRowForce(m_joint, 1), -NewtonUserJointGetRowForce(m_joint, 2));
    }

    C3Vector GetTorque() const
    {
        return C3Vector(-NewtonUserJointGetRowForce(m_joint, 5), -NewtonUserJointGetRowForce(m_joint, 3), -NewtonUserJointGetRowForce(m_joint, 4));
    }

    bool m_broken;
};

class CConstraintDyn::csimNewtonCommonJointData
{
    public:
    enum JointMode
    {
        m_free,
        m_motor,
        m_position,
    };

    csimNewtonCommonJointData()
        :m_joint(nullptr)
        ,m_lowLimit(-1.0e10f)
        ,m_highLimit(1.0e10f)
        ,m_lowForce(1.0e10f)
        ,m_highForce(1.0e10f)
        ,m_motorSpeed(0.0)
        ,m_targetPosition(0.0)
        ,m_axisWasActive(false)
        ,m_lockOnTargetMode(false)
        ,m_mode(m_free)
    {
    }

    sReal GetJointForce() const
    {
        return m_axisWasActive ? -NewtonUserJointGetRowForce(m_joint->GetJoint(), 5) : 0.0;
    }

    void SetLimits(sReal low, sReal high)
    {
        m_lowLimit = low;
        m_highLimit = high;
    }

    void SetMotor(bool motorIsOn, sReal speed, sReal maxForce)
    {
        m_lockOnTargetMode = false;
        if (motorIsOn) {
            m_motorSpeed = speed;
            m_highForce = fabsf(maxForce);
            m_lowForce = -m_highForce;
            m_mode = m_motor;
        }
        else
        {
            m_mode = m_free;
        }
    }

    void LockOnTargetPosition(sReal targetPosition)
    {
        m_lockOnTargetMode = true;
        m_targetPosition = targetPosition;
        m_mode = m_position;
    }

    CustomJoint* m_joint;
    sReal m_lowLimit;
    sReal m_highLimit;
    sReal m_lowForce;
    sReal m_highForce;
    sReal m_motorSpeed;
    sReal m_targetPosition;
    bool m_axisWasActive;
    bool m_lockOnTargetMode;
    JointMode m_mode;
};

class CConstraintDyn::csimNewtonRevoluteJoint: public CustomHingeActuator
{
    public:
    csimNewtonRevoluteJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomHingeActuator(pinAndPivotFrame, child, parent)
        ,m_data()
    {
        m_data.m_joint = this;
        EnableLimits(true);
        CustomHinge::SetLimits(-1.0e10f, 1.0e10f);
        SetFriction(0.0);

        // JULIO: By default joints disable collision on the object that the link, in general the application handle this by limits,
        // collision can be enable by calling the function which can be a joint option.
        // for now I am assuming they alway collide
        SetBodiesCollisionState(true);
    }

    void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
    {
        m_data.m_axisWasActive = false;
        //dTrace (("%f %f\n", GetJointAngle(), GetJointOmega()));

        switch (m_data.m_mode)
        {
            case csimNewtonCommonJointData::m_free:
            {
                CustomHinge::SetLimits(m_data.m_lowLimit, m_data.m_highLimit);
                CustomHinge::SubmitConstraintsFreeDof(timestep, matrix0, matrix1);
                m_data.m_axisWasActive = m_lastRowWasUsed;
                break;
            }

            case csimNewtonCommonJointData::m_position:
            {
                // I did not test this, but I believe is right
                m_data.m_axisWasActive = true;
                sReal posit = GetJointAngle();
                sReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_targetPosition, posit);
                NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                NewtonUserJointSetRowStiffness(m_joint, 1.0);
                break;
            }

            case csimNewtonCommonJointData::m_motor:
            default:
            {
                sReal posit = GetJointAngle();
                sReal speed = GetJointOmega();
                sReal accel = 0.5 * (m_data.m_motorSpeed - speed) / timestep;
                if (posit <= m_data.m_lowLimit)
                {
                    sReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_lowLimit, posit);
                    NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed > 0.0) && (posit1 > m_data.m_lowLimit))
                    if (m_data.m_motorSpeed > 0.0)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else if (posit >= m_data.m_highLimit)
                {
                    sReal step = CConstraintDyn::getAngleMinusAlpha(m_data.m_highLimit, posit);
                    NewtonUserJointAddAngularRow(m_joint, step, &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed < 0.0) && (posit1 > m_data.m_highLimit))
                    if (m_data.m_motorSpeed < 0.0)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else
                {
                    NewtonUserJointAddAngularRow(m_joint, 0.0, &matrix0.m_front[0]);
                    NewtonUserJointSetRowAcceleration(m_joint, accel);
                    NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                    NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                }
                NewtonUserJointSetRowStiffness(m_joint, 1.0);
                m_data.m_axisWasActive = true;
                break;
            }
        }
    }

    csimNewtonCommonJointData m_data;
};

class CConstraintDyn::csimNewtonPrismaticJoint: public CustomSliderActuator
{
    public:
    csimNewtonPrismaticJoint(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
        :CustomSliderActuator(pinAndPivotFrame, child, parent)
        ,m_data()
    {
        m_data.m_joint = this;
        EnableLimits(true);
        CustomSlider::SetLimits(-1.0e10f, 1.0e10f);

        // JULIO: By default joints disable collision on the object that the link, in general the application handle this by limits,
        // collision can be enable by calling the function which can be a joint option.
        // for now I am assuming they alway collide
        SetBodiesCollisionState(true);
    }

    void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
    {
        m_data.m_axisWasActive = false;
        //dTrace (("%f %f %f\n", GetJointPosit(), GetJointSpeed(), (m_data.m_motorSpeed - GetJointSpeed()) / timestep));
        switch (m_data.m_mode)
        {
            case csimNewtonCommonJointData::m_free:
            {
                CustomSlider::SetLimits(m_data.m_lowLimit, m_data.m_highLimit);
                CustomSlider::SubmitConstraintsFreeDof(timestep, matrix0, matrix1);
                m_data.m_axisWasActive = m_lastRowWasUsed;
                break;
            }
            case csimNewtonCommonJointData::m_position:
            {
                // I did not test this, but I believe is right
                m_data.m_axisWasActive = true;
                sReal posit = GetJointPosit();
                sReal step = m_data.m_targetPosition - posit;
                const dVector& p0 = matrix0.m_posit;
                dVector p1(p0 + matrix0.m_front.Scale(step));
                NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                NewtonUserJointSetRowStiffness(m_joint, 1.0);
                break;
            }

            case csimNewtonCommonJointData::m_motor:
            default:
            {
                sReal posit = GetJointPosit();
                sReal speed = GetJointSpeed();
                sReal accel = 0.5 * (m_data.m_motorSpeed - speed) / timestep;
                if (posit <= m_data.m_lowLimit)
                {
                    sReal step = m_data.m_lowLimit - posit;
                    const dVector& p0 = matrix0.m_posit;
                    dVector p1(p0 + matrix0.m_front.Scale(step));
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed > 0.0) && (posit1 > m_data.m_lowLimit))
                    if (m_data.m_motorSpeed > 0.0)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else if (posit >= m_data.m_highLimit)
                {
                    sReal step = m_data.m_highLimit - posit;
                    const dVector& p0 = matrix0.m_posit;
                    dVector p1(p0 + matrix0.m_front.Scale(step));
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
                    //dFloat posit1 = posit + speed * timestep + accel * timestep * timestep;
                    //if ((m_data.m_motorSpeed < 0.0) && (posit1 < m_data.m_highLimit))
                    if (m_data.m_motorSpeed < 0.0)
                    {
                        NewtonUserJointSetRowAcceleration(m_joint, accel);
                        NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                        NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                    }
                }
                else
                {
                    const dVector& p0 = matrix0.m_posit;
                    NewtonUserJointAddLinearRow(m_joint, &p0[0], &p0[0], &matrix0.m_front[0]);
                    NewtonUserJointSetRowAcceleration(m_joint, accel);
                    NewtonUserJointSetRowMinimumFriction(m_joint, m_data.m_lowForce);
                    NewtonUserJointSetRowMaximumFriction(m_joint, m_data.m_highForce);
                }
                NewtonUserJointSetRowStiffness(m_joint, 1.0);
                m_data.m_axisWasActive = true;
                break;
            }
        }
    }
    csimNewtonCommonJointData m_data;
};

CConstraintDyn::CConstraintDyn()
{
}
CConstraintDyn::~CConstraintDyn()
{
    _notifySekeletonRebuild();
    delete _newtonConstraint;
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _newtonJointOffset = _simGetJointPosition(joint);

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint, jtr.X.data, jtr.Q.data, true);

    // in Newton the moving direction is the x-axis
    C3X3Matrix m;
    m.buildYRotation(-piValue*0.5);
    jtr.Q=jtr.Q*m.getQuaternion();

    // wake up the parent body and disable deactivation for future:
    //NewtonBodySetSleepState(bodyA->getNewtonRigidBody(),1);
    //NewtonBodySetFreezeState(bodyA->getNewtonRigidBody(),0);
    //NewtonBodySetAutoSleep(bodyA->getNewtonRigidBody(),0);

    // wake up the child body and disable deactivation for future:
    //NewtonBodySetSleepState(bodyB->getNewtonRigidBody(),1);
    //NewtonBodySetFreezeState(bodyB->getNewtonRigidBody(),0);
    //NewtonBodySetAutoSleep(bodyB->getNewtonRigidBody(),0);

    NewtonBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getNewtonRigidBody();

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    switch (_simGetJointType(joint))
    {
        case sim_joint_spherical_subtype:
        {
            _newtonConstraint = new CustomBallAndSocket(matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }

        case sim_joint_prismatic_subtype:
        {
            _newtonConstraint = new csimNewtonPrismaticJoint(matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }

        case sim_joint_revolute_subtype:
        {
            _newtonConstraint = new csimNewtonRevoluteJoint (matrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData (this);
            break;
        }
        default:
        {
            _ASSERTE (0);
        }
    }

    _isAcyclic = true;
    _setNewtonParameters(joint);
    _notifySekeletonRebuild();
    handleMotorControl(joint, 0, 0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXJoint* joint, CXDummy* dummyA, CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint,dummyA,dummyB);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _newtonJointOffset =0.0;


    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint, jtr.X.data, jtr.Q.data, true);

    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(dummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_localTrA_2.getInverse());


    C3X3Matrix m;
    m.buildYRotation(-piValue*0.5);
    jtr.Q = jtr.Q*m.getQuaternion();


    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
            _newtonJointOffset =_simGetJointPosition(joint);
    //        jointOffsetThing.buildZRotation(_newtonJointOffset);
        }
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
    }
    else
        jtr2.Q = jtr2.Q*m.getQuaternion();
    dMatrix jmatrix(GetDMatrixFromCoppeliaSimTransformation(jtr));

//----
    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;
//----

//----

    NewtonBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getNewtonRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
//    dMatrix matrixT=matrixI.Transpose4X4();
//    C4X4Matrix mTemp;
//    mTemp.setData(&matrixT[0][0]);
//    C7Vector cb_a(mTemp.getTransformation());
    dMatrix matrixT=matrixI;
    C7Vector cb_a(GetCoppeliaSimTransformationFromDMatrix(matrixT));
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);
//----

    switch (_simGetJointType(joint))
    {
        case sim_joint_spherical_subtype:
        {
            _newtonConstraint = new CustomBallAndSocket(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }

        case sim_joint_prismatic_subtype:
        {
            _newtonConstraint = new csimNewtonPrismaticJoint(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }

        case sim_joint_revolute_subtype:
        {
            _newtonConstraint = new csimNewtonRevoluteJoint(jmatrix, childRigidBody, parentRigidBody);
            _newtonConstraint->SetUserData(this);
            break;
        }
        default:
        {
              _ASSERTE(0);
        }
    }

    //----
    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);
    //----

    _isAcyclic = false;
    _setNewtonParameters(joint);
    _notifySekeletonRebuild();
    handleMotorControl(joint, 0, 0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXDummy* dummyA, CXDummy* dummyB)
{ // This is a rigid link between 2 rigid bodies involved in a loop closure (i.e. body1 - dummy1 - dummy2 - body2)
    CConstraintDyn_base::init(bodyA,bodyB,dummyA,dummyB);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _newtonJointOffset=0.0;

    C7Vector dtr,dtr2;
    _simGetObjectCumulativeTransformation(dummyA,dtr.X.data,dtr.Q.data,true);
    _simGetObjectCumulativeTransformation(dummyB,dtr2.X.data,dtr2.Q.data,true);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector dtrRelToBodyA;
    C7Vector dtrRelToBodyB;

    NewtonBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getNewtonRigidBody();

    // Set the configuration of the child as if the dummies were overlapping:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
//    dMatrix matrixT=matrixI.Transpose4X4();
//    C4X4Matrix tmp;
//    tmp.setData(&matrixT[0][0]);
//    C7Vector cb_a(tmp);
    dMatrix matrixT=matrixI;
    C7Vector cb_a(GetCoppeliaSimTransformationFromDMatrix(matrixT));
    C7Vector x(cb_a.getInverse()*dtr2);
    C7Vector cb_b(dtr*x.getInverse());
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);


    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(dtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);

    _notifySekeletonRebuild();
    _isAcyclic = false;

}

void CConstraintDyn::init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _newtonJointOffset=0.0;

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(forceSensor,jtr2.X.data,jtr2.Q.data,false);

    NewtonBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getNewtonRigidBody();

    // wake up the parent body and disable deactivation for future:
    NewtonBodySetSleepState(parentRigidBody,1);
    NewtonBodySetFreezeState(parentRigidBody,0);
    NewtonBodySetAutoSleep(parentRigidBody,0);

    // wake up the child body and disable deactivation for future:
    NewtonBodySetSleepState(childRigidBody,1);
    NewtonBodySetFreezeState(childRigidBody,0);
    NewtonBodySetAutoSleep(childRigidBody,0);

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    _isAcyclic = true;
    _setForceSensorBrokenUnbrokenConstraints_newton ();
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA, CRigidBodyDyn* bodyB, CXForceSensor* forceSensor, CXDummy* dummyA, CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor,dummyA,dummyB);

    NewtonWorld* world=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _newtonJointOffset =0.0;

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(dummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_localTrA_2.getInverse());

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    NewtonBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getNewtonRigidBody();
    NewtonBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getNewtonRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    NewtonBody* cb=childRigidBody;
    dMatrix matrixI;
    NewtonBodyGetMatrix(cb,&matrixI[0][0]);
//    dMatrix matrixT=matrixI.Transpose4X4();
//    C4X4Matrix tmp;
//    tmp.setData(&matrixT[0][0]);
//    C7Vector cb_a(tmp);
    dMatrix matrixT=matrixI;
    C7Vector cb_a(GetCoppeliaSimTransformationFromDMatrix(matrixT));
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    matrixT=GetDMatrixFromCoppeliaSimTransformation(cb_b);
    NewtonBodySetMatrix(cb,&matrixT[0][0]);
//----

    dMatrix matrix (GetDMatrixFromCoppeliaSimTransformation(jtr));
    _newtonConstraint = new csimNewtonForceSensorJoint (matrix, childRigidBody, parentRigidBody);
    _newtonConstraint->SetUserData (this);

    //----
    // Reset the configuration of the child as it is now:
    NewtonBodySetMatrix(cb,&matrixI[0][0]);
    //----

    _notifySekeletonRebuild();
    _setForceSensorBrokenUnbrokenConstraints_newton();
    _ASSERTE (dummyA && dummyB);
    _isAcyclic = false;
}

void CConstraintDyn::_updateJointLimits(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    if (jointType==sim_joint_revolute_subtype)
    {
        csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*)_newtonConstraint;
        if (_simGetJointPositionInterval(joint, nullptr, nullptr) == 0)
            jointClass->m_data.SetLimits(-1.0e10f, 1.0e10f);
        else
            jointClass->m_data.SetLimits(_nonCyclicRevoluteJointPositionMinimum-_newtonJointOffset,_nonCyclicRevoluteJointPositionMinimum+_nonCyclicRevoluteJointPositionRange-_newtonJointOffset);
    }
    else
    {
        sReal jiMin,jiRange;
        _simGetJointPositionInterval(joint,&jiMin,&jiRange);

        csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*)_newtonConstraint;
        if (_simGetJointPositionInterval(joint, nullptr, nullptr) == 0)
            jointClass->m_data.SetLimits(-1.0e10f, 1.0e10f);
        else
        {
            _ASSERTE(jiMin <= 0.0);
            _ASSERTE(jiMin + jiRange >= 0.0);
    //            jointClass->m_data.SetLimits(jiMin * linScaling, (jiMin + jiRange) * linScaling);
            jointClass->m_data.SetLimits(jiMin-_newtonJointOffset,jiMin+jiRange-_newtonJointOffset);
        }
    }
}

void CConstraintDyn::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    int ctrlMode=_simGetJointDynCtrlMode(joint);
    sReal dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    csimNewtonRevoluteJoint* revJoint;
    csimNewtonPrismaticJoint* prismJoint;
    sReal e=0.0;
    if (jointType==sim_joint_revolute_subtype)
    {
        revJoint = (csimNewtonRevoluteJoint*)_newtonConstraint;
        if (ctrlMode>=sim_jointdynctrl_position)
        {
            if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
                e=getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint), getRevoluteJointAngle());
            else
                e=_simGetDynamicMotorTargetPosition(joint)-getRevoluteJointAngle();
        }
    }
    else
    {
        prismJoint = (csimNewtonPrismaticJoint*)_newtonConstraint;
        if (ctrlMode>=sim_jointdynctrl_position)
            e=_simGetDynamicMotorTargetPosition(joint)-getPrismaticJointPosition();
    }

    int auxV=0;
    if (_dynPassCount==0)
        auxV|=1;
    int inputValuesInt[5]={0,0,0,0,0};
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    sReal inputValuesFloat[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    if (jointType==sim_joint_revolute_subtype)
        inputValuesFloat[0]=getRevoluteJointAngle();
    else
       inputValuesFloat[0]=getPrismaticJointPosition();
    inputValuesFloat[1]=_lastEffortOnJoint;
    inputValuesFloat[2]=dynStepSize;
    inputValuesFloat[3]=e;
    // When also providing vel and accel info from engine:
    //inputValuesFloat[4]=currentVel;
    //inputValuesFloat[5]=currentAccel;
    // auxV|=2+4; // 2: vel, 4: accel
    sReal outputValues[5];
    int res=_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    sReal velocityToApply=outputValues[0];
    sReal forceToApply=fabs(outputValues[1]);
    if ((res&2)==0)
    { // motor is not locked
        if (jointType==sim_joint_revolute_subtype)
            revJoint->m_data.SetMotor((res&1)==1,velocityToApply, forceToApply);
        else
            prismJoint->m_data.SetMotor((res&1)==1,velocityToApply, forceToApply);
        _targetPositionToHoldAtZeroVelOn_velocityMode=false;
    }
    else
    { // motor is locked
        if (jointType==sim_joint_revolute_subtype)
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode = ((CustomHinge*)_newtonConstraint)->GetJointAngle();
    //            _targetPositionToHoldAtZeroVel_velocityMode = getRevoluteJointAngle()-_newtonJointOffset;
            _targetPositionToHoldAtZeroVelOn_velocityMode = true;
            revJoint->m_data.LockOnTargetPosition(_targetPositionToHoldAtZeroVel_velocityMode);
        }
        else
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode = getPrismaticJointPosition()-_newtonJointOffset;
            _targetPositionToHoldAtZeroVelOn_velocityMode = true;
            prismJoint->m_data.LockOnTargetPosition(_targetPositionToHoldAtZeroVel_velocityMode);
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=true;
    }
}

sReal CConstraintDyn::getPrismaticJointPosition() const
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))
    csimNewtonPrismaticJoint* const slider = (csimNewtonPrismaticJoint*) _newtonConstraint;
    return slider->GetJointPosit()+_newtonJointOffset;
}

sReal CConstraintDyn::getRevoluteJointAngle()
{
    sReal retVal=0.0;
    if (true)
    { // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
        sReal jointPos=0.0;

        CustomHinge* const hinge = (CustomHinge*) _newtonConstraint;
        jointPos = hinge->GetJointAngle();
        _lastJointPosSet = false;
        retVal = jointPos + _newtonJointOffset;
//        printf("%f (offset: %f)\n",retVal*180.0/piValue,_newtonJointOffset*180.0/piValue);
    }
    return(retVal);
}

sReal CConstraintDyn::getRevoluteJointAngle_forCoppeliaSim()
{
    return(getRevoluteJointAngle());
}

void CConstraintDyn::reportStateToCoppeliaSim(sReal simulationTime,int currentPass,int totalPasses)
{
    CConstraintDyn_base::reportStateToCoppeliaSim(simulationTime,currentPass,totalPasses);
    int totalPassesCount=0;
    if (currentPass==totalPasses-1)
        totalPassesCount=totalPasses;
    if (_jointHandle!=-1)
    {
        // Now report forces and torques acting on the joint:
        sReal forceOrTorque=0.0;

        if (_simGetJointType(_joint)==sim_joint_revolute_subtype)
        {
            csimNewtonRevoluteJoint* const jointClass = (csimNewtonRevoluteJoint*) _newtonConstraint;
            forceOrTorque = jointClass->m_data.GetJointForce();
        }
        else if (_simGetJointType(_joint)==sim_joint_prismatic_subtype)
        {
            csimNewtonPrismaticJoint* const jointClass = (csimNewtonPrismaticJoint*) _newtonConstraint;
            forceOrTorque = jointClass->m_data.GetJointForce();
        }
        _lastEffortOnJoint=forceOrTorque;
        _simAddJointCumulativeForcesOrTorques(_joint,forceOrTorque,totalPassesCount,simulationTime);
    }
    if (_forceSensorHandle!=-1)
    {
        // Report force/torque here, but do NOT report the sensor's intrinsic pose error:
        C3Vector forces;
        forces.clear();
        C3Vector torques;
        torques.clear();

        csimNewtonForceSensorJoint* const sensor = (csimNewtonForceSensorJoint*) _newtonConstraint;
        forces = sensor->GetForce();
        torques = sensor->GetTorque();

        _simAddForceSensorCumulativeForcesAndTorques(_forceSensor,forces.data,torques.data,totalPassesCount,simulationTime);
        if (totalPassesCount>0)
            _setForceSensorBrokenUnbrokenConstraints_newton();
    }
}

void CConstraintDyn::_setForceSensorBrokenUnbrokenConstraints_newton()
{
    csimNewtonForceSensorJoint* const joint = (csimNewtonForceSensorJoint*)_newtonConstraint;
    joint->m_broken = false;
}

CRigidBodyDyn* CConstraintDyn::_getChild() const
{
    return _bodyB;
}

CRigidBodyDyn* CConstraintDyn::_getParent() const
{
    return _bodyA;
}


bool CConstraintDyn::_isAcyclicJoint() const
{
    return _isAcyclic;
}

bool CConstraintDyn::getNewtonDependencyInfo(int& linkedJoint,sReal& fact,sReal& off)
{
    if (_newtonDependencyJointId==-1)
        return(false);
    linkedJoint=_newtonDependencyJointId;
    fact=_newtonDependencyFact;
    off=_newtonDependencyOff;
    _newtonDependencyJointId=-1; // to indicate that it was processed
    return(true);
}

CustomJoint* CConstraintDyn::_getNewtonJoint() const
{
    return _newtonConstraint;
}

void CConstraintDyn::_notifySekeletonRebuild()
{
    CRigidBodyContainerDyn* const rigidBodyContainerDyn = (CRigidBodyContainerDyn*) NewtonWorldGetUserData(NewtonBodyGetWorld (_newtonConstraint->GetBody0()));
    rigidBodyContainerDyn->_notifySekeletonRebuild();
}

void CConstraintDyn::_setNewtonParameters(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    sReal floatParams[2];
    int intParams[2];
    int parVer=0;
    _simGetNewtonParameters(joint,&parVer,floatParams,intParams);

    const sReal dependencyFactor=floatParams[0];
    const sReal dependencyOffset=floatParams[1];

    const int dependencyJointA_ID=intParams[0];
    const int dependencyJointB_ID=intParams[1]; // -1 if no dependent joint

    if (jointType==sim_joint_revolute_subtype)
    {
        // TODO_NEWTON:
    }

    if (jointType==sim_joint_prismatic_subtype)
    {
        // TODO_NEWTON:
    }

    if (jointType==sim_joint_spherical_subtype)
    {
        // TODO_NEWTON:
    }

    // TODO_NEWTON:
    // Store information about a dependent joint here. Constraint creation for that happens in the _createDependenciesBetweenJoints
    _newtonDependencyJointId=dependencyJointB_ID;
    _newtonDependencyFact=dependencyFactor;
    _newtonDependencyOff=dependencyOffset;
}

