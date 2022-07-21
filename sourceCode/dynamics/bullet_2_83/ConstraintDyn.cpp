#include "ConstraintDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CConstraintDyn::CConstraintDyn()
{
}
CConstraintDyn::~CConstraintDyn()
{
    CRigidBodyContainerDyn::getDynWorld()->getWorld()->removeConstraint(_constraint);
    delete _constraint;
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(joint,jtr2.X.data,jtr2.Q.data,false);

    C3X3Matrix m;
    m.setIdentity();
    m.buildYRotation(1.5707963267f);
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    { // in Bullet the moving direction is the x-axis (not like CoppeliaSim the z-axis)
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*m.getQuaternion();
    }
    m.setIdentity();
    m.buildYRotation(piValue);
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    { // in Bullet the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5f;
            jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
            _jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
        }
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
    }

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    btRigidBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getBtRigidBody();
    btRigidBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getBtRigidBody();

    if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        parentRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        childRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING
    float stopERP=simGetEngineFloatParameter(sim_bullet_joint_stoperp,-1,joint,nullptr);
    float stopCFM=simGetEngineFloatParameter(sim_bullet_joint_stopcfm,-1,joint,nullptr);
    float normalCFM=simGetEngineFloatParameter(sim_bullet_joint_normalcfm,-1,joint,nullptr);
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        btHingeConstraint* hinge;
        btTransform jtrA;
        jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
        jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
        btTransform jtrB;
        jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
        hinge=new btHingeConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
        _constraint=hinge;
        // You have to modify the btHingeConstraint code and remove the -pi;+pi limitation in the setLimit routine!!!

        hinge->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        hinge->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        hinge->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    {
        btSliderConstraint* slider;

        btTransform jtrA;
        jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
        jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));

        btTransform jtrB;
        jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));

        slider=new btSliderConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
        _constraint=slider;

        slider->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        slider->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        slider->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }
    if (_simGetJointType(joint)==sim_joint_spherical_subtype)
    {
        btPoint2PointConstraint* ballSocket;
        ballSocket=new btPoint2PointConstraint(*parentRigidBody,*childRigidBody,btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)),btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        _constraint=ballSocket;

        ballSocket->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        ballSocket->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        ballSocket->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }
    _constraint->setJointFeedback(&_bulletJointfeedbackStructure);
    _constraint->enableFeedback(true);
    bulletWorld->addConstraint(_constraint);
    handleMotorControl(joint,0,0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint,dummyA,dummyB);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);

    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(dummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_localTrA_2.getInverse());

    C3X3Matrix m;
    m.setIdentity();
    m.buildYRotation(1.5707963267f);
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    { // in Bullet the moving direction is the x-axis (not like CoppeliaSim the z-axis)
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*m.getQuaternion();
    }
    m.setIdentity();
    m.buildYRotation(piValue);
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    { // in Bullet the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5f;
            jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
            _jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
        }
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion()*m.getQuaternion();
    }

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    btRigidBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getBtRigidBody();
    btRigidBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getBtRigidBody();

    if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        parentRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        childRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING
    float stopERP=simGetEngineFloatParameter(sim_bullet_joint_stoperp,-1,joint,nullptr);
    float stopCFM=simGetEngineFloatParameter(sim_bullet_joint_stopcfm,-1,joint,nullptr);
    float normalCFM=simGetEngineFloatParameter(sim_bullet_joint_normalcfm,-1,joint,nullptr);
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        btHingeConstraint* hinge;
        btTransform jtrA;
        jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
        jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
        btTransform jtrB;
        jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
        hinge=new btHingeConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
        _constraint=hinge;

        // you have to modify the btHingeConstraint code and remove the -pi;+pi limitation in the setLimit routine!!!

        hinge->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        hinge->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        hinge->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    {
        btSliderConstraint* slider;

        btTransform jtrA;
        jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
        jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));

        btTransform jtrB;
        jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));

        slider=new btSliderConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);
        _constraint=slider;

        slider->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        slider->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        slider->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }
    if (_simGetJointType(joint)==sim_joint_spherical_subtype)
    {
        btPoint2PointConstraint* ballSocket;
        ballSocket=new btPoint2PointConstraint(*parentRigidBody,*childRigidBody,btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)),btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
        _constraint=ballSocket;

        ballSocket->setParam(BT_CONSTRAINT_STOP_ERP,stopERP);
        ballSocket->setParam(BT_CONSTRAINT_STOP_CFM,stopCFM);
        ballSocket->setParam(BT_CONSTRAINT_CFM,normalCFM);
    }

    _constraint->setJointFeedback(&_bulletJointfeedbackStructure);
    _constraint->enableFeedback(true);
    bulletWorld->addConstraint(_constraint);
    handleMotorControl(joint,0,0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,dummyA,dummyB);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector dtr,dtr2;
    _simGetObjectCumulativeTransformation(dummyA,dtr.X.data,dtr.Q.data,true);
    _simGetObjectCumulativeTransformation(dummyB,dtr2.X.data,dtr2.Q.data,true);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector dtrRelToBodyA;
    C7Vector dtrRelToBodyB;

    btRigidBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getBtRigidBody();
    btRigidBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getBtRigidBody();

    if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        parentRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    batr=bodyA->getInertiaFrameTransformation();
    dtrRelToBodyA=batr.getInverse()*dtr;

    if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        childRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    bbtr=bodyB->getInertiaFrameTransformation();
    dtrRelToBodyB=bbtr.getInverse()*dtr2;

    float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    dtrRelToBodyA.X*=linScaling; // ********** SCALING
    dtrRelToBodyB.X*=linScaling; // ********** SCALING

    btGeneric6DofConstraint* generalConstraint;
    btTransform dtrA;
    dtrA.setOrigin(btVector3(dtrRelToBodyA.X(0),dtrRelToBodyA.X(1),dtrRelToBodyA.X(2)));
    dtrA.setRotation(btQuaternion(dtrRelToBodyA.Q(1),dtrRelToBodyA.Q(2),dtrRelToBodyA.Q(3),dtrRelToBodyA.Q(0)));
    btTransform dtrB;
    dtrB.setOrigin(btVector3(dtrRelToBodyB.X(0),dtrRelToBodyB.X(1),dtrRelToBodyB.X(2)));
    dtrB.setRotation(btQuaternion(dtrRelToBodyB.Q(1),dtrRelToBodyB.Q(2),dtrRelToBodyB.Q(3),dtrRelToBodyB.Q(0)));
    generalConstraint=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,dtrA,dtrB,true);

    // Lock all axes (translations are locked by default):
    generalConstraint->getRotationalLimitMotor(0)->m_loLimit=0.0f;
    generalConstraint->getRotationalLimitMotor(0)->m_hiLimit=0.0f;
    generalConstraint->getRotationalLimitMotor(1)->m_loLimit=0.0f;
    generalConstraint->getRotationalLimitMotor(1)->m_hiLimit=0.0f;
    generalConstraint->getRotationalLimitMotor(2)->m_loLimit=0.0f;
    generalConstraint->getRotationalLimitMotor(2)->m_hiLimit=0.0f;

    _constraint=generalConstraint;
    _constraint->setJointFeedback(&_bulletJointfeedbackStructure);
    _constraint->enableFeedback(true);
    bulletWorld->addConstraint(_constraint);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(forceSensor,jtr2.X.data,jtr2.Q.data,false);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    btRigidBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getBtRigidBody();
    btRigidBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getBtRigidBody();

    if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        parentRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        childRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    btGeneric6DofConstraint* constr;
    btTransform jtrA;
    jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
    jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
    btTransform jtrB;
    jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
    jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
    constr=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);

    constr->setUseFrameOffset(false);

    _setForceSensorBrokenUnbrokenConstraints_bullet(constr);

    _constraint=constr;
    _constraint->setJointFeedback(&_bulletJointfeedbackStructure);
    _constraint->enableFeedback(true);
    bulletWorld->addConstraint(_constraint);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor,dummyA,dummyB);

    btDiscreteDynamicsWorld* bulletWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

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

    btRigidBody* parentRigidBody=((CRigidBodyDyn*)bodyA)->getBtRigidBody();
    btRigidBody* childRigidBody=((CRigidBodyDyn*)bodyB)->getBtRigidBody();

    if ((parentRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        parentRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

    if ((childRigidBody->getCollisionFlags()&(btCollisionObject::CF_KINEMATIC_OBJECT|btCollisionObject::CF_STATIC_OBJECT))==0)
        childRigidBody->setActivationState(DISABLE_DEACTIVATION); // this is a dynamic object
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    btGeneric6DofConstraint* constr;
    btTransform jtrA;
    jtrA.setOrigin(btVector3(jtrRelToBodyA.X(0),jtrRelToBodyA.X(1),jtrRelToBodyA.X(2)));
    jtrA.setRotation(btQuaternion(jtrRelToBodyA.Q(1),jtrRelToBodyA.Q(2),jtrRelToBodyA.Q(3),jtrRelToBodyA.Q(0)));
    btTransform jtrB;
    jtrB.setOrigin(btVector3(jtrRelToBodyB.X(0),jtrRelToBodyB.X(1),jtrRelToBodyB.X(2)));
    jtrB.setRotation(btQuaternion(jtrRelToBodyB.Q(1),jtrRelToBodyB.Q(2),jtrRelToBodyB.Q(3),jtrRelToBodyB.Q(0)));
    constr=new btGeneric6DofConstraint(*parentRigidBody,*childRigidBody,jtrA,jtrB,false);

    constr->setUseFrameOffset(false);

    _setForceSensorBrokenUnbrokenConstraints_bullet(constr);

    _constraint=constr;
    _constraint->setJointFeedback(&_bulletJointfeedbackStructure);
    _constraint->enableFeedback(true);
    bulletWorld->addConstraint(_constraint);
}

void CConstraintDyn::_updateJointLimits(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    if (jointType==sim_joint_revolute_subtype)
    {
        btHingeConstraint* hinge;
        hinge=(btHingeConstraint*)_constraint;
        if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
            hinge->setLimit(+1.0f,-1.0f); // no limits
        else
        { // Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems  (revolute joints only)
            if (_nonCyclicRevoluteJointPositionRange<=359.0f*piValue*2.0f/360.0f)
            { // when the range is <359, we keep the limits on all the time
                hinge->setLimit(-_nonCyclicRevoluteJointPositionRange*0.5f,+_nonCyclicRevoluteJointPositionRange*0.5f); // active limits. Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
            }
            else
            { // Bullet doesn't support a range > 360. So we manually turn limits on/off as needed:
                // That doesn't work in Bullet. We leave it unlimited for now:
                hinge->setLimit(+1.0f,-1.0f); // no limits
            }
        }
    }
    if (jointType==sim_joint_prismatic_subtype)
    {
        float linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
        float jiMin,jiRange;
        _simGetJointPositionInterval(joint,&jiMin,&jiRange);

        btSliderConstraint* slider;
        slider=(btSliderConstraint*)_constraint;
        slider->setLowerLinLimit(jiMin*linScaling);
        slider->setUpperLinLimit((jiMin+jiRange)*linScaling);
    }
}

void CConstraintDyn::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    int ctrlMode=_simGetJointDynCtrlMode(joint);
    float posScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    float forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
    float torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
    float linVelocityScaling=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();
    float dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    btHingeConstraint* hinge;
    btSliderConstraint* slider;
    float e=0.0f;
    if (jointType==sim_joint_revolute_subtype)
    {
        hinge=(btHingeConstraint*)_constraint;
        if (ctrlMode>=sim_jointdynctrl_position)
        {
            if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
                e=getAngleMinusAlpha(_simGetDynamicMotorTargetPosition(joint)+_nonCyclicRevoluteJointPositionOffset,getRevoluteJointAngle()); // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
            else
                e=_simGetDynamicMotorTargetPosition(joint)+_nonCyclicRevoluteJointPositionOffset-getRevoluteJointAngle(); // since 18/11/2012 we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
        }
    }
    else
    {
        slider=(btSliderConstraint*)_constraint;
        if (ctrlMode>=sim_jointdynctrl_position)
            e=_simGetDynamicMotorTargetPosition(joint)-getPrismaticJointPosition();
    }

    int auxV=0;
    if (_dynPassCount==0)
        auxV|=1;
    int inputValuesInt[5]={0,0,0,0,0};
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    float inputValuesFloat[7]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    if (jointType==sim_joint_revolute_subtype)
        inputValuesFloat[0]=getRevoluteJointAngle()-_nonCyclicRevoluteJointPositionOffset;
    else
       inputValuesFloat[0]=getPrismaticJointPosition();
    inputValuesFloat[1]=_lastEffortOnJoint;
    inputValuesFloat[2]=dynStepSize;
    inputValuesFloat[3]=e;
    // When also providing vel and accel info from engine:
    //inputValuesFloat[4]=currentVel;
    //inputValuesFloat[5]=currentAccel;
    // auxV|=2+4; // 2: vel, 4: accel
    float outputValues[5];
    int res=_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    float velocityToApply=outputValues[0];
    float forceToApply=fabs(outputValues[1]);
    if ((res&2)==0)
    { // motor is not locked
        if (jointType==sim_joint_revolute_subtype)
            hinge->enableAngularMotor((res&1)==1,velocityToApply,forceToApply*torqueScaling*dynStepSize);
        else
        {
            slider->setPoweredLinMotor((res&1)==1);
            slider->setTargetLinMotorVelocity(velocityToApply*linVelocityScaling);
            slider->setMaxLinMotorForce(forceToApply*forceScaling);// why? *dynStepSize*dynStepSize);
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=false;
    }
    else
    { // motor is locked
        if (jointType==sim_joint_revolute_subtype)
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=((btHingeConstraint*)_constraint)->getHingeAngle();
            hinge->setLimit(_targetPositionToHoldAtZeroVel_velocityMode,_targetPositionToHoldAtZeroVel_velocityMode);
            hinge->enableAngularMotor(false,0.0f,0.0f);
        }
        else
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=getPrismaticJointPosition();
            slider->setPoweredLinMotor(false); // motor off
            slider->setLowerLinLimit(_targetPositionToHoldAtZeroVel_velocityMode*posScaling);
            slider->setUpperLinLimit(_targetPositionToHoldAtZeroVel_velocityMode*posScaling);
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=true;
    }
}


dynReal CConstraintDyn::getPrismaticJointPosition() const
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))
    C7Vector p(_bodyA->getShapeFrameTransformation());
    C7Vector c(_bodyB->getShapeFrameTransformation());
    p=p*_localTrA;
    c=c*_localTrB;
    if (_dummyAHandle!=-1)
        c=c*_localTrA_2.getInverse(); // Non-regular case (looped) (bug correction on 2010/10/08)
    return((dynReal)((p.getInverse()*c.X)(2)));
}

dynReal CConstraintDyn::getRevoluteJointAngle()
{
    dynReal retVal=(dynReal)0.0;
    if (true)
    { // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
        dynReal jointPos=(dynReal)0.0;

        jointPos=((btHingeConstraint*)_constraint)->getHingeAngle();

        if (_jointIsCyclic)
        { // turn count not needed here
            retVal=jointPos;
        }
        else
        {
            if (_lastJointPosSet)
            {
                dynReal dx=jointPos-_lastJointPos;
                if (dx>=0.0)
                    dx=fmod(dx+piValue,piValTimes2)-piValue;
                else
                    dx=fmod(dx-piValue,piValTimes2)+piValue;
                _jointPosAlt+=dx;
                if (_jointPosAlt>=0.0)
                {
                    float jp=jointPos+piValue;
                    float jap=_jointPosAlt+piValue;
                    jap=jap-jp+piValue;
                    int cnt=int(jap/piValTimes2);
                    _jointPosAlt=float(cnt)*piValTimes2+jp-piValue;
                }
                else
                {
                    float jp=jointPos-piValue;
                    float jap=_jointPosAlt-piValue;
                    jap=jap-jp-piValue;
                    int cnt=int(jap/-piValTimes2);
                    _jointPosAlt=float(cnt)*-piValTimes2+jp+piValue;
                }
            }
            retVal=_jointPosAlt;
        }
        _lastJointPos=jointPos;
        _lastJointPosSet=true;
    }
    return(retVal);
}

dynReal CConstraintDyn::getRevoluteJointAngle_forCoppeliaSim()
{
    return(getRevoluteJointAngle()-_nonCyclicRevoluteJointPositionOffset);
}

void CConstraintDyn::reportStateToCoppeliaSim(float simulationTime,int currentPass,int totalPasses)
{
    CConstraintDyn_base::reportStateToCoppeliaSim(simulationTime,currentPass,totalPasses);
    int totalPassesCount=0;
    if (currentPass==totalPasses-1)
        totalPassesCount=totalPasses;
    if (_jointHandle!=-1)
    {
        float forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
        float torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
        // Now report forces and torques acting on the joint:
        float forceOrTorque=0.0f;

        float dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
        if (_simGetJointType(_joint)!=sim_joint_spherical_subtype)
        { // Spherical joints are not supported here
            if (_simGetJointType(_joint)==sim_joint_revolute_subtype)
                forceOrTorque=-_constraint->getAppliedImpulse()/(torqueScaling*dynStepSize);
            if (_simGetJointType(_joint)==sim_joint_prismatic_subtype)
            {
                btJointFeedback* jfb=_constraint->getJointFeedback();
                C3Vector absF(jfb->m_appliedForceBodyB.getX(),jfb->m_appliedForceBodyB.getY(),jfb->m_appliedForceBodyB.getZ());
                absF/=forceScaling; // ********** SCALING
                C7Vector bodyBAbsConf(_bodyB->getInertiaFrameTransformation());
                C7Vector parentShapeAbsConf(_bodyA->getShapeFrameTransformation());
                C7Vector sensorAbsConf(parentShapeAbsConf*_localTrA);
                C3Vector absCorrectionV(sensorAbsConf.X-bodyBAbsConf.X);
                C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
                C3Vector forces(sensorAbsInverseQ*(absF*-1.0f));
                forceOrTorque=forces(2);
            }
        }
        _lastEffortOnJoint=forceOrTorque;
        _simAddJointCumulativeForcesOrTorques(_joint,forceOrTorque,totalPassesCount,simulationTime);
    }
    if (_forceSensorHandle!=-1)
    {
        // Report force/torque here, but do NOT report the sensor's intrinsic pose error:
        float forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
        float torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
        C3Vector forces;
        forces.clear();
        C3Vector torques;
        torques.clear();

        btJointFeedback* jfb=_constraint->getJointFeedback();
        C3Vector absF(jfb->m_appliedForceBodyB.getX(),jfb->m_appliedForceBodyB.getY(),jfb->m_appliedForceBodyB.getZ());
        absF/=forceScaling; // ********** SCALING
        C3Vector absT(jfb->m_appliedTorqueBodyB.getX(),jfb->m_appliedTorqueBodyB.getY(),jfb->m_appliedTorqueBodyB.getZ());
        absT/=torqueScaling; // ********** SCALING
        C7Vector bodyBAbsConf(_bodyB->getInertiaFrameTransformation());
        C7Vector parentShapeAbsConf(_bodyA->getShapeFrameTransformation());
        C7Vector sensorAbsConf(parentShapeAbsConf*_localTrA);
        C3Vector absCorrectionV(sensorAbsConf.X-bodyBAbsConf.X);
        absT+=absF^absCorrectionV;
        C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
        forces=sensorAbsInverseQ*(absF*-1.0f);
        torques=sensorAbsInverseQ*(absT*-1.0f);

        _simAddForceSensorCumulativeForcesAndTorques(_forceSensor,forces.data,torques.data,totalPassesCount,simulationTime);
        if (totalPassesCount>0)
            _setForceSensorBrokenUnbrokenConstraints_bullet((btGeneric6DofConstraint*)_constraint);
    }
}

void CConstraintDyn::_setForceSensorBrokenUnbrokenConstraints_bullet(btGeneric6DofConstraint* bulletConstr)
{
    btTranslationalLimitMotor* m=bulletConstr->getTranslationalLimitMotor();
    btRotationalLimitMotor* r[3];
    r[0]=bulletConstr->getRotationalLimitMotor(0);
    r[1]=bulletConstr->getRotationalLimitMotor(1);
    r[2]=bulletConstr->getRotationalLimitMotor(2);

    for (size_t i=0;i<3;i++) // First translational constraints:
    {
        m->m_lowerLimit[i]=0.0f;
        m->m_upperLimit[i]=0.0f;
        m->m_enableMotor[i]=true;
        m->m_targetVelocity[i]=0.0f;
        m->m_maxMotorForce[i]=SIM_MAX_FLOAT;
    }

    for (size_t i=0;i<3;i++) // Now rotational constraints:
    {
        r[i]->m_loLimit=0.0f;
        r[i]->m_hiLimit=0.0f;
        r[i]->m_enableMotor=true;
        r[i]->m_targetVelocity=0.0f;
        r[i]->m_maxMotorForce=SIM_MAX_FLOAT;
    }
}

btTypedConstraint* CConstraintDyn::getBtTypedConstraint()
{
    return(_constraint);
}
