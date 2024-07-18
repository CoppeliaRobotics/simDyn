#include <ConstraintDyn.h>
#include <RigidBodyContainerDyn.h>
#include <simLib/simLib.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxHinge.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxBallAndSocket.h>
#include <Vx/VxConstraintFriction.h>
#include <VortexConvertUtil.h>

const int VortexHingeCoordinate = Vx::VxHinge::kAngularCoordinate;
const int VortexPrismCoordinate = Vx::VxPrismatic::kLinearCoordinate;

/*  option to use a lock and lock velocity (position-based constraint)
 *  instead of a motor (velocity-based constraint). In the case of a lock and a lock velocity,
 *  the same max force is applied as on the motor. The difference is that the motor enforces a position and
 *  a velocity. Much like a step motor. You may have some use for this. It could be an option in the constraint motor.
 *  In this case the lock position is increment by lock velocity*dt. For a gripper,
 *  if lock velocity is used, the controller should stop applying velocity once the object is grabbed since
 *  we don't want to lock position to move too far away from the current actuator position. For the moment I
 *  simply reset the lock position to the current position at each step to prevent this.
*/
const bool sVortexMotorLock = false;

/*  option to lock a motor when desired velocity is 0. active only if sVortexMotorLock=false. */
const bool sVortexAutoLock = false;

void PrintDebugConstraint(Vx::VxConstraint*_vortexConstraint, Vx::VxReal desiredVel, const char* str)
{
    /*
    Vx::VxReal cv = _vortexConstraint->getCoordinateVelocity(0);
    Vx::VxReal mv = _vortexConstraint->getMotorDesiredVelocity(0);
    Vx::VxReal pos = _vortexConstraint->getCoordinateCurrentPosition(0);
    Vx::VxReal ll = _vortexConstraint->getLimitPosition(0, Vx::VxConstraint::kLimitLower);
    Vx::VxReal ul = _vortexConstraint->getLimitPosition(0, Vx::VxConstraint::kLimitUpper);
    bool exc = _vortexConstraint->isLimitExceeded(0);
    */
}

CConstraintDyn::CConstraintDyn()
{
}
CConstraintDyn::~CConstraintDyn()
{
    if (_vortexConstraint->getUniverse())
        _vortexConstraint->getUniverse()->removeConstraint(_vortexConstraint);
    delete _vortexConstraint;
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint)
{ // This is a revolute, prismatic or spherical joint constraint, between 2 rigid bodies (i.e. body1 - joint - body2)
    CConstraintDyn_base::init(bodyA,bodyB,joint);

    // Revolute and prismatic joints can be motorized, and their torque/force measured
    Vx::VxUniverse* vortexWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _vortexDependencyJointId=-1;

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(joint,jtr2.X.data,jtr2.Q.data,false);

    if (_simGetJointType(joint)==sim_joint_revolute)
    {
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5;
            jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
            _jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
        }
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion();
    }

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    Vx::VxPart* parentRigidBody=((CRigidBodyDyn*)bodyA)->getVortexRigidBody();
    Vx::VxPart* childRigidBody=((CRigidBodyDyn*)bodyB)->getVortexRigidBody();

    //    if ((bodyA->isStatic())
    { // this is a dynamic object
        //dBodySetAutoDisableFlag(bodyA->getVortexRigidBody(),0);
        parentRigidBody->wakeDynamics(true);
    }
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

//    if ((bodyB->isStatic())
    { // this is a dynamic object
        //dBodySetAutoDisableFlag(bodyB->getVortexRigidBody(),0);
        childRigidBody->wakeDynamics(true);
    }
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    C3X3Matrix jtrm(jtr.Q);

    if (_simGetJointType(joint)==sim_joint_revolute)
    {
        _vortexConstraint= new Vx::VxHinge();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    if (_simGetJointType(joint)==sim_joint_prismatic)
    {
        _vortexConstraint= new Vx::VxPrismatic();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    if (_simGetJointType(joint)==sim_joint_spherical)
    {
        _vortexConstraint= new Vx::VxBallAndSocket();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    _setVortexParameters(joint);
    vortexWorld->addConstraint(_vortexConstraint);
    handleMotorControl(joint,0,0); // prepare joint limits and motorization of this joint
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB)
{ // This is a revolute, prismatic or spherical joint constraint, between 2 rigid bodies involved in a loop closure (i.e. body1 - joint - dummy1 - dummy2 - body2)

    CConstraintDyn_base::init(bodyA,bodyB,joint,dummyA,dummyB);

    // Revolute and prismatic joints can be motorized, and their torque/force measured
    Vx::VxUniverse* vortexWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _vortexDependencyJointId=-1;

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);

    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(dummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_localTrA_2.getInverse());


    if (_simGetJointType(joint)==sim_joint_revolute)
    {
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5;
            jointOffsetThing.buildZRotation(_nonCyclicRevoluteJointPositionOffset);
            _jointPosAlt=_simGetJointPosition(joint)+_nonCyclicRevoluteJointPositionOffset;
        }
        jtr2.Q=jtr2.Q*jointOffsetThing.getQuaternion();
    }

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector jtrRelToBodyA;
    C7Vector jtrRelToBodyB;

    Vx::VxPart* parentRigidBody=((CRigidBodyDyn*)bodyA)->getVortexRigidBody();
    Vx::VxPart* childRigidBody=((CRigidBodyDyn*)bodyB)->getVortexRigidBody();

    //    if ((bodyA->isStatic())
    { // this is a dynamic object
        //dBodySetAutoDisableFlag(bodyA->getVortexRigidBody(),0);
        parentRigidBody->wakeDynamics(true);
    }
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

//    if ((bodyB->isStatic())
    { // this is a dynamic object
        //dBodySetAutoDisableFlag(bodyB->getOdeRigidBody(),0);
        childRigidBody->wakeDynamics(true);
    }
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    C3X3Matrix jtrm(jtr.Q);

    if (_simGetJointType(joint)==sim_joint_revolute)
    {
        _vortexConstraint= new Vx::VxHinge();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    if (_simGetJointType(joint)==sim_joint_prismatic)
    {
        _vortexConstraint= new Vx::VxPrismatic();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    if (_simGetJointType(joint)==sim_joint_spherical)
    {
        _vortexConstraint= new Vx::VxBallAndSocket();

        // Set the configuration of the child as if the joint was at 0 position:
        Vx::VxPart* cb=childRigidBody;
        Vx::VxReal4 quat;
        cb->getOrientationQuaternion(quat);
        C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        cb->setPosition(C3Vector2VxVector3(cb_b.X));
        cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

        // Attach the joint to the 2 bodies
        _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
        _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

        // Reset the configuration of the child as it is now:
        cb->setPosition(C3Vector2VxVector3(cb_a.X));
        cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    }
    _setVortexParameters(joint);
    vortexWorld->addConstraint(_vortexConstraint);
    handleMotorControl(joint,0,0); // prepare joint limits and motorization of this joint
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB)
{ // This is a rigid link between 2 rigid bodies involved in a loop closure (i.e. body1 - dummy1 - dummy2 - body2)
    CConstraintDyn_base::init(bodyA,bodyB,dummyA,dummyB);

    Vx::VxUniverse* vortexWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _vortexDependencyJointId=-1;

    C7Vector dtr,dtr2;
    _simGetObjectCumulativeTransformation(dummyA,dtr.X.data,dtr.Q.data,true);
    _simGetObjectCumulativeTransformation(dummyB,dtr2.X.data,dtr2.Q.data,true);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector dtrRelToBodyA;
    C7Vector dtrRelToBodyB;

    _vortexConstraint= new Vx::VxHinge();

    Vx::VxPart* parentRigidBody=((CRigidBodyDyn*)bodyA)->getVortexRigidBody();
    Vx::VxPart* childRigidBody=((CRigidBodyDyn*)bodyB)->getVortexRigidBody();

    // Set the configuration of the child as if the dummies were overlapping:
    Vx::VxPart* cb=childRigidBody;
    Vx::VxReal4 quat;
    cb->getOrientationQuaternion(quat);
    C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
    C7Vector x(cb_a.getInverse()*dtr2);
    C7Vector cb_b(dtr*x.getInverse());
    cb->setPosition(C3Vector2VxVector3(cb_b.X));
    cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

    // Attach the joint to the 2 bodies
    _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(dtr.X), C3Vector2VxVector3(C3Vector::unitZVector), C3Vector2VxVector3(C3Vector::unitXVector));
    _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(dtr.X), C3Vector2VxVector3(C3Vector::unitZVector), C3Vector2VxVector3(C3Vector::unitXVector));

    // Reset the configuration of the child as it is now:
    cb->setPosition(C3Vector2VxVector3(cb_a.X));
    cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));
    _vortexConstraint->setControl(0, Vx::VxConstraint::kControlLocked);

   // Vx::VxInfo(0, "++++++++++++++++++++++++++++++++");
    vortexWorld->addConstraint(_vortexConstraint);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor)
{ // This is a force/torque sensor constraint (rigid), between 2 rigid bodies (i.e. body1 - force/torque sensor - body2)
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor);

    Vx::VxUniverse* vortexWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    _vortexDependencyJointId=-1;

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(forceSensor,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(forceSensor,jtr2.X.data,jtr2.Q.data,false);

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

    C3X3Matrix jtrm(jtr.Q);

//    const Vx::VxTransform& tm = bodyB->getVortexRigidBody()->getTransform();
//    _vortexConstraint= new Vx::VxHinge(bodyA->getVortexRigidBody(), bodyB->getVortexRigidBody(), tm.t(), tm.axis(0));

    _vortexConstraint= new Vx::VxHinge();

    Vx::VxPart* parentRigidBody=((CRigidBodyDyn*)bodyA)->getVortexRigidBody();
    Vx::VxPart* childRigidBody=((CRigidBodyDyn*)bodyB)->getVortexRigidBody();

    // Attach the joint to the 2 bodies and lock it:
    _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
    _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
    _vortexConstraint->setControl(0, Vx::VxConstraint::kControlLocked);

    vortexWorld->addConstraint(_vortexConstraint);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB)
{ // This is a force/torque sensor constraint, between 2 rigid bodies involved in a loop closure (i.e. body1 - force/torque sensor - dummy1 - dummy2 - body2)
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor,dummyA,dummyB);

    Vx::VxUniverse* vortexWorld=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _vortexDependencyJointId=-1;

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

    C3X3Matrix jtrm(jtr.Q);

    Vx::VxPart* parentRigidBody=((CRigidBodyDyn*)bodyA)->getVortexRigidBody();
    Vx::VxPart* childRigidBody=((CRigidBodyDyn*)bodyB)->getVortexRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    Vx::VxPart* cb=childRigidBody;
    Vx::VxReal4 quat;
    cb->getOrientationQuaternion(quat);
    C7Vector cb_a(C4Vector(quat),VxVector32C3Vector(cb->getTransform().t()));
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    cb->setPosition(C3Vector2VxVector3(cb_b.X));
    cb->setOrientationQuaternion(cb_b.Q(0), cb_b.Q(1), cb_b.Q(2), cb_b.Q(3));

    _vortexConstraint= new Vx::VxHinge();

    // Attach the joint to the 2 bodies and lock it:
    _vortexConstraint->setPartAndAttachment(0, parentRigidBody, C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));
    _vortexConstraint->setPartAndAttachment(1, childRigidBody,  C3Vector2VxVector3(jtr.X), C3Vector2VxVector3(jtrm.axis[2]*-1.0), C3Vector2VxVector3(jtrm.axis[0]*-1.0));

    _vortexConstraint->setControl(0, Vx::VxConstraint::kControlLocked);

    // Reset the configuration of the child as it is now:
    cb->setPosition(C3Vector2VxVector3(cb_a.X));
    cb->setOrientationQuaternion(cb_a.Q(0), cb_a.Q(1), cb_a.Q(2), cb_a.Q(3));

    vortexWorld->addConstraint(_vortexConstraint);
}

void CConstraintDyn::_updateJointLimits(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical)
        return;
    if (jointType==sim_joint_revolute)
    {
        if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
        { // no limits
            _vortexConstraint->setLimitsActive(VortexHingeCoordinate, false);
        }
        else
        {
            if (_nonCyclicRevoluteJointPositionRange<=359.0*piValue*2.0/360.0)
            { // when the range is <359, we keep the limits on all the time
                // Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems  (revolute joints only)
                _vortexConstraint->setLimitsActive(VortexHingeCoordinate, true);
                _vortexConstraint->setLimitPositions(VortexHingeCoordinate, -_nonCyclicRevoluteJointPositionRange*0.5, _nonCyclicRevoluteJointPositionRange*0.5);
            }
            else
            { // here we keep limits off too, to behave as ODE and Bullet:
                _vortexConstraint->setLimitsActive(VortexHingeCoordinate, false);
            }
        }
    }
    else
    {
        double jiMin,jiRange;
        _simGetJointPositionInterval(joint,&jiMin,&jiRange);

        _vortexConstraint->setLimitsActive(VortexPrismCoordinate, true);
        _vortexConstraint->setLimitPositions(VortexPrismCoordinate, jiMin, (jiMin+jiRange));
    }
}

void CConstraintDyn::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical)
        return;
    int ctrlMode=_simGetJointDynCtrlMode(joint);
    double dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    double e=0.0;
    if (jointType==sim_joint_revolute)
    {
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
        if (ctrlMode>=sim_jointdynctrl_position)
            e=_simGetDynamicMotorTargetPosition(joint)-getPrismaticJointPosition();
    }

    int auxV=0;
    if (_dynPassCount==0)
        auxV|=1;
    int inputValuesInt[5]={0,0,0,0,0};
    inputValuesInt[0]=passCnt;
    inputValuesInt[1]=totalPasses;
    double inputValuesFloat[7]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    if (jointType==sim_joint_revolute)
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
    double outputValues[5];
    int res=_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    double velocityToApply=outputValues[0];
    double forceToApply=fabs(outputValues[1]);
    if ((res&2)==0)
    { // motor is not locked
        if (jointType==sim_joint_revolute)
        {
            if ((res&1)==1)
            {
                if (sVortexMotorLock)
                {
                    _vortexConstraint->setControl(VortexHingeCoordinate, Vx::VxConstraint::kControlLocked);
                    _vortexConstraint->setLockPosition(VortexHingeCoordinate, _vortexConstraint->getCoordinateCurrentPosition(VortexPrismCoordinate));
                    _vortexConstraint->setLockVelocity(VortexHingeCoordinate, velocityToApply);
                    _vortexConstraint->setLockMaximumForce(VortexHingeCoordinate, forceToApply);
                }
                else if (sVortexAutoLock && fabs(velocityToApply) < Vx::VX_MEDIUM_EPSILON)
                {
                    _vortexConstraint->setLockPosition(VortexHingeCoordinate, _vortexConstraint->getCoordinateCurrentPosition(VortexHingeCoordinate));
                    _vortexConstraint->setControl(VortexHingeCoordinate, Vx::VxConstraint::kControlLocked);
                    _vortexConstraint->setLockMaximumForce(VortexHingeCoordinate, forceToApply);
                    _vortexConstraint->setLockVelocity(VortexHingeCoordinate, 0);
                }
                else
                {
                    _vortexConstraint->setMotorParameters(VortexHingeCoordinate, velocityToApply, forceToApply, 0.0);
                    _vortexConstraint->setControl(VortexHingeCoordinate, Vx::VxConstraint::kControlMotorized);
                }
            }
            else
                _vortexConstraint->setControl(VortexHingeCoordinate, Vx::VxConstraint::kControlFree);
        }
        else
        {
            if ((res&1)==1)
            {
                if (sVortexMotorLock)
                {
                    _vortexConstraint->setControl(VortexPrismCoordinate, Vx::VxConstraint::kControlLocked);
                    _vortexConstraint->setLockPosition(VortexPrismCoordinate, _vortexConstraint->getCoordinateCurrentPosition(VortexPrismCoordinate));
                    _vortexConstraint->setLockVelocity(VortexPrismCoordinate, velocityToApply);
                    _vortexConstraint->setLockMaximumForce(VortexPrismCoordinate, forceToApply);
                }
                else if (sVortexAutoLock && fabs(velocityToApply) < Vx::VX_MEDIUM_EPSILON)
                {
                    _vortexConstraint->setLockPosition(VortexPrismCoordinate, _vortexConstraint->getCoordinateCurrentPosition(VortexPrismCoordinate));
                    _vortexConstraint->setControl(VortexPrismCoordinate, Vx::VxConstraint::kControlLocked);
                    _vortexConstraint->setLockMaximumForce(VortexPrismCoordinate, forceToApply);
                    _vortexConstraint->setLockVelocity(VortexPrismCoordinate, 0);
                }
                else
                {
                    _vortexConstraint->setMotorParameters(VortexPrismCoordinate, velocityToApply, forceToApply);
                    _vortexConstraint->setControl(VortexPrismCoordinate, Vx::VxConstraint::kControlMotorized);
                }
            }
            else
                _vortexConstraint->setControl(VortexPrismCoordinate, Vx::VxConstraint::kControlFree);
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=false;
    }
    else
    { // motor is locked
        if (jointType==sim_joint_revolute)
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=(double)_vortexConstraint->recalculateCoordinateCurrentPosition(VortexPrismCoordinate);
    //            _targetPositionToHoldAtZeroVel_velocityMode=getRevoluteJointAngle()-_nonCyclicRevoluteJointPositionOffset;
            _targetPositionToHoldAtZeroVelOn_velocityMode=true;
            _vortexConstraint->setControl(VortexHingeCoordinate, Vx::VxConstraint::kControlFree);
            _vortexConstraint->setLimitsActive(VortexHingeCoordinate, true);
            _vortexConstraint->setLimitPositions(VortexHingeCoordinate,_targetPositionToHoldAtZeroVel_velocityMode,_targetPositionToHoldAtZeroVel_velocityMode);
        }
        else
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=getPrismaticJointPosition();
            _targetPositionToHoldAtZeroVelOn_velocityMode=true;
            _vortexConstraint->setControl(VortexPrismCoordinate, Vx::VxConstraint::kControlFree); // motor off
            _vortexConstraint->setLimitsActive(VortexPrismCoordinate, true);
            _vortexConstraint->setLimitPositions(VortexPrismCoordinate, _targetPositionToHoldAtZeroVel_velocityMode, _targetPositionToHoldAtZeroVel_velocityMode);
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=true;
    }
}



double CConstraintDyn::getPrismaticJointPosition() const
{ // important! The slider pos is not initialized when added! (at least in debug mode, it is not! (release it is I think))

    // note if the constraint was never stepped, this will return 0.
    // an alternative is to return _vortexConstraint->recalculateCoordinateCurrentPosition(VortexPrismCoordinate);
    // which is slower but works all the time
    return (double)_vortexConstraint->recalculateCoordinateCurrentPosition(VortexPrismCoordinate);
}

double CConstraintDyn::getRevoluteJointAngle()
{
    double retVal=(double)0.0;
    if (true)
    { // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
        double jointPos=(double)0.0;

        // note if the constraint was never stepped, this will return 0.
        // an alternative is to return _vortexConstraint->recalculateCoordinateCurrentPosition(VortexPrismCoordinate);
        // which is slower but works all the time
        //       return (double)_vortexConstraint->getCoordinateCurrentPosition(VortexHingeCoordinate);
        jointPos=(double)_vortexConstraint->recalculateCoordinateCurrentPosition(VortexPrismCoordinate);

        if (_jointIsCyclic)
        {
            jointPos=fmod(jointPos,6.2831853);
            if (jointPos<-3.1415926)
                jointPos+=6.2831853;
            else if (jointPos>+3.1415926)
                jointPos-=6.2831853;
        }
        
        if (_jointIsCyclic)
        { // turn count not needed here
            retVal=jointPos;
        }
        else
        {
            if (_lastJointPosSet)
            {
                double dx=jointPos-_lastJointPos;
                if (dx>=0.0)
                    dx=fmod(dx+piValue,piValT2)-piValue;
                else
                    dx=fmod(dx-piValue,piValT2)+piValue;
                _jointPosAlt+=dx;
                if (_jointPosAlt>=0.0)
                {
                    double jp=jointPos+piValue;
                    double jap=_jointPosAlt+piValue;
                    jap=jap-jp+piValue;
                    int cnt=int(jap/piValT2);
                    _jointPosAlt=double(cnt)*piValT2+jp-piValue;
                }
                else
                {
                    double jp=jointPos-piValue;
                    double jap=_jointPosAlt-piValue;
                    jap=jap-jp-piValue;
                    int cnt=int(jap/-piValT2);
                    _jointPosAlt=double(cnt)*-piValT2+jp+piValue;
                }
            }
            retVal=_jointPosAlt;
        }
        _lastJointPos=jointPos;
        _lastJointPosSet=true;
    }
    return(retVal);
}

double CConstraintDyn::getRevoluteJointAngle_forCoppeliaSim()
{
    return(getRevoluteJointAngle()-_nonCyclicRevoluteJointPositionOffset);
}

void CConstraintDyn::reportStateToCoppeliaSim(double simulationTime,int currentPass,int totalPasses)
{
    CConstraintDyn_base::reportStateToCoppeliaSim(simulationTime,currentPass,totalPasses);
    int totalPassesCount=0;
    if (currentPass==totalPasses-1)
        totalPassesCount=totalPasses;
    if (_jointHandle!=-1)
    {
        // Now report forces and torques acting on the joint:
        double forceOrTorque=0.0;

        // measure the force/torque in a revolute/prismatic joint:
        if (_simGetJointType(_joint)!=sim_joint_spherical)
        { // Spherical joints are not supported here!

            // CoppeliaSim supposes that the force and torque are measured at the constraint location, in ABSOLUTE coordinates!
            forceOrTorque = -_vortexConstraint->getCoordinateForce(0);
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
        // CoppeliaSim supposes that the force and torque are measured at the constraint location, in ABSOLUTE coordinates!
        Vx::VxVector3 force, torque;
        if (_vortexConstraint->getPart(1)->getControl() == Vx::VxPart::kControlDynamic)
        {
            _vortexConstraint->getPartForce(1, force);
            _vortexConstraint->getPartTorque(1, torque);
        }
        else
        {
            _vortexConstraint->getPartForce(0, force);
            _vortexConstraint->getPartTorque(0, torque);
        }

        C3Vector absF = VxVector32C3Vector(force);
        C3Vector absT = VxVector32C3Vector(torque);


        C7Vector parentShapeAbsConf(_bodyA->getShapeFrameTransformation());
        C7Vector sensorAbsConf(parentShapeAbsConf*_localTrA);

        if (_vortexConstraint->getPart(1)->getControl() == Vx::VxPart::kControlDynamic)
        {
            C7Vector bodyBAbsConf(_bodyB->getInertiaFrameTransformation());
            C3Vector absCorrectionV(sensorAbsConf.X-bodyBAbsConf.X);
            absT+=absF^absCorrectionV;
        }
        else
        {
            C7Vector bodyAAbsConf(_bodyA->getInertiaFrameTransformation());
            C3Vector absCorrectionV(sensorAbsConf.X-bodyAAbsConf.X);
            absT+=absF^absCorrectionV;
            absF*=-1.0;
            absT*=-1.0;
        }

        C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
        forces=sensorAbsInverseQ*(absF*-1.0);
        torques=sensorAbsInverseQ*(absT*-1.0);
        _simAddForceSensorCumulativeForcesAndTorques(_forceSensor,forces.data,torques.data,totalPassesCount,simulationTime);
    }
}

void CConstraintDyn::_setVortexParameters(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double floatParams[51];
    int intParams[7];
    _simGetVortexParameters(joint,3,floatParams,intParams);

    const double limit_lower_damping=getVortexUnsignedDouble(floatParams[0]);
    const double limit_upper_damping=getVortexUnsignedDouble(floatParams[1]);
    const double limit_lower_stiffness=getVortexUnsignedDouble(floatParams[2]);
    const double limit_upper_stiffness=getVortexUnsignedDouble(floatParams[3]);
    const double limit_lower_restitution=getVortexUnsignedDouble(floatParams[4]);
    const double limit_upper_restitution=getVortexUnsignedDouble(floatParams[5]);
    const double limit_lower_maxForce=getVortexUnsignedDouble(floatParams[6]);
    const double limit_upper_maxForce=getVortexUnsignedDouble(floatParams[7]);
    const double motorConstraint_friction_coeff=getVortexUnsignedDouble(floatParams[8]);
    const double motorConstraint_friction_maxForce=getVortexUnsignedDouble(floatParams[9]);
    const double motorConstraint_friction_loss=getVortexUnsignedDouble(floatParams[10]);
    const double P0_loss=getVortexUnsignedDouble(floatParams[11]);
    const double P0_stiffness=getVortexUnsignedDouble(floatParams[12]);
    const double P0_damping=getVortexUnsignedDouble(floatParams[13]);
    const double P0_friction_coeff=getVortexUnsignedDouble(floatParams[14]);
    const double P0_friction_maxForce=getVortexUnsignedDouble(floatParams[15]);
    const double P0_friction_loss=getVortexUnsignedDouble(floatParams[16]);
    const double P1_loss=getVortexUnsignedDouble(floatParams[17]);
    const double P1_stiffness=getVortexUnsignedDouble(floatParams[18]);
    const double P1_damping=getVortexUnsignedDouble(floatParams[19]);
    const double P1_friction_coeff=getVortexUnsignedDouble(floatParams[20]);
    const double P1_friction_maxForce=getVortexUnsignedDouble(floatParams[21]);
    const double P1_friction_loss=getVortexUnsignedDouble(floatParams[22]);
    const double P2_loss=getVortexUnsignedDouble(floatParams[23]);
    const double P2_stiffness=getVortexUnsignedDouble(floatParams[24]);
    const double P2_damping=getVortexUnsignedDouble(floatParams[25]);
    const double P2_friction_coeff=getVortexUnsignedDouble(floatParams[26]);
    const double P2_friction_maxForce=getVortexUnsignedDouble(floatParams[27]);
    const double P2_friction_loss=getVortexUnsignedDouble(floatParams[28]);
    const double A0_loss=getVortexUnsignedDouble(floatParams[29]);
    const double A0_stiffness=getVortexUnsignedDouble(floatParams[30]);
    const double A0_damping=getVortexUnsignedDouble(floatParams[31]);
    const double A0_friction_coeff=getVortexUnsignedDouble(floatParams[32]);
    const double A0_friction_maxForce=getVortexUnsignedDouble(floatParams[33]);
    const double A0_friction_loss=getVortexUnsignedDouble(floatParams[34]);
    const double A1_loss=getVortexUnsignedDouble(floatParams[35]);
    const double A1_stiffness=getVortexUnsignedDouble(floatParams[36]);
    const double A1_damping=getVortexUnsignedDouble(floatParams[37]);
    const double A1_friction_coeff=getVortexUnsignedDouble(floatParams[38]);
    const double A1_friction_maxForce=getVortexUnsignedDouble(floatParams[39]);
    const double A1_friction_loss=getVortexUnsignedDouble(floatParams[40]);
    const double A2_loss=getVortexUnsignedDouble(floatParams[41]);
    const double A2_stiffness=getVortexUnsignedDouble(floatParams[42]);
    const double A2_damping=getVortexUnsignedDouble(floatParams[43]);
    const double A2_friction_coeff=getVortexUnsignedDouble(floatParams[44]);
    const double A2_friction_maxForce=getVortexUnsignedDouble(floatParams[45]);
    const double A2_friction_loss=getVortexUnsignedDouble(floatParams[46]);
    //const double dependencyFact=double(floatParams[47]);
    //const double dependencyOff=double(floatParams[48]);
    //const double reservedForFutureExt1=double(floatParams[49]);
    //const double reservedForFutureExt2=double(floatParams[50]);

    const bool motorFrictionEnabled=((intParams[0]&1)!=0);
    const bool motorFrictionProportional=((intParams[0]&2)!=0);
    const bool P0_relaxation_enabled=((intParams[1]&1)!=0);
    const bool P1_relaxation_enabled=((intParams[1]&2)!=0);
    const bool P2_relaxation_enabled=((intParams[1]&4)!=0);
    const bool A0_relaxation_enabled=((intParams[1]&8)!=0);
    const bool A1_relaxation_enabled=((intParams[1]&16)!=0);
    const bool A2_relaxation_enabled=((intParams[1]&32)!=0);
    const bool P0_friction_enabled=((intParams[2]&1)!=0);
    const bool P1_friction_enabled=((intParams[2]&2)!=0);
    const bool P2_friction_enabled=((intParams[2]&4)!=0);
    const bool A0_friction_enabled=((intParams[2]&8)!=0);
    const bool A1_friction_enabled=((intParams[2]&16)!=0);
    const bool A2_friction_enabled=((intParams[2]&32)!=0);
    const bool P0_friction_proportional=((intParams[3]&1)!=0);
    const bool P1_friction_proportional=((intParams[3]&2)!=0);
    const bool P2_friction_proportional=((intParams[3]&4)!=0);
    const bool A0_friction_proportional=((intParams[3]&8)!=0);
    const bool A1_friction_proportional=((intParams[3]&16)!=0);
    const bool A2_friction_proportional=((intParams[3]&32)!=0);
    //const int thisJointId=intParams[4];
    // const int dependentJointId=intParams[5];
    //const int reservedForFutureExt3=intParams[6];

    // if relaxXX = false or if stiffA1 > angular/linear stiff in solverparameters,
    // parameters as set in the VxSolverParameters are used. The loss parameters is not used
    // for position-pased row (which is the case for all rows and coordinate for hinge and prismatic

    // for internal friction, if it is enabled, an additional row is introduced in the
    // system so it will slow down the simulation. The friction exist for constraint equations as well as for
    // coordinate. I suggest adding the option only for the coordinate for now.

    // The Vortex joint frame and the CoppeliaSim joint frames are mapped like following:
    // vortex_X --> CoppeliaSim_Z, vortex_Y --> CoppeliaSim_X, vortex_Z --> CoppeliaSim_Y

    int coord=-1;
    if (jointType==sim_joint_revolute)
    {
        coord=VortexHingeCoordinate;

        // Following line in order to have relaxations relative to the joint base, and not relative to the absolute frame:
        if (P0_relaxation_enabled || P1_relaxation_enabled || P2_relaxation_enabled) // the condition since 3/1/2014
            static_cast<Vx::VxHinge*>(_vortexConstraint)->setLinearAxisReferencePartIndex(Vx::VxHinge::kLinearAxisReferencePartIndex0);

        _vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintP0, P2_stiffness, P2_damping, P2_loss, P2_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintP1, P0_stiffness, P0_damping, P0_loss, P0_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintP2, P1_stiffness, P1_damping, P1_loss, P1_relaxation_enabled);
        //_vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintA0, A2_stiffness, A2_damping, A2_loss, A2_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintA1, A1_stiffness, A1_damping, A1_loss, A1_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxHinge::kConstraintA2, A0_stiffness, A0_damping, A0_loss, A0_relaxation_enabled);

        Vx::VxConstraintFriction* frP0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintP0);
        frP0->setEnabled(P2_friction_enabled);
        frP0->setProportional(P2_friction_proportional);
        frP0->setCoefficient(P2_friction_coeff);
        frP0->setMaxForce(P2_friction_maxForce);
        frP0->setLoss(P2_friction_loss);

        Vx::VxConstraintFriction* frP1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintP1);
        frP1->setEnabled(P0_friction_enabled);
        frP1->setProportional(P0_friction_proportional);
        frP1->setCoefficient(P0_friction_coeff);
        frP1->setMaxForce(P0_friction_maxForce);
        frP1->setLoss(P0_friction_loss);

        Vx::VxConstraintFriction* frP2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintP2);
        frP2->setEnabled(P1_friction_enabled);
        frP2->setProportional(P1_friction_proportional);
        frP2->setCoefficient(P1_friction_coeff);
        frP2->setMaxForce(P1_friction_maxForce);
        frP2->setLoss(P1_friction_loss);

        //Vx::VxConstraintFriction* frA0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintA0);
        //frA0->setEnabled(A2_friction_enabled);
        //frA0->setProportional(A2_friction_proportional);
        //frA0->setCoefficient(A2_friction_coeff);
        //frA0->setMaxForce(A2_friction_maxForce);
        //frA0->setLoss(A2_friction_loss);

        Vx::VxConstraintFriction* frA1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintA1);
        frA1->setEnabled(A1_friction_enabled);
        frA1->setProportional(A1_friction_proportional);
        frA1->setCoefficient(A1_friction_coeff);
        frA1->setMaxForce(A1_friction_maxForce);
        frA1->setLoss(A1_friction_loss);

        Vx::VxConstraintFriction* frA2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxHinge::kConstraintA2);
        frA2->setEnabled(A0_friction_enabled);
        frA2->setProportional(A0_friction_proportional);
        frA2->setCoefficient(A0_friction_coeff);
        frA2->setMaxForce(A0_friction_maxForce);
        frA2->setLoss(A0_friction_loss);
    }

    if (jointType==sim_joint_prismatic)
    {
        coord=VortexPrismCoordinate;

        //_vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintP0, P2_stiffness, P2_damping, P2_loss, P2_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintP1, P0_stiffness, P0_damping, P0_loss, P0_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintP2, P1_stiffness, P1_damping, P1_loss, P1_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintA0, A2_stiffness, A2_damping, A2_loss, A2_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintA1, A0_stiffness, A0_damping, A0_loss, A0_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxPrismatic::kConstraintA2, A1_stiffness, A1_damping, A1_loss, A1_relaxation_enabled);

        //Vx::VxConstraintFriction* frP0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintP0);
        //frP0->setEnabled(P2_friction_enabled);
        //frP0->setProportional(P2_friction_proportional);
        //frP0->setCoefficient(P2_friction_coeff);
        //frP0->setMaxForce(P2_friction_maxForce);
        //frP0->setLoss(P2_friction_loss);

        Vx::VxConstraintFriction* frP1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintP1);
        frP1->setEnabled(P0_friction_enabled);
        frP1->setProportional(P0_friction_proportional);
        frP1->setCoefficient(P0_friction_coeff);
        frP1->setMaxForce(P0_friction_maxForce);
        frP1->setLoss(P0_friction_loss);

        Vx::VxConstraintFriction* frP2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintP2);
        frP2->setEnabled(P1_friction_enabled);
        frP2->setProportional(P1_friction_proportional);
        frP2->setCoefficient(P1_friction_coeff);
        frP2->setMaxForce(P1_friction_maxForce);
        frP2->setLoss(P1_friction_loss);

        Vx::VxConstraintFriction* frA0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintA0);
        frA0->setEnabled(A2_friction_enabled);
        frA0->setProportional(A2_friction_proportional);
        frA0->setCoefficient(A2_friction_coeff);
        frA0->setMaxForce(A2_friction_maxForce);
        frA0->setLoss(A2_friction_loss);

        Vx::VxConstraintFriction* frA1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintA1);
        frA1->setEnabled(A0_friction_enabled);
        frA1->setProportional(A0_friction_proportional);
        frA1->setCoefficient(A0_friction_coeff);
        frA1->setMaxForce(A0_friction_maxForce);
        frA1->setLoss(A0_friction_loss);

        Vx::VxConstraintFriction* frA2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxPrismatic::kConstraintA2);
        frA2->setEnabled(A1_friction_enabled);
        frA2->setProportional(A1_friction_proportional);
        frA2->setCoefficient(A1_friction_coeff);
        frA2->setMaxForce(A1_friction_maxForce);
        frA2->setLoss(A1_friction_loss);
    }

    if (jointType==sim_joint_spherical)
    {

        // Following line in order to have relaxations relative to the joint base, and not relative to the absolute frame:
        if (P0_relaxation_enabled || P1_relaxation_enabled || P2_relaxation_enabled)
            static_cast<Vx::VxBallAndSocket*>(_vortexConstraint)->setLinearAxisReferencePartIndex(Vx::VxBallAndSocket::kLinearAxisReferencePartIndex0);

        _vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintP0, P2_stiffness, P2_damping, P2_loss, P2_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintP1, P0_stiffness, P0_damping, P0_loss, P0_relaxation_enabled);
        _vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintP2, P1_stiffness, P1_damping, P1_loss, P1_relaxation_enabled);
        //_vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintA0, A2_stiffness, A2_damping, A2_loss, A2_relaxation_enabled);
        //_vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintA1, A0_stiffness, A0_damping, A0_loss, A0_relaxation_enabled);
        //_vortexConstraint->setRelaxationParameters(Vx::VxBallAndSocket::kConstraintA2, A1_stiffness, A1_damping, A1_loss, A1_relaxation_enabled);

        Vx::VxConstraintFriction* frP0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintP0);
        frP0->setEnabled(P2_friction_enabled);
        frP0->setProportional(P2_friction_proportional);
        frP0->setCoefficient(P2_friction_coeff);
        frP0->setMaxForce(P2_friction_maxForce);
        frP0->setLoss(P2_friction_loss);

        Vx::VxConstraintFriction* frP1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintP1);
        frP1->setEnabled(P0_friction_enabled);
        frP1->setProportional(P0_friction_proportional);
        frP1->setCoefficient(P0_friction_coeff);
        frP1->setMaxForce(P0_friction_maxForce);
        frP1->setLoss(P0_friction_loss);

        Vx::VxConstraintFriction* frP2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintP2);
        frP2->setEnabled(P1_friction_enabled);
        frP2->setProportional(P1_friction_proportional);
        frP2->setCoefficient(P1_friction_coeff);
        frP2->setMaxForce(P1_friction_maxForce);
        frP2->setLoss(P1_friction_loss);

        //Vx::VxConstraintFriction* frA0 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintA0);
        //frA0->setEnabled(A2_friction_enabled);
        //frA0->setProportional(A2_friction_proportional);
        //frA0->setCoefficient(A2_friction_coeff);
        //frA0->setMaxForce(A2_friction_maxForce);
        //frA0->setLoss(A2_friction_loss);

        //Vx::VxConstraintFriction* frA1 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintA1);
        //frA1->setEnabled(A0_friction_enabled);
        //frA1->setProportional(A0_friction_proportional);
        //frA1->setCoefficient(A0_friction_coeff);
        //frA1->setMaxForce(A0_friction_maxForce);
        //frA1->setLoss(A0_friction_loss);

        //Vx::VxConstraintFriction* frA2 = _vortexConstraint->getConstraintEquationFriction(Vx::VxBallAndSocket::kConstraintA2);
        //frA2->setEnabled(A1_friction_enabled);
        //frA2->setProportional(A1_friction_proportional);
        //frA2->setCoefficient(A1_friction_coeff);
        //frA2->setMaxForce(A1_friction_maxForce);
        //frA2->setLoss(A1_friction_loss);
    }

    if (coord!=-1)
    {
        _vortexConstraint->setMotorLoss(coord,0.0); // Vortex default: 0.0
        _vortexConstraint->setLimitDamping(coord,Vx::VxConstraint::kLimitLower,limit_lower_damping); // Vortex default: 0.0
        _vortexConstraint->setLimitDamping(coord,Vx::VxConstraint::kLimitUpper,limit_upper_damping); // Vortex default: 0.0
        _vortexConstraint->setLimitStiffness(coord,Vx::VxConstraint::kLimitLower,limit_lower_stiffness); // Vortex default: 1.7e308
        _vortexConstraint->setLimitStiffness(coord,Vx::VxConstraint::kLimitUpper,limit_upper_stiffness); // Vortex default: 1.7e308
        _vortexConstraint->setLimitRestitution(coord,Vx::VxConstraint::kLimitLower,limit_lower_restitution); // Vortex default: 0.0
        _vortexConstraint->setLimitRestitution(coord,Vx::VxConstraint::kLimitUpper,limit_upper_restitution); // Vortex default: 0.0
        _vortexConstraint->setLimitMaximumForce(coord,Vx::VxConstraint::kLimitLower,limit_lower_maxForce); // Vortex default: 1.7e308
        _vortexConstraint->setLimitMaximumForce(coord,Vx::VxConstraint::kLimitUpper,limit_upper_maxForce); // Vortex default: 1.7e308

        // loss is equivalent of slip in the contact material
        // if loss < solverparameter linear or angular loss, the solverparameter value is used.
        Vx::VxConstraintFriction* fr = _vortexConstraint->getCoordinateFriction(coord);
        fr->setEnabled(motorFrictionEnabled);
        fr->setProportional(motorFrictionProportional);
        fr->setCoefficient(motorConstraint_friction_coeff); // used only if proportional = true
        fr->setMaxForce(motorConstraint_friction_maxForce); // used only if proportional = false
        fr->setLoss(motorConstraint_friction_loss);
    }

    // Store information about a dependent joint here. Constraint creation for that happens in the _createDependenciesBetweenJoints
    double off,mult;
    simGetJointDependency(_simGetObjectID(joint),&_vortexDependencyJointId,&off,&mult);
    if (_vortexDependencyJointId!=-1)
    {
        _vortexDependencyFact=mult;
        _vortexDependencyOff=off;
    }
}

bool CConstraintDyn::getVortexDependencyInfo(int& linkedJoint,double& fact, double& off)
{
    if (_vortexDependencyJointId==-1)
        return(false);
    linkedJoint=_vortexDependencyJointId;
    fact=_vortexDependencyFact;
    off=_vortexDependencyOff;
    _vortexDependencyJointId=-1; // to indicsate that it was processed
    return(true);
}

Vx::VxConstraint* CConstraintDyn::getVortexConstraint()
{
    return(_vortexConstraint);
}
