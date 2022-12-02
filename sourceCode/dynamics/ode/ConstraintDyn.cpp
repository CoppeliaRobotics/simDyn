#include "ConstraintDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CConstraintDyn::CConstraintDyn()
{
}
CConstraintDyn::~CConstraintDyn()
{
    dJointDestroy(_odeConstraint);
    delete _odeJointFeedbackStructure;
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint);

    dWorldID odeWorldID=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector jtr,jtr2;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);
    _simGetObjectCumulativeTransformation(joint,jtr2.X.data,jtr2.Q.data,false);

    C3X3Matrix m;
    m.buildYRotation(piValue);
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    { // in ODE the rotation direction is the negative z-axis (not like CoppeliaSim the z-axis)
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*m.getQuaternion();
    }
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    { // in ODE the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5;
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

    dBodyID parentRigidBody=((CRigidBodyDyn*)bodyA)->getOdeRigidBody();
    dBodyID childRigidBody=((CRigidBodyDyn*)bodyB)->getOdeRigidBody();

    //    if ((bodyA->isStatic())
    { // this is a dynamic object
        dBodySetAutoDisableFlag(parentRigidBody,0);
        dBodyEnable(parentRigidBody);
    }
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

//    if ((bodyB->isStatic())
    { // this is a dynamic object
        dBodySetAutoDisableFlag(childRigidBody,0);
        dBodyEnable(childRigidBody);
    }
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    jtr.X*=linScaling; // ********** SCALING
    jtr2.X*=linScaling; // ********** SCALING
    C3X3Matrix jtrm(jtr.Q);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double stopERP,stopCFM,bounce,fudge,normalCFM;
    _simGetJointOdeParameters(joint,&stopERP,&stopCFM,&bounce,&fudge,&normalCFM);
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        _odeConstraint=dJointCreateHinge(odeWorldID,0);
        dJointSetHingeParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetHingeParam(_odeConstraint,dParamBounce,bounce);
        dJointSetHingeParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetHingeParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetHingeParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);


        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetHingeAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));
        dJointSetHingeAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=new dJointFeedback;
        dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
    }
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    {
        _odeConstraint=dJointCreateSlider(odeWorldID,0);
        dJointSetSliderParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetSliderParam(_odeConstraint,dParamBounce,bounce);
        dJointSetSliderParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetSliderParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetSliderParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetSliderAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=new dJointFeedback;
        dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
    }
    if (_simGetJointType(joint)==sim_joint_spherical_subtype)
    {
        _odeConstraint=dJointCreateBall(odeWorldID,0);
        dJointSetBallParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetBallParam(_odeConstraint,dParamBounce,bounce);
        dJointSetBallParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetBallParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetBallParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetBallAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=nullptr;
    }
    handleMotorControl(joint,0,0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,joint,dummyA,dummyB);

    dWorldID odeWorldID=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    C7Vector jtr;
    _simGetObjectCumulativeTransformation(joint,jtr.X.data,jtr.Q.data,true);

    // Following is DIFFERENT from the regular situation (non-looped):
    C7Vector tmpTr1;
    _simGetObjectCumulativeTransformation(dummyB,tmpTr1.X.data,tmpTr1.Q.data,false);
    C7Vector jtr2(tmpTr1*_localTrA_2.getInverse());


    C3X3Matrix m;
    m.setIdentity();
    m.buildYRotation(piValue);
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    { // in ODE the rotation direction is the negative z-axis (not like CoppeliaSim the z-axis)
        jtr.Q=jtr.Q*m.getQuaternion();
        jtr2.Q=jtr2.Q*m.getQuaternion();
    }
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    { // in ODE the moving direction is the negative z-axis (not like CoppeliaSim the z-axis)
        C3X3Matrix jointOffsetThing;
        jointOffsetThing.setIdentity();
        if (_simGetJointPositionInterval(joint,nullptr,nullptr))
        { // We are using an offset between CoppeliaSim joint position and Bullet/ODE joint position, since low/high rev. limits are symmetric with those engines
            _nonCyclicRevoluteJointPositionOffset=-_nonCyclicRevoluteJointPositionMinimum-_nonCyclicRevoluteJointPositionRange*0.5;
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

    dBodyID parentRigidBody=((CRigidBodyDyn*)bodyA)->getOdeRigidBody();
    dBodyID childRigidBody=((CRigidBodyDyn*)bodyB)->getOdeRigidBody();

    //    if ((bodyA->isStatic())
    { // this is a dynamic object
        dBodySetAutoDisableFlag(parentRigidBody,0);
        dBodyEnable(parentRigidBody);
    }
    batr=bodyA->getInertiaFrameTransformation();
    jtrRelToBodyA=batr.getInverse()*jtr;

//    if ((bodyB->isStatic())
    { // this is a dynamic object
        dBodySetAutoDisableFlag(childRigidBody,0);
        dBodyEnable(childRigidBody);
    }
    bbtr=bodyB->getInertiaFrameTransformation();
    jtrRelToBodyB=bbtr.getInverse()*jtr2;

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    jtr.X*=linScaling; // ********** SCALING
    jtr2.X*=linScaling; // ********** SCALING
    C3X3Matrix jtrm(jtr.Q);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double stopERP,stopCFM,bounce,fudge,normalCFM;
    _simGetJointOdeParameters(joint,&stopERP,&stopCFM,&bounce,&fudge,&normalCFM);

    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
    {
        _odeConstraint=dJointCreateHinge(odeWorldID,0);
        dJointSetHingeParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetHingeParam(_odeConstraint,dParamBounce,bounce);
        dJointSetHingeParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetHingeParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetHingeParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);


        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetHingeAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));
        dJointSetHingeAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=new dJointFeedback;
        dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
    }
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
    {
        _odeConstraint=dJointCreateSlider(odeWorldID,0);
        dJointSetSliderParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetSliderParam(_odeConstraint,dParamBounce,bounce);
        dJointSetSliderParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetSliderParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetSliderParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetSliderAxis(_odeConstraint,jtrm.axis[2](0),jtrm.axis[2](1),jtrm.axis[2](2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        _odeJointFeedbackStructure=new dJointFeedback;
        dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
    }
    if (_simGetJointType(joint)==sim_joint_spherical_subtype)
    {
        _odeConstraint=dJointCreateBall(odeWorldID,0);
        dJointSetBallParam(_odeConstraint,dParamFudgeFactor,fudge);
        dJointSetBallParam(_odeConstraint,dParamBounce,bounce);
        dJointSetBallParam(_odeConstraint,dParamCFM,normalCFM);
        dJointSetBallParam(_odeConstraint,dParamStopERP,stopERP);
        dJointSetBallParam(_odeConstraint,dParamStopCFM,stopCFM);

        // Set the configuration of the child as if the joint was at 0 position:
        dBodyID cb=childRigidBody;
        C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
        C7Vector alpha(jtr2.getInverse()*cb_a);
        C7Vector cb_b(jtr*alpha);
        dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
        dQuaternion dQ;
        dQ[0]=cb_b.Q.data[0];
        dQ[1]=cb_b.Q.data[1];
        dQ[2]=cb_b.Q.data[2];
        dQ[3]=cb_b.Q.data[3];
        dBodySetQuaternion(cb,dQ);

        // Attach the joint to the 2 bodies
        dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
        dJointSetBallAnchor(_odeConstraint,jtr.X(0),jtr.X(1),jtr.X(2));

        // Reset the configuration of the child as it is now:
        dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
        dQ[0]=cb_a.Q.data[0];
        dQ[1]=cb_a.Q.data[1];
        dQ[2]=cb_a.Q.data[2];
        dQ[3]=cb_a.Q.data[3];
        dBodySetQuaternion(cb,dQ);
        _odeJointFeedbackStructure=nullptr;
    }
    handleMotorControl(joint,0,0);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,dummyA,dummyB);

    dWorldID odeWorldID=CRigidBodyContainerDyn::getDynWorld()->getWorld();
    _odeJointFeedbackStructure=nullptr; // Only used with ODE force sensors or joints

    C7Vector dtr,dtr2;
    _simGetObjectCumulativeTransformation(dummyA,dtr.X.data,dtr.Q.data,true);
    _simGetObjectCumulativeTransformation(dummyB,dtr2.X.data,dtr2.Q.data,true);

    C7Vector batr;
    C7Vector bbtr;
    batr.setIdentity();
    bbtr.setIdentity();
    C7Vector dtrRelToBodyA;
    C7Vector dtrRelToBodyB;

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();

    dtr.X*=linScaling; // ********** SCALING
    dtr2.X*=linScaling; // ********** SCALING

    _odeConstraint=dJointCreateFixed(odeWorldID,0);

    dBodyID parentRigidBody=((CRigidBodyDyn*)bodyA)->getOdeRigidBody();
    dBodyID childRigidBody=((CRigidBodyDyn*)bodyB)->getOdeRigidBody();

    // Set the configuration of the child as if the dummies were overlapping:
    dBodyID cb=childRigidBody;
    C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
    C7Vector x(cb_a.getInverse()*dtr2);
    C7Vector cb_b(dtr*x.getInverse());
    dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
    dQuaternion dQ;
    dQ[0]=cb_b.Q.data[0];
    dQ[1]=cb_b.Q.data[1];
    dQ[2]=cb_b.Q.data[2];
    dQ[3]=cb_b.Q.data[3];
    dBodySetQuaternion(cb,dQ);

    // Attach the fixed joint to the 2 bodies:
    dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
    dJointSetFixed(_odeConstraint);

    // Reset the configuration of the child as it is now:
    dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
    dQ[0]=cb_a.Q.data[0];
    dQ[1]=cb_a.Q.data[1];
    dQ[2]=cb_a.Q.data[2];
    dQ[3]=cb_a.Q.data[3];
    dBodySetQuaternion(cb,dQ);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor);

    dWorldID odeWorldID=CRigidBodyContainerDyn::getDynWorld()->getWorld();

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

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    jtr.X*=linScaling; // ********** SCALING
    jtr2.X*=linScaling; // ********** SCALING
    C3X3Matrix jtrm(jtr.Q);

    dBodyID parentRigidBody=((CRigidBodyDyn*)bodyA)->getOdeRigidBody();
    dBodyID childRigidBody=((CRigidBodyDyn*)bodyB)->getOdeRigidBody();

    _odeConstraint=dJointCreateFixed(odeWorldID,0);
    dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
    dJointSetFixed(_odeConstraint);

    _odeJointFeedbackStructure=new dJointFeedback;
    dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
}

void CConstraintDyn::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB)
{
    CConstraintDyn_base::init(bodyA,bodyB,forceSensor,dummyA,dummyB);

    dWorldID odeWorldID=CRigidBodyContainerDyn::getDynWorld()->getWorld();

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

    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    jtrRelToBodyA.X*=linScaling; // ********** SCALING
    jtrRelToBodyB.X*=linScaling; // ********** SCALING

    jtr.X*=linScaling; // ********** SCALING
    jtr2.X*=linScaling; // ********** SCALING

    _odeConstraint=dJointCreateFixed(odeWorldID,0);

    dBodyID parentRigidBody=((CRigidBodyDyn*)bodyA)->getOdeRigidBody();
    dBodyID childRigidBody=((CRigidBodyDyn*)bodyB)->getOdeRigidBody();

    // Set the configuration of the child as if the joint was at 0 position:
    dBodyID cb=childRigidBody;
    C7Vector cb_a(C4Vector(((double*)dBodyGetQuaternion(cb))),C3Vector(((double*)dBodyGetPosition(cb))));
    C7Vector alpha(jtr2.getInverse()*cb_a);
    C7Vector cb_b(jtr*alpha);
    dBodySetPosition(cb,cb_b.X(0),cb_b.X(1),cb_b.X(2));
    dQuaternion dQ;
    dQ[0]=cb_b.Q.data[0];
    dQ[1]=cb_b.Q.data[1];
    dQ[2]=cb_b.Q.data[2];
    dQ[3]=cb_b.Q.data[3];
    dBodySetQuaternion(cb,dQ);

    // Attach the joint to the 2 bodies
    dJointAttach(_odeConstraint,parentRigidBody,childRigidBody);
    dJointSetFixed(_odeConstraint);

    // Reset the configuration of the child as it is now:
    dBodySetPosition(cb,cb_a.X(0),cb_a.X(1),cb_a.X(2));
    dQ[0]=cb_a.Q.data[0];
    dQ[1]=cb_a.Q.data[1];
    dQ[2]=cb_a.Q.data[2];
    dQ[3]=cb_a.Q.data[3];
    dBodySetQuaternion(cb,dQ);

    _odeJointFeedbackStructure=new dJointFeedback;
    dJointSetFeedback(_odeConstraint,_odeJointFeedbackStructure);
}

void CConstraintDyn::_updateJointLimits(CXJoint* joint)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    if (jointType==sim_joint_revolute_subtype)
    {
        if (_simGetJointPositionInterval(joint,nullptr,nullptr)==0)
        { // no limits
            dJointSetHingeParam(_odeConstraint,dParamLoStop,-dInfinity);
            dJointSetHingeParam(_odeConstraint,dParamHiStop,+dInfinity);
        }
        else
        {
            // Limits are symmetric since 18/11/2012, since we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems  (revolute joints only)
            if (_nonCyclicRevoluteJointPositionRange<=359.0*piValue*2.0/360.0)
            { // when the range is <359, we keep the limits on all the time
                dJointSetHingeParam(_odeConstraint,dParamLoStop,-_nonCyclicRevoluteJointPositionRange*0.5);
                dJointSetHingeParam(_odeConstraint,dParamHiStop,+_nonCyclicRevoluteJointPositionRange*0.5);
            }
            else
            { // since 23/3/2014: some rev. joints need to be limited to a range>360deg (e.g. for motion planning), so we keep it pseudo-cyclic on the dynamic side
                // ODE doesn't support a range > 360. So we manually turn limits on/off as needed:
                // That doesn't work in ODE. We leave it unlimited:
                // double a=getRevoluteJointAngle();
                if (false)//a<-_nonCyclicRevoluteJointPositionRange*0.5+3.14159265)
                {
                    double m=fmod(-_nonCyclicRevoluteJointPositionRange*0.5,3.14159265);
                    dJointSetHingeParam(_odeConstraint,dParamLoStop,m); // low limit on
                }
                else
                    dJointSetHingeParam(_odeConstraint,dParamLoStop,-dInfinity); // low limit off
                if (false)//a>_nonCyclicRevoluteJointPositionRange*0.5-3.14159265)
                    dJointSetHingeParam(_odeConstraint,dParamHiStop,+_nonCyclicRevoluteJointPositionRange*0.5); // high limit on
                else
                    dJointSetHingeParam(_odeConstraint,dParamHiStop,+dInfinity); // high limit off
            }
        }
    }
    else
    {
        double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
        double jiMin,jiRange;
        _simGetJointPositionInterval(joint,&jiMin,&jiRange);

        dJointSetSliderParam(_odeConstraint,dParamLoStop,jiMin*linScaling); // ********** SCALING
        dJointSetSliderParam(_odeConstraint,dParamHiStop,(jiMin+jiRange)*linScaling); // ********** SCALING
    }
}

void CConstraintDyn::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
    int jointType=_simGetJointType(joint);
    if (jointType==sim_joint_spherical_subtype)
        return;
    int ctrlMode=_simGetJointDynCtrlMode(joint);
    double posScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    double forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
    double torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
    double linVelocityScaling=CRigidBodyContainerDyn::getDynWorld()->getLinearVelocityScalingFactorDyn();
    double dynStepSize=CRigidBodyContainerDyn::getDynWorld()->getDynamicsInternalTimeStep();
    double e=0.0;
    if (jointType==sim_joint_revolute_subtype)
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
    double outputValues[5];
    int res=_simHandleJointControl(joint,auxV,inputValuesInt,inputValuesFloat,outputValues);
    double velocityToApply=outputValues[0];
    double forceToApply=fabs(outputValues[1]);
    if ((res&2)==0)
    { // motor is not locked
        if (jointType==sim_joint_revolute_subtype)
        {
            if ((res&1)==1)
            {
                dJointSetHingeParam(_odeConstraint,dParamFMax,forceToApply*torqueScaling); // ********** SCALING
                dJointSetHingeParam(_odeConstraint,dParamVel,velocityToApply);
            }
            else
                dJointSetHingeParam(_odeConstraint,dParamFMax,0.0); // Motor off
        }
        else
        {
            if ((res&1)==1)
            {
                dJointSetSliderParam(_odeConstraint,dParamFMax,forceToApply*forceScaling); // ********** SCALING
                dJointSetSliderParam(_odeConstraint,dParamVel,velocityToApply*linVelocityScaling); // ********** SCALING
            }
            else
                dJointSetSliderParam(_odeConstraint,dParamFMax,0.0); // Motor off
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=false;
    }
    else
    { // motor is locked
        if (jointType==sim_joint_revolute_subtype)
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=dJointGetHingeAngle(_odeConstraint);
    //            _targetPositionToHoldAtZeroVel_velocityMode=getRevoluteJointAngle()-_nonCyclicRevoluteJointPositionOffset;
            _targetPositionToHoldAtZeroVelOn_velocityMode=true;
            dJointSetHingeParam(_odeConstraint,dParamLoStop,_targetPositionToHoldAtZeroVel_velocityMode);
            dJointSetHingeParam(_odeConstraint,dParamHiStop,_targetPositionToHoldAtZeroVel_velocityMode);
            dJointSetHingeParam(_odeConstraint,dParamFMax,0.0); // Motor off
        }
        else
        {
            if (!_targetPositionToHoldAtZeroVelOn_velocityMode)
                _targetPositionToHoldAtZeroVel_velocityMode=getPrismaticJointPosition();
            _targetPositionToHoldAtZeroVelOn_velocityMode=true;
            dJointSetSliderParam(_odeConstraint,dParamFMax,0.0); // Motor off
            dJointSetSliderParam(_odeConstraint,dParamLoStop,_targetPositionToHoldAtZeroVel_velocityMode*posScaling); // ********** SCALING
            dJointSetSliderParam(_odeConstraint,dParamHiStop,_targetPositionToHoldAtZeroVel_velocityMode*posScaling); // ********** SCALING
        }
        _targetPositionToHoldAtZeroVelOn_velocityMode=true;
    }
}


double CConstraintDyn::getPrismaticJointPosition() const
{ // important! The slider pos is not initialized when added!
    return(dJointGetSliderPosition(_odeConstraint)/CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn());
}

double CConstraintDyn::getRevoluteJointAngle()
{
    double retVal=(double)0.0;
    if (true)
    { // Bullet and ODE do not take into account turn count. So we need to handle this manually here:
        double jointPos=(double)0.0;

        jointPos=dJointGetHingeAngle(_odeConstraint);
        
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
        double forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
        double torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
        // Now report forces and torques acting on the joint:
        double forceOrTorque=0.0;

        if (_simGetJointType(_joint)!=sim_joint_spherical_subtype)
        { // Spherical joints are not supported here!
            C3Vector absF(_odeJointFeedbackStructure->f2[0],_odeJointFeedbackStructure->f2[1],_odeJointFeedbackStructure->f2[2]);
            absF/=forceScaling; // ********** SCALING
            C3Vector absT(_odeJointFeedbackStructure->t2[0],_odeJointFeedbackStructure->t2[1],_odeJointFeedbackStructure->t2[2]);
            absT/=torqueScaling; // ********** SCALING
            C7Vector bodyBAbsConf(_bodyB->getInertiaFrameTransformation());

            C7Vector parentShapeAbsConf(_bodyA->getShapeFrameTransformation());
            C7Vector sensorAbsConf(parentShapeAbsConf*_localTrA);

            C3Vector absCorrectionV(sensorAbsConf.X-bodyBAbsConf.X);
            absT+=absF^absCorrectionV;

            C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
            C3Vector forces(sensorAbsInverseQ*(absF*-1.0));
            C3Vector torques(sensorAbsInverseQ*(absT*-1.0));
            if (_simGetJointType(_joint)==sim_joint_revolute_subtype)
                forceOrTorque=torques(2);
            if (_simGetJointType(_joint)==sim_joint_prismatic_subtype)
                forceOrTorque=forces(2);
        }
        _lastEffortOnJoint=forceOrTorque;
        _simAddJointCumulativeForcesOrTorques(_joint,forceOrTorque,totalPassesCount,simulationTime);
    }
    if (_forceSensorHandle!=-1)
    {
        // Report force/torque here, but do NOT report the sensor's intrinsic pose error:
        double forceScaling=CRigidBodyContainerDyn::getDynWorld()->getForceScalingFactorDyn();
        double torqueScaling=CRigidBodyContainerDyn::getDynWorld()->getTorqueScalingFactorDyn();
        C3Vector forces;
        forces.clear();
        C3Vector torques;
        torques.clear();

        C3Vector absF(_odeJointFeedbackStructure->f2[0],_odeJointFeedbackStructure->f2[1],_odeJointFeedbackStructure->f2[2]);
        absF/=forceScaling; // ********** SCALING
        C3Vector absT(_odeJointFeedbackStructure->t2[0],_odeJointFeedbackStructure->t2[1],_odeJointFeedbackStructure->t2[2]);
        absT/=torqueScaling; // ********** SCALING
        C7Vector bodyBAbsConf(_bodyB->getInertiaFrameTransformation());

        C7Vector parentShapeAbsConf(_bodyA->getShapeFrameTransformation());
        C7Vector sensorAbsConf(parentShapeAbsConf*_localTrA);

        C3Vector absCorrectionV(sensorAbsConf.X-bodyBAbsConf.X);
        absT+=absF^absCorrectionV;

        C4Vector sensorAbsInverseQ(sensorAbsConf.Q.getInverse());
        forces=sensorAbsInverseQ*(absF*-1.0);
        torques=sensorAbsInverseQ*(absT*-1.0);
        _simAddForceSensorCumulativeForcesAndTorques(_forceSensor,forces.data,torques.data,totalPassesCount,simulationTime);
    }
}
