#pragma once

#include "dummyClasses.h"
#include "RigidBodyDyn.h"
#include "7Vector.h"

class CConstraintDyn_base  
{
public:
    CConstraintDyn_base();
    virtual ~CConstraintDyn_base();

    // ShapeA -> joint -> shapeB
    virtual void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint);
    // ShapeA -> joint -> dummyA - dummyB <- shapeB
    virtual void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB);
    // ShapeA -> dummyA - dummyB <- shapeB
    virtual void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB);
    // ShapeA -> force sensor -> shapeB
    virtual void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor);
    // ShapeA -> force sensor -> dummyA - dummyB <- shapeB
    virtual void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB);


    virtual void reportStateToCoppeliaSim(float simulationTime,int currentPass,int totalPasses);
    virtual dynReal getPrismaticJointPosition() const; // important! The slider pos is not initialized when added (Bullet)!
    virtual dynReal getRevoluteJointAngle();
    virtual dynReal getRevoluteJointAngle_forCoppeliaSim(); // we are using an offset between CoppeliaSim joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)

    int getJointHandle() const;
    int getDummyAHandle() const;
    int getDummyBHandle() const;
    int getForceSensorHandle() const;
    int getJointOrForceSensorHandle() const;
    int getIdentifyingHandle() const;
    int getShapeAHandle() const;
    int getShapeBHandle() const;

    CXJoint* getJoint() const;
    CXDummy* getDummyA() const;
    CXDummy* getDummyB() const;
    CXForceSensor* getForceSensor() const;
    CXSceneObject* getJointOrForceSensor() const;
    CXShape* getShapeA() const;
    CXShape* getShapeB() const;

    bool announceBodyWillBeDestroyed(int shapeHandle) const;
    void handleMotorControl(CXJoint* joint,int passCnt,int totalPasses);

protected:
    virtual void _updateJointLimits(CXJoint* joint);
    virtual void _handleJoint(CXJoint* joint,int passCnt,int totalPasses);

    static dynReal getAngleMinusAlpha(dynReal angle,dynReal alpha);

    int _shapeAHandle;
    int _shapeBHandle;
    int _jointHandle;       // is the constraint's handle, if not -1
    int _forceSensorHandle; // is the constraint's handle, if not -1
    int _dummyAHandle;      // min(_dummyAHandle,_dummyBHandle) is the constraint0s handle, if _jointHandle and _forceSensorHandle == -1
    int _dummyBHandle;

    CXJoint* _joint;
    CXForceSensor* _forceSensor;
    CXDummy* _dummyA;
    CXDummy* _dummyB;
    CXShape* _shapeA;
    CXShape* _shapeB;

    CRigidBodyDyn* _bodyA;
    CRigidBodyDyn* _bodyB;

    C7Vector _localTrA;     // between ShapeA and its child
    C7Vector _localTrA_2;   // between joint/fSensor and dummyA
    C7Vector _localTrB;     // between ShapeB and its child, or inv(between parent of shapeB and shape B)

    dynReal _nonCyclicRevoluteJointPositionOffset;    // Since 18/11/2012: Needed to avoid problems with Bullet (or was it ODE?) with joint limits below/above -180/+180 degrees
                                                    // The offset allows to model Bullet and ODE joints that have symmetrical limits (i.e. +- xx degrees)
                                                    // So: CoppeliaSim Joint + _nonCyclicRevoluteJointPositionOffset = physics engine joint
    float _nonCyclicRevoluteJointPositionMinimum;    // those are a copy of CoppeliaSim's values. But we make sure that they don't change during simulation!
    float _nonCyclicRevoluteJointPositionRange;


    dynReal _lastEffortOnJoint;
    dynReal _lastJointPos; // used with Bullet and ODE in order to keep track of number of turns
    dynReal _jointPosAlt; // used with Bullet and ODE in order to keep track of number of turns
    bool _lastJointPosSet; // used with Bullet and ODE in order to keep track of number of turns
    int _dynPassCount;
    bool _jointIsCyclic;

    bool _targetPositionToHoldAtZeroVelOn_velocityMode;
    dynReal _targetPositionToHoldAtZeroVel_velocityMode;
};
