#pragma once

#include "ConstraintDyn_base.h"
#include "Newton.h"

class CustomJoint;

class CConstraintDyn : public CConstraintDyn_base
{
    class csimNewtonRevoluteJoint;
    class csimNewtonPrismaticJoint;
    class csimNewtonCommonJointData;
    class csimNewtonForceSensorJoint;
public:
    CConstraintDyn();
    virtual ~CConstraintDyn();
    // ShapeA -> joint -> shapeB
    void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint);
    // ShapeA -> joint -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB);
    // ShapeA -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB);
    // ShapeA -> force sensor -> shapeB
    void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor);
    // ShapeA -> force sensor -> dummyA - dummyB <- shapeB
    void init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB);

    CRigidBodyDyn* _getChild() const;
    CRigidBodyDyn* _getParent() const;
    CustomJoint* _getNewtonJoint() const;
    bool _isAcyclicJoint() const;
    void _setNewtonParameters(CXJoint* joint);
    bool getNewtonDependencyInfo(int& linkedJoint,double& fact, double& off);

    void reportStateToCoppeliaSim(double simulationTime,int currentPass,int totalPasses);
    double getPrismaticJointPosition() const; // important! The slider pos is not initialized when added (Bullet)!
    double getRevoluteJointAngle();
    double getRevoluteJointAngle_forCoppeliaSim();

protected:
    void _updateJointLimits(CXJoint* joint);
    void _handleJoint(CXJoint* joint,int passCnt,int totalPasses);

    void _notifySekeletonRebuild();
    void _setForceSensorBrokenUnbrokenConstraints_newton();
    CustomJoint* _newtonConstraint;
    bool _isAcyclic;
    double _newtonJointOffset;    // internally, we initialize Newton joints at 0, so this is the actual joint value when the joint is added
    int _newtonDependencyJointId;
    double _newtonDependencyFact;
    double _newtonDependencyOff;
};
