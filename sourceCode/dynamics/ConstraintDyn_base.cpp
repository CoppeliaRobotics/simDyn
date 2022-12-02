#include "ConstraintDyn_base.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CConstraintDyn_base::CConstraintDyn_base()
{
    _joint=nullptr;
    _forceSensor=nullptr;
    _dummyA=nullptr;
    _dummyB=nullptr;
    _nonCyclicRevoluteJointPositionOffset=0.0;
    _jointIsCyclic=false;
    _dummyAHandle=-1;
    _dummyBHandle=-1;
    _forceSensorHandle=-1;
    _jointHandle=-1;
    _lastEffortOnJoint=0.0;
    _dynPassCount=0;
    _lastJointPosSet=false;
    _targetPositionToHoldAtZeroVelOn_velocityMode=false;
}

CConstraintDyn_base::~CConstraintDyn_base()
{
}

void CConstraintDyn_base::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint)
{
    _joint=joint;
    _bodyA=bodyA;
    _bodyB=bodyB;
    _shapeAHandle=bodyA->getShapeHandle();
    _shapeBHandle=bodyB->getShapeHandle();
    _shapeA=(CXShape*)_simGetObject(_shapeAHandle);
    _shapeB=(CXShape*)_simGetObject(_shapeBHandle);
    _jointIsCyclic=!_simGetJointPositionInterval(joint,&_nonCyclicRevoluteJointPositionMinimum,&_nonCyclicRevoluteJointPositionRange);
    _jointHandle=_simGetObjectID(joint);

    // Following 2 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    // Really? Can probably be removed...
    bodyA->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());
    bodyB->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());

    _simGetObjectLocalTransformation(joint,_localTrA.X.data,_localTrA.Q.data,true);
    C7Vector tmpTr1;
    C7Vector tmpTr2;
    _simGetObjectCumulativeTransformation(bodyB->getShape(),tmpTr1.X.data,tmpTr1.Q.data,false);
    _simGetObjectCumulativeTransformation(joint,tmpTr2.X.data,tmpTr2.Q.data,false);
    _localTrB=tmpTr1.getInverse()*tmpTr2;
}

void CConstraintDyn_base::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXJoint* joint,CXDummy* dummyA,CXDummy* dummyB)
{
    _joint=joint;
    _dummyA=dummyA;
    _dummyB=dummyB;
    _bodyA=bodyA;
    _bodyB=bodyB;
    _shapeAHandle=bodyA->getShapeHandle();
    _shapeBHandle=bodyB->getShapeHandle();
    _shapeA=(CXShape*)_simGetObject(_shapeAHandle);
    _shapeB=(CXShape*)_simGetObject(_shapeBHandle);
    _jointIsCyclic=!_simGetJointPositionInterval(joint,&_nonCyclicRevoluteJointPositionMinimum,&_nonCyclicRevoluteJointPositionRange);
    _dummyAHandle=_simGetObjectID(dummyA);
    _dummyBHandle=_simGetObjectID(dummyB);
    _jointHandle=_simGetObjectID(joint);

    // Following 2 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    // Really? Can probably be removed...
    bodyA->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());
    bodyB->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());

    _simGetObjectLocalTransformation(joint,_localTrA.X.data,_localTrA.Q.data,true);
    _simGetObjectLocalTransformation(dummyA,_localTrA_2.X.data,_localTrA_2.Q.data,false);
    _simGetObjectLocalTransformation(dummyB,_localTrB.X.data,_localTrB.Q.data,false);
}

void CConstraintDyn_base::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXDummy* dummyA,CXDummy* dummyB)
{
    _dummyA=dummyA;
    _dummyB=dummyB;
    _bodyA=bodyA;
    _bodyB=bodyB;
    _shapeAHandle=bodyA->getShapeHandle();
    _shapeBHandle=bodyB->getShapeHandle();
    _shapeA=(CXShape*)_simGetObject(_shapeAHandle);
    _shapeB=(CXShape*)_simGetObject(_shapeBHandle);
    _dummyAHandle=_simGetObjectID(dummyA);
    _dummyBHandle=_simGetObjectID(dummyB);

    // Following 2 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    // Really? Can probably be removed...
    bodyA->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());
    bodyB->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());

    _simGetObjectLocalTransformation(dummyA,_localTrA.X.data,_localTrA.Q.data,true);
    _simGetObjectLocalTransformation(dummyB,_localTrB.X.data,_localTrB.Q.data,true);
}

void CConstraintDyn_base::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor)
{
    _forceSensor=forceSensor;
    _bodyA=bodyA;
    _bodyB=bodyB;
    _shapeAHandle=bodyA->getShapeHandle();
    _shapeBHandle=bodyB->getShapeHandle();
    _shapeA=(CXShape*)_simGetObject(_shapeAHandle);
    _shapeB=(CXShape*)_simGetObject(_shapeBHandle);
    _forceSensorHandle=_simGetObjectID(forceSensor);

    // Following 2 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    // Really? Can probably be removed...
    bodyA->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());
    bodyB->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());

    _simGetObjectLocalTransformation(forceSensor,_localTrA.X.data,_localTrA.Q.data,true);
    C7Vector tmpTr1,tmpTr2;
    _simGetObjectCumulativeTransformation(bodyB->getShape(),tmpTr1.X.data,tmpTr1.Q.data,false);
    _simGetObjectCumulativeTransformation(forceSensor,tmpTr2.X.data,tmpTr2.Q.data,false);
    _localTrB=tmpTr1.getInverse()*tmpTr2;
}

void CConstraintDyn_base::init(CRigidBodyDyn* bodyA,CRigidBodyDyn* bodyB,CXForceSensor* forceSensor,CXDummy* dummyA,CXDummy* dummyB)
{
    _forceSensor=forceSensor;
    _dummyA=dummyA;
    _dummyB=dummyB;
    _bodyA=bodyA;
    _bodyB=bodyB;
    _shapeAHandle=bodyA->getShapeHandle();
    _shapeBHandle=bodyB->getShapeHandle();
    _shapeA=(CXShape*)_simGetObject(_shapeAHandle);
    _shapeB=(CXShape*)_simGetObject(_shapeBHandle);
    _dummyAHandle=_simGetObjectID(dummyA);
    _dummyBHandle=_simGetObjectID(dummyB);
    _forceSensorHandle=_simGetObjectID(forceSensor);

    // Following 2 lines are important when a joint is added during simulation for instance. At that time the shape's positions are not yet updated (this is called before stepSimulation and shape update)
    // Really? Can probably be removed...
    bodyA->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());
    bodyB->reportConfigurationToShape(CRigidBodyContainerDyn::getDynWorld()->getSimulationTime());

    _simGetObjectLocalTransformation(forceSensor,_localTrA.X.data,_localTrA.Q.data,true);
    _simGetObjectLocalTransformation(dummyA,_localTrA_2.X.data,_localTrA_2.Q.data,false);
    _simGetObjectLocalTransformation(dummyB,_localTrB.X.data,_localTrB.Q.data,false);
}

void CConstraintDyn_base::_updateJointLimits(CXJoint* joint)
{
}

void CConstraintDyn_base::_handleJoint(CXJoint* joint,int passCnt,int totalPasses)
{
}

double CConstraintDyn_base::getPrismaticJointPosition() const
{
    return(0.0);
}

double CConstraintDyn_base::getRevoluteJointAngle()
{
    return(0.0);
}

double CConstraintDyn_base::getRevoluteJointAngle_forCoppeliaSim()
{
    return(0.0);
}

void CConstraintDyn_base::reportStateToCoppeliaSim(double simulationTime,int currentPass,int totalPasses)
{
    if (_jointHandle!=-1)
    {
        // Following doesn't change throughout the simulation, but enforce it anyways:
        _simSetObjectLocalTransformation(_joint,_localTrA.X.data,_localTrA.Q.data,simulationTime);
        if (_dummyA!=nullptr)
        { // special case (looped)
            _simSetObjectLocalTransformation(_dummyA,_localTrA_2.X.data,_localTrA_2.Q.data,simulationTime);
            _simSetObjectLocalTransformation(_dummyB,_localTrB.X.data,_localTrB.Q.data,simulationTime);
        }

        // Report the joint's intrinsic pose (exclude the joint's intrinsic error pose):
        if (_simGetJointType(_joint)==sim_joint_revolute_subtype)
        {
            double angle=getRevoluteJointAngle_forCoppeliaSim();
            if (_jointIsCyclic)
                angle=atan2(sin(angle),cos(angle));
            _simSetDynamicMotorReflectedPositionFromDynamicEngine(_joint,angle,simulationTime);
        }
        if (_simGetJointType(_joint)==sim_joint_prismatic_subtype)
            _simSetDynamicMotorReflectedPositionFromDynamicEngine(_joint,getPrismaticJointPosition(),simulationTime);
        if (_simGetJointType(_joint)==sim_joint_spherical_subtype)
        { // a bit more troublesome here!
            C7Vector parentCumul(_bodyA->getShapeFrameTransformation());
            C7Vector childCumul(_bodyB->getShapeFrameTransformation());
            C7Vector p(parentCumul*_localTrA);
            C7Vector c;
            if (_dummyA!=nullptr)
                c=childCumul*_localTrB*_localTrA_2.getInverse(); // special case (looped)
            else
                c=childCumul*_localTrB; // regular case (non-looped)
            C7Vector x(p.getInverse()*c);
            _simSetJointSphericalTransformation(_joint,x.Q.data,simulationTime);
        }
    }
    if (_forceSensorHandle!=-1)
    {
        // Following doesn't change throughout the simulation, but enforce it anyways:
        _simSetObjectLocalTransformation(_forceSensor,_localTrA.X.data,_localTrA.Q.data,simulationTime);
        if (_dummyA!=nullptr)
        { // special case (looped)
            _simSetObjectLocalTransformation(_dummyA,_localTrA_2.X.data,_localTrA_2.Q.data,simulationTime);
            _simSetObjectLocalTransformation(_dummyB,_localTrB.X.data,_localTrB.Q.data,simulationTime);
        }
    }
    _dynPassCount++;
}

void CConstraintDyn_base::handleMotorControl(CXJoint* joint,int passCnt,int totalPasses)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:
    _updateJointLimits(joint);
    if (totalPasses!=0)
    { // execute this part not twice if the joint just got added
        // Now the control part:
        _handleJoint(joint,passCnt,totalPasses);
    }
}

int CConstraintDyn_base::getShapeAHandle() const
{
    return(_shapeAHandle);
}

int CConstraintDyn_base::getShapeBHandle() const
{
    return(_shapeBHandle);
}

bool CConstraintDyn_base::announceBodyWillBeDestroyed(int shapeHandle) const
{ // return value true means: this constraint should be removed
    return( (shapeHandle==_shapeAHandle)||(shapeHandle==_shapeBHandle) );
}

CXJoint* CConstraintDyn_base::getJoint() const
{
    return(_joint);
}

CXDummy* CConstraintDyn_base::getDummyA() const
{
    return(_dummyA);
}

CXDummy* CConstraintDyn_base::getDummyB() const
{
    return(_dummyB);
}

CXForceSensor* CConstraintDyn_base::getForceSensor() const
{
    return(_forceSensor);
}

CXSceneObject* CConstraintDyn_base::getJointOrForceSensor() const
{
    CXSceneObject* retVal=nullptr;
    if (_forceSensor!=nullptr)
        retVal=_forceSensor;
    if (_joint!=nullptr)
        retVal=_joint;
    return(retVal);
}

CXShape* CConstraintDyn_base::getShapeA() const
{
    return(_shapeA);
}

CXShape* CConstraintDyn_base::getShapeB() const
{
    return(_shapeB);
}

int CConstraintDyn_base::getJointHandle() const
{
    return(_jointHandle);
}

int CConstraintDyn_base::getDummyAHandle() const
{
    return(_dummyAHandle);
}

int CConstraintDyn_base::getDummyBHandle() const
{
    return(_dummyBHandle);
}

int CConstraintDyn_base::getForceSensorHandle() const
{
    return(_forceSensorHandle);
}

int CConstraintDyn_base::getJointOrForceSensorHandle() const
{
    int retVal=-1;
    if (_forceSensorHandle!=-1)
        retVal=_forceSensorHandle;
    if (_jointHandle!=-1)
        retVal=_jointHandle;
    return(retVal);
}

int CConstraintDyn_base::getIdentifyingHandle() const
{
    int retVal=0;
    if (_jointHandle>=0)
        retVal=_jointHandle;
    else if (_forceSensorHandle>=0)
        retVal=_forceSensorHandle;
    else
        retVal=std::min<int>(_dummyAHandle,_dummyBHandle);
    return(retVal);
}

double CConstraintDyn_base::getAngleMinusAlpha(double angle,double alpha)
{    // Returns angle-alpha. Angle and alpha are cyclic angles!!
    double sinAngle0 = sinf (angle);
    double sinAngle1 = sinf (alpha);
    double cosAngle0 = cosf(angle);
    double cosAngle1 = cosf(alpha);
    double sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
    double cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
    double angle_da = atan2(sin_da, cos_da);
    return angle_da;
}
