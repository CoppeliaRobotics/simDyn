#pragma once

#include "RigidBodyDyn_base.h"
#include "Newton.h"

class CRigidBodyDyn : public CRigidBodyDyn_base
{
public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    void init(CXShape* shape,bool forceStatic,bool forceNonRespondable);

    NewtonBody* getNewtonRigidBody() const;

    C7Vector getNewtonMatrix() const;
    static void TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    static void ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex);

    C7Vector getInertiaFrameTransformation();
    C7Vector getShapeFrameTransformation();
    void reportVelocityToShape(float simulationTime);
    void handleAdditionalForcesAndTorques();
    void handleKinematicBody_step(float t,float cumulatedTimeStep);
    void handleKinematicBody_end();

protected:    
    void _setNewtonParameters(CXShape* shape);

    NewtonBody* _newtonBody;
    C3Vector m_externForce;
    C3Vector m_externTorque;
    void* _newtonBodyUserData[5];// shapeHandle,this,stat. friction,kin. friction, restitution
    float _newtonStaticFriction;
    float _newtonKineticFriction;
    float _newtonRestitution;
    float _newtonLinearDrag;
    float _newtonAngularDrag;
    bool _newtonFastMoving;
};
