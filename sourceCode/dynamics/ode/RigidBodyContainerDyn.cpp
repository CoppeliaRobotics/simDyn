#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "ConstraintDyn.h"
#include <simLib/simLib.h>

// Modifications in ODE source:
// *******************************************************************************
// ODE_MARC_MOD1
// ODE_MARC_MOD2
// ODE_MARC_MOD3
// ODE_MARC_MOD4
// ODE_MARC_MOD5

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine=sim_physics_ode;
    _engineVersion=0;
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
    std::vector<CRigidBodyDyn*> allBodies;
    _getAllRigidBodies(allBodies);
    for (size_t i=0;i<allBodies.size();i++)
        _removeRigidBody(allBodies[i]->getShapeHandle());

    for (int i=0;i<_simGetObjectListSize(sim_handle_all);i++)
    {
        CXSceneObject* it=(CXSceneObject*)_simGetObjectFromIndex(sim_handle_all,i);
        _simSetDynamicSimulationIconCode(it,sim_dynamicsimicon_none);
    }

    _particleCont->removeAllParticles();
    dJointGroupEmpty(_odeContactGroup);
    dJointGroupDestroy(_odeContactGroup);
    dSpaceDestroy(_odeSpace);
    dWorldDestroy(_dynamicsWorld);
    dCloseODE();

    // Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
    _particleCont->removeAllObjects();
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams,intParams);

    dInitODE2(0);
    int customSeed=simGetEngineInt32Param(sim_ode_global_randomseed,-1,nullptr,nullptr);
    if (customSeed>=0)
        dRandSetSeed(customSeed);
    _dynamicsWorld=dWorldCreate();
    _odeSpace=dHashSpaceCreate(0);
    _odeContactGroup=dJointGroupCreate(0);
    dWorldSetQuickStepNumIterations(_dynamicsWorld,simGetEngineInt32Param(sim_ode_global_constraintsolvingiterations,-1,nullptr,nullptr)); // 20 is default
    dWorldSetCFM(_dynamicsWorld,simGetEngineFloatParam(sim_ode_global_cfm,-1,nullptr,nullptr)); // (0.00001 is default, is also CoppeliaSim default)
    dWorldSetERP (_dynamicsWorld,simGetEngineFloatParam(sim_ode_global_erp,-1,nullptr,nullptr)); // (0.2 is default, CoppeliaSim default is 0.6)
    dWorldSetAutoDisableFlag(_dynamicsWorld,1);
    dWorldSetAutoDisableAverageSamplesCount(_dynamicsWorld,10);
    dWorldSetMaxAngularSpeed(_dynamicsWorld,200.0);
    dWorldSetContactSurfaceLayer(_dynamicsWorld,(dReal)(0.0002*_positionScalingFactorDyn)); // (0.0 is default)
    return("");
}

std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    return("ODE v0.12");
}

void CRigidBodyContainerDyn::_odeCollisionCallbackStatic(void* data,dGeomID o1,dGeomID o2)
{ // this function is static and will call the corresponding function of the current object:
    ((CRigidBodyContainerDyn*)_dynWorld)->_odeCollisionCallback(data,o1,o2);
}

void CRigidBodyContainerDyn::_odeCollisionCallback(void* data,dGeomID o1,dGeomID o2)
{
    dBodyID b1=dGeomGetBody(o1);
    dBodyID b2=dGeomGetBody(o2);
    
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    { // This very probably not needed (only when we have spaces that contain other spaces?)
        dSpaceCollide2(o1,o2,data,&_odeCollisionCallbackStatic);
    }
    else
    {
        CXShape* shapeA=(CXShape*)_simGetObject((unsigned long long)dBodyGetData(b1));
        CXShape* shapeB=(CXShape*)_simGetObject((unsigned long long)dBodyGetData(b2));

        bool canCollide=false;
        dContact contact[64];
        // version,contactCount,contactMode
        int dataInt[3]={0,4,4+8+16+2048}; //dContactBounce|dContactSoftCFM|dContactApprox1|dContactSoftERP};
        //                    mu,mu2,bounce,bunce_vel,soft_erp,soft_cfm,motion1,motion2,motionN,slip1,slip2,fdir1x,fdir1y,fdir1z
        double dataFloat[14]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        int objID1;
        int objID2;
//        int contactMode=dContactBounce|dContactSoftCFM|dContactApprox1|dContactSoftERP;

        if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
        { // regular case (shape-shape)
            unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
            unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
            canCollide=(_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB))&&((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_respondable)!=0)&&((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_respondable)!=0);
            if (_simGetLastParentForLocalGlobalCollidable(shapeA)==_simGetLastParentForLocalGlobalCollidable(shapeB))
                canCollide=canCollide&&(collFA&collFB&0x00ff); // we are local
            else
                canCollide=canCollide&&(collFA&collFB&0xff00); // we are global
            if ( (_simIsShapeDynamicallyStatic(shapeA)||((_simGetTreeDynamicProperty(shapeA)&sim_objdynprop_dynamic)==0))&&
                (_simIsShapeDynamicallyStatic(shapeB)||((_simGetTreeDynamicProperty(shapeB)&sim_objdynprop_dynamic)==0)) )
                canCollide=false;
            if (canCollide)
            { // Ok, the two object have flags that make them respondable to each other
                CXGeomWrap* shapeAWrap=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(shapeA);
                CXGeomWrap* shapeBWrap=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(shapeB);

                // Following parameter retrieval is OLD. Use instead following functions:
                // - simGetEngineFloatParam
                // - simGetEngineInt32Param
                // - simGetEngineBoolParam
                int maxContactsA,maxContactsB;
                double frictionA,frictionB;
                double cfmA,cfmB;
                double erpA,erpB;
                _simGetOdeMaxContactFrictionCFMandERP(shapeAWrap,&maxContactsA,&frictionA,&cfmA,&erpA);
                _simGetOdeMaxContactFrictionCFMandERP(shapeBWrap,&maxContactsB,&frictionB,&cfmB,&erpB);
                dataInt[1]=(maxContactsA+maxContactsB)/2;
                if (dataInt[1]<1)
                    dataInt[1]=1;
                dataFloat[0]=frictionA*frictionB;
                dataFloat[4]=(erpA+erpB)/2.0;
                dataFloat[5]=(cfmA+cfmB)/2.0;
                objID1=_simGetObjectID(shapeA);
                objID2=_simGetObjectID(shapeB);
            }
        }
        else
        { // particle-shape or particle-particle case:
            CParticleObjectContainer_base* particleCont=CRigidBodyContainerDyn::getDynWorld()->getParticleCont();
            int dataA=(unsigned long long)dBodyGetData(b1);
            int dataB=(unsigned long long)dBodyGetData(b2);
            if ( (shapeA==nullptr)&&(shapeB==nullptr) )
            { // particle-particle case:
                CParticleObject_base* pa=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                CParticleObject_base* pb=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);

                if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
                {
                    canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
                    if (canCollide)
                    { // They can collide:
                        dataInt[1]=1;
                        dataFloat[0]=pa->parameters[2]*pb->parameters[2];
                        dataFloat[4]=(pa->parameters[3]*pb->parameters[3])*0.5;
                        dataFloat[5]=(pa->parameters[4]*pb->parameters[4])*0.5;
                        objID1=dataA;
                        objID2=dataB;
                    }
                }
            }
            else
            { // particle-shape case:
                CXShape* shape=nullptr;
                CParticleObject_base* particle=nullptr;
                if (shapeA!=nullptr)
                {
                    shape=shapeA;
                    objID1=_simGetObjectID(shapeA);
                }
                else
                {
                    particle=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                    objID1=dataA;
                }
                if (shapeB!=nullptr)
                {
                    shape=shapeB;
                    objID2=_simGetObjectID(shapeB);
                }
                else
                {
                    particle=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynamicParticlesIdStart(),false);
                    objID2=dataB;
                }

                if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                {
                    canCollide=_simIsShapeDynamicallyRespondable(shape)&&(_simGetDynamicCollisionMask(shape)&particle->getShapeRespondableMask()&0xff00)&&((_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable)!=0); // we are global
                    if (canCollide)
                    {
                        dataInt[1]=1;
                        CXGeomWrap* shapeWrap=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(shape);

                        // Following parameter retrieval is OLD. Use instead following functions:
                        // - simGetEngineFloatParam
                        // - simGetEngineInt32Param
                        // - simGetEngineBoolParam
                        int maxContacts;
                        double friction;
                        double cfm;
                        double erp;
                        _simGetOdeMaxContactFrictionCFMandERP(shapeWrap,&maxContacts,&friction,&cfm,&erp);
                        dataFloat[0]=friction*particle->parameters[2];
                        dataFloat[4]=(erp+particle->parameters[3])/2.0;
                        dataFloat[5]=(cfm+particle->parameters[4])/2.0;
                    }
                }
            }
        }

        if (canCollide)
        {
            bool canReallyCollide=(_simHandleCustomContact(objID1,objID2,sim_physics_ode,dataInt,dataFloat)!=0);
            if (canReallyCollide)
            {
                // dataInt[0] represents the version. with 0, we have 3 values in dataInt, and 14 values in dataFloat!
                int contactMode=0;
                if (dataInt[2]&1)
                    contactMode|=dContactMu2;
                if (dataInt[2]&2)
                    contactMode|=dContactFDir1;
                if (dataInt[2]&4)
                    contactMode|=dContactBounce;
                if (dataInt[2]&8)
                    contactMode|=dContactSoftERP;
                if (dataInt[2]&16)
                    contactMode|=dContactSoftCFM;
                if (dataInt[2]&32)
                    contactMode|=dContactMotion1;
                if (dataInt[2]&64)
                    contactMode|=dContactMotion2;
                if (dataInt[2]&128)
                    contactMode|=dContactSlip1;
                if (dataInt[2]&256)
                    contactMode|=dContactSlip2;
                if (dataInt[2]&512)
                    contactMode|=dContactApprox1_1;
                if (dataInt[2]&1024)
                    contactMode|=dContactApprox1_2;
                if (dataInt[2]&2048)
                    contactMode|=dContactApprox1;

                for (int i=0;i<dataInt[1];i++)
                {
                    contact[i].surface.mode=contactMode;//|dContactSlip1|dContactSlip2;//|dContactSoftERP;
                    contact[i].surface.mu=dataFloat[0];//0.25; // use 0.25 as CoppeliaSim default value!
                    contact[i].surface.mu2=dataFloat[1];
                    contact[i].surface.bounce=dataFloat[2];
                    contact[i].surface.bounce_vel=dataFloat[3];
                    contact[i].surface.soft_erp=dataFloat[4];//0.25; // 0.2 appears not bouncy, 0.4 appears medium-bouncy. default is around 0.5
                    contact[i].surface.soft_cfm=dataFloat[5];//0.0;
                    contact[i].surface.motion1=dataFloat[6];
                    contact[i].surface.motion2=dataFloat[7];
                    contact[i].surface.motionN=dataFloat[8];
                    contact[i].surface.slip1=dataFloat[9];
                    contact[i].surface.slip2=dataFloat[10];
                    contact[i].fdir1[0]=dataFloat[11];
                    contact[i].fdir1[1]=dataFloat[12];
                    contact[i].fdir1[2]=dataFloat[13];
                }
                int numc=dCollide(o1,o2,dataInt[1],&contact[0].geom,sizeof(dContact));
                if (numc) 
                {
                    for (int i=0;i<numc;i++) 
                    {
                        dJointID c=dJointCreateContact(_dynamicsWorld,_odeContactGroup,contact+i);
                        dJointAttach(c,b1,b2);

                        dJointFeedback* feedback=new dJointFeedback;
                        dJointSetFeedback(c,feedback);
                        SOdeContactData ctct;
                        ctct.jointID=c;
                        ctct.objectID1=(unsigned long long)dBodyGetData(b1);
                        ctct.objectID2=(unsigned long long)dBodyGetData(b2);
                        ctct.positionScaled=C3Vector(contact[i].geom.pos[0],contact[i].geom.pos[1],contact[i].geom.pos[2]);
                        ctct.normalVector=C3Vector(contact[i].geom.normal[0],contact[i].geom.normal[1],contact[i].geom.normal[2]);
                        _odeContactsRegisteredForFeedback.push_back(ctct);

                        _contactPoints.push_back(contact[i].geom.pos[0]/_positionScalingFactorDyn);
                        _contactPoints.push_back(contact[i].geom.pos[1]/_positionScalingFactorDyn);
                        _contactPoints.push_back(contact[i].geom.pos[2]/_positionScalingFactorDyn);
                    }
                }
            }
        }
    }
}

void CRigidBodyContainerDyn::_applyGravity()
{ // gravity is scaled here!!

    C3Vector gravity;
    _simGetGravity(gravity.data);
    dWorldSetGravity(_dynamicsWorld,gravity(0)*_gravityScalingFactorDyn,gravity(1)*_gravityScalingFactorDyn,gravity(2)*_gravityScalingFactorDyn);
}

dWorldID CRigidBodyContainerDyn::getWorld() const
{
    return(_dynamicsWorld);
}

void CRigidBodyContainerDyn::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
    FILE* f;
    f=fopen(filenameAndPath.c_str(),"wt");
    if (f!=nullptr)
    {
        dWorldExportDIF(_dynamicsWorld,f,"");
        fclose(f);
    }
}

dSpaceID CRigidBodyContainerDyn::getOdeSpace()
{
    return(_odeSpace);
}

void CRigidBodyContainerDyn::_createDependenciesBetweenJoints()
{
}

void CRigidBodyContainerDyn::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
}

void CRigidBodyContainerDyn::_stepDynamics(double dt,int pass)
{
    dSpaceCollide(_odeSpace,0,&_odeCollisionCallbackStatic);
    if (simGetEngineBoolParam(sim_ode_global_quickstep,-1,nullptr,nullptr)!=0)
        dWorldQuickStep(_dynamicsWorld,dt);
    else
        dWorldStep(_dynamicsWorld,dt);

    // We need to destroy the structure for force feedback:
    for (size_t ctfb=0;ctfb<_odeContactsRegisteredForFeedback.size();ctfb++)
    {
        SOdeContactData ctct=_odeContactsRegisteredForFeedback[ctfb];
        SContactInfo ci;
        ci.subPassNumber=pass;
        ci.objectID1=ctct.objectID1;
        ci.objectID2=ctct.objectID2;
        ci.position=ctct.positionScaled/dReal(_positionScalingFactorDyn); // ********** SCALING
        dJointFeedback* fbck=dJointGetFeedback(ctct.jointID);
        C3Vector n(ctct.normalVector);
        n.normalize();
        C3Vector f(fbck->f1[0],fbck->f1[1],fbck->f1[2]);
        if (f*n<0.0)
            n=n*-1.0;
        ci.surfaceNormal=n;
        ci.directionAndAmplitude=f;
        ci.directionAndAmplitude/=dReal(_forceScalingFactorDyn); // ********** SCALING
        delete fbck;
        _contactInfo.push_back(ci);
    }
    _odeContactsRegisteredForFeedback.clear();

    dJointGroupEmpty(_odeContactGroup);

    // Following is very specific to ODE trimeshes:
    std::vector<CRigidBodyDyn*> allBodies;
    _getAllRigidBodies(allBodies);
    for (size_t i=0;i<allBodies.size();i++)
        allBodies[i]->getCollisionShapeDyn()->setOdeMeshLastTransform();
}
