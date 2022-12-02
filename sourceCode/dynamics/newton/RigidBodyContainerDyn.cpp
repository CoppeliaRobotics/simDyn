#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "ConstraintDyn.h"
#include "simLib.h"

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine=sim_physics_newton;
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

    NewtonWaitForUpdateToFinish (_dynamicsWorld);
    NewtonDestroyAllBodies (_dynamicsWorld);
    NewtonDestroy(_dynamicsWorld);

    // Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
    _particleCont->removeAllObjects();
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams,intParams);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double fParams[2];
    int iParams[2];
    int ver=0;
    _simGetNewtonParameters(nullptr,&ver,fParams,iParams);

    const double stepSize=fParams[0];
    const double contactMergeTolerance=fParams[1];

    const int newtonIterationsCount=iParams[0];
    const bool multithreaded=(iParams[1]&1)!=false;
    const bool exactSolver=(iParams[1]&2)!=false;
    const bool highJointAccuracy=(iParams[1]&4)!=false;

    // TODO_NEWTON_X2
    // Above settings from CoppeliaSim are not used to configure the engine:
    // contactMergeTolerance, exactSolver, highJointAccuracy

    // install the memory handle
    NewtonSetMemorySystem (NewtonAllocMemory, NewtonFreeMemory);

    // Create the Newton world
    _dynamicsWorld = NewtonCreate ();

    // plus the user pointer call back
    NewtonWorldSetUserData (_dynamicsWorld, this);

    // set the solver accuracy mode
    NewtonSetSolverModel (_dynamicsWorld,newtonIterationsCount);


    if (multithreaded)
        NewtonSetThreadsCount(_dynamicsWorld,4);
    else
        NewtonSetThreadsCount(_dynamicsWorld,1);

    // set the solver to a high converge rate quality
    NewtonSetSolverConvergenceQuality (_dynamicsWorld, 1);

    // set the Material call back for pair default-default of the material graph
    int defaultMaterialID = NewtonMaterialGetDefaultGroupID(_dynamicsWorld);
    NewtonMaterialSetCollisionCallback (_dynamicsWorld, defaultMaterialID, defaultMaterialID, nullptr, NewtonOnAABBOverlap, NewtonOnUserContacts);

    // set joint serialization call back
    CustomJoint::Initalize(_dynamicsWorld);

    _rebuildSkeletons = true;
    return("");
}

std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    return("Newton v3.14");
}

void CRigidBodyContainerDyn::_addNewtonContactPoints(int dynamicPassNumber)
{
    dTree <NewtonJoint*, NewtonJoint*> filter;
    for (NewtonBody* body = NewtonWorldGetFirstBody(_dynamicsWorld); body; body = NewtonWorldGetNextBody(_dynamicsWorld, body))
    {
        for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint))
        {
            if (!filter.Find(joint))
            {
                filter.Insert (joint, joint);
                if (NewtonJointIsActive(joint))
                {
                    NewtonBody* const newtonBodyA = NewtonJointGetBody0(joint);
                    NewtonBody* const newtonBodyB = NewtonJointGetBody1(joint);
                    _ASSERTE ((body == newtonBodyA) || (body == newtonBodyB));
                    void** userDataA=(void**)NewtonBodyGetUserData(newtonBodyA);
                    void** userDataB=(void**)NewtonBodyGetUserData(newtonBodyB);
                    CRigidBodyDyn* const bodyA = (CRigidBodyDyn*)userDataA[1];
                    CRigidBodyDyn* const bodyB = (CRigidBodyDyn*)userDataB[1];
                    for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact))
                    {
                        dVector point;
                        dVector normal;
                        dVector dir0;
                        dVector dir1;
                        dVector force;
                        NewtonMaterial* const material = NewtonContactGetMaterial(contact);
                        NewtonMaterialGetContactPositionAndNormal(material, newtonBodyA, &point.m_x, &normal.m_x);
                        NewtonMaterialGetContactForce (material, newtonBodyA, &force.m_x);
                        NewtonMaterialGetContactTangentDirections(material, body, &dir0.m_x, &dir1.m_x);

                        // We want the full contact force (including the friction-related force).
                        // But keep following commented, in future we will also offer the normal force:
                        // force = normal.Scale (normal % force);

                        _contactPoints.push_back(point.m_x);
                        _contactPoints.push_back(point.m_y);
                        _contactPoints.push_back(point.m_z);

                        SContactInfo ci;
                        ci.subPassNumber = dynamicPassNumber;
                        ci.objectID1 = ((int*)userDataA[0])[0];
                        ci.objectID2 = ((int*)userDataB[0])[0];
                        ci.position = C3Vector (point.m_x, point.m_y, point.m_z);
                        C3Vector n(normal.m_x, normal.m_y, normal.m_z);
                        n.normalize();
                        C3Vector f(force.m_x, force.m_y, force.m_z);
                        if (n*f<0.0)
                            n=n*-1.0;
                        ci.surfaceNormal =n;
                        ci.directionAndAmplitude = f;
                        _contactInfo.push_back(ci);
                    }
                }
            }
        }
    }
}

int CRigidBodyContainerDyn::NewtonOnAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{ // As far as I understood this, this is the broad-phase collision callback.
    // If it returns 0, no further collision is checked between those 2 bodies,
    // and they will not collide with each other.
    // So here, we check if those two bodies can collide

    // If we have a contact callback registered, we always return 1:
    if (_simGetContactCallbackCount()>0)
        return(1); // the narrow-phase routine will handle the callback!

    void** userDataA=(void**)NewtonBodyGetUserData(body0);
    void** userDataB=(void**)NewtonBodyGetUserData(body1);
    CXShape* shapeA=(CXShape*)_simGetObject(((int*)userDataA[0])[0]);
    CXShape* shapeB=(CXShape*)_simGetObject(((int*)userDataB[0])[0]);
    bool canCollide=false;
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
    }
    else
    { // particle-shape or particle-particle case:
        CParticleObjectContainer_base* particleCont=CRigidBodyContainerDyn::getDynWorld()->getParticleCont();
        int dataA=((int*)userDataA[0])[0];
        int dataB=((int*)userDataB[0])[0];
        if ( (shapeA==nullptr)&&(shapeB==nullptr) )
        { // particle-particle case:
            CParticleObject_base* pa=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
            CParticleObject_base* pb=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);

            if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
                canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
        }
        else
        { // particle-shape case:
            CXShape* shape=nullptr;
            CParticleObject_base* particle=nullptr;
            if (shapeA!=nullptr)
                shape=shapeA;
            else
                particle=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
            if (shapeB!=nullptr)
                shape=shapeB;
            else
                particle=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);

            if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                canCollide=_simIsShapeDynamicallyRespondable(shape)&&(_simGetDynamicCollisionMask(shape)&particle->getShapeRespondableMask()&0xff00)&&((_simGetTreeDynamicProperty(shape)&sim_objdynprop_respondable)!=0); // we are global
        }
    }

    if (canCollide)
        return 1;
    return 0;
}

void CRigidBodyContainerDyn::_rebuildSkeletonList()
{
    if (_rebuildSkeletons) {
        for (NewtonBody* body = NewtonWorldGetFirstBody(_dynamicsWorld); body; body = NewtonWorldGetNextBody (_dynamicsWorld, body)) {
            NewtonSkeletonContainer* const skeleton = NewtonBodyGetSkeleton(body);
            if (skeleton) {
                NewtonSkeletonContainerDelete (skeleton);
            }
        }

        dTree<CRigidBodyDyn*, CRigidBodyDyn*> filter;
        std::vector<CRigidBodyDyn*> allBodies;
        _getAllRigidBodies(allBodies);
        for (size_t i=0;i<allBodies.size();i++) {

            CRigidBodyDyn* root = allBodies[i];
            if (!filter.Find(root)) {
                bool hasAcyclicJoints = false;
                for (bool foundParent = true; foundParent; ) {
                    foundParent = false;
                    NewtonBody* const body = ((CRigidBodyDyn*)root)->getNewtonRigidBody();
                    dAssert(body);
                    for (NewtonJoint* jointptr = NewtonBodyGetFirstJoint(body); jointptr; jointptr = NewtonBodyGetNextJoint(body, jointptr)) {
                        CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (jointptr);
                        CConstraintDyn* const constraint = (CConstraintDyn*)joint->GetUserData();
                        if (((CConstraintDyn*)constraint)->_isAcyclicJoint()) {
                            hasAcyclicJoints = true;
                            if (((CConstraintDyn*)constraint)->_getChild() == root) {
                                root = ((CConstraintDyn*)constraint)->_getParent();
                                foundParent = true;
                                break;
                            }
                        }
                    }
                }

                if (hasAcyclicJoints) {
                    int stack = 1;
                    CRigidBodyDyn* pool[64];
                    pool[0] = root;

                    NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate (_dynamicsWorld, ((CRigidBodyDyn*)root)->getNewtonRigidBody(), nullptr);
                    NewtonSkeletonSetSolverMode (skeleton, 1);
                    while (stack) {
                        stack --;
                        CRigidBodyDyn* const root = pool[stack];
                        filter.Insert(root);
                        NewtonBody* const parentBone = ((CRigidBodyDyn*)root)->getNewtonRigidBody();
                        for (NewtonJoint* jointptr = NewtonBodyGetFirstJoint(parentBone); jointptr; jointptr = NewtonBodyGetNextJoint(parentBone, jointptr)) {
                            CustomJoint* const joint = (CustomJoint*) NewtonJointGetUserData (jointptr);
                            CConstraintDyn* const constraint = (CConstraintDyn*)joint->GetUserData();
                            if (((CConstraintDyn*)constraint)->_isAcyclicJoint()) {
                                if (((CConstraintDyn*)constraint)->_getParent() == root) {
                                    CRigidBodyDyn* const child = ((CConstraintDyn*)constraint)->_getChild();
                                    pool[stack] = child;
                                    stack ++;
                                    NewtonSkeletonContainerAttachBone (skeleton,((CRigidBodyDyn*)child)->getNewtonRigidBody(), parentBone);
                                }
                            }
                        }
                    }
                    NewtonSkeletonContainerFinalize (skeleton);
                }
            }
        }
    }
    _rebuildSkeletons = false;
}

void CRigidBodyContainerDyn::_notifySekeletonRebuild ()
{
    _rebuildSkeletons = true;
}
/*
struct MaterialsPareamaters
{
    dFloat m_restitution;
    dFloat m_staticFriction;
    dFloat m_kineticFriction;
};
*/
/*
static MaterialsPareamaters materialGraph [4][4]=
{
                   // steel                 concrete                wood                asphalt
    // steel      { {0.5, 0.7, 0.6},  {0.5, 0.7, 0.6},    {0.5, 0.7, 0.6}, {0.5, 0.7, 0.6} },
    // concrete  { {0.5, 0.7, 0.6},  {0.5, 0.7, 0.6},    {0.5, 0.7, 0.6}, {0.5, 0.7, 0.6} },
    // wood      { {0.5, 0.7, 0.6},  {0.5, 0.7, 0.6},    {0.5, 0.7, 0.6}, {0.5, 0.7, 0.6} },
    // asphalt   { {0.5, 0.7, 0.6},  {0.5, 0.7, 0.6},    {0.5, 0.7, 0.6}, {0.5, 0.7, 0.6} },
};
*/
void CRigidBodyContainerDyn::NewtonOnUserContacts(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{ // This is the narrow-phase collision checking and material setting if I understood it correctly
    // Here we actually (normally) want our objects to collide, unless a callback tells us otherwise:

    NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
    NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

    void** userDataA=(void**)NewtonBodyGetUserData(body0);
    void** userDataB=(void**)NewtonBodyGetUserData(body1);

    CXShape* shapeA=(CXShape*)_simGetObject(((int*)userDataA[0])[0]);
    CXShape* shapeB=(CXShape*)_simGetObject(((int*)userDataB[0])[0]);
    bool collides=true; // was already checked previously, unless we have a user callback
    int id_A=((int*)userDataA[0])[0];
    int id_B=((int*)userDataB[0])[0];
    double statFriction_A=0.0;
    double statFriction_B=0.0;
    double kinFriction_A=0.0;
    double kinFriction_B=0.0;
    double restit_A=0.0;
    double restit_B=0.0;
    if ( (shapeA!=nullptr)&&(shapeB!=nullptr) )
    { // regular case (shape-shape)
        statFriction_A=(double)((float*)userDataA[2])[0];
        statFriction_B=(double)((float*)userDataB[2])[0];
        kinFriction_A=(double)((float*)userDataA[3])[0];
        kinFriction_B=(double)((float*)userDataB[3])[0];
        restit_A=(double)((float*)userDataA[4])[0];
        restit_B=(double)((float*)userDataB[4])[0];
    }
    else
    { // particle-shape or particle-particle case:
        CParticleObjectContainer_base* particleCont=CRigidBodyContainerDyn::getDynWorld()->getParticleCont();
        if ( (shapeA==nullptr)&&(shapeB==nullptr) )
        { // particle-particle case:
            CParticleObject_base* pa=particleCont->getObject(id_A-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
            CParticleObject_base* pb=particleCont->getObject(id_B-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);

            if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
            {
                // Get the particle's user data:
                statFriction_A=(double)((float*)userDataA[2])[0];
                statFriction_B=(double)((float*)userDataB[2])[0];
                kinFriction_A=(double)((float*)userDataA[3])[0];
                kinFriction_B=(double)((float*)userDataB[3])[0];
                restit_A=(double)((float*)userDataA[4])[0];
                restit_B=(double)((float*)userDataB[4])[0];
            }
            else
                collides=false; // not normal
        }
        else
        { // particle-shape case:
            if (shapeA!=nullptr)
            {
                statFriction_A=(double)((float*)userDataA[2])[0];
                kinFriction_A=(double)((float*)userDataA[3])[0];
                restit_A=(double)((float*)userDataA[4])[0];
            }
            else
            {
                CParticleObject_base* particle=particleCont->getObject(id_A-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                {
                    // Get the particle's user data:
                    statFriction_B=(double)((float*)userDataB[2])[0];
                    kinFriction_B=(double)((float*)userDataB[3])[0];
                    restit_B=(double)((float*)userDataB[4])[0];
                }
                else
                    collides=false; // not normal
            }
            if (shapeB!=nullptr)
            {
                statFriction_B=(double)((float*)userDataB[2])[0];
                kinFriction_B=(double)((float*)userDataB[3])[0];
                restit_B=(double)((float*)userDataB[4])[0];
            }
            else
            {
                CParticleObject_base* particle=particleCont->getObject(id_B-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                if (particle!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                {
                    // Get the particle's user data:
                    statFriction_A=(double)((float*)userDataA[2])[0];
                    kinFriction_A=(double)((float*)userDataA[3])[0];
                    restit_A=(double)((float*)userDataA[4])[0];
                }
                else
                    collides=false; // not normal
            }
        }
    }

    double statFriction=statFriction_A*statFriction_B;
    double kinFriction=kinFriction_A*kinFriction_B;
    double restit=(restit_A+restit_B)/2.0;

    int dataInt[3]={0,0,0};
    double dataFloat[14]={statFriction,kinFriction,restit,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    if (collides)
    {
        int flag=0;
#ifndef DG_USE_THREAD_EMULATION
        flag|=1024; // means: the callback scripts won't be called (e.g. when this thread is not the simulation thread)
#endif
        int customHandleRes=_simHandleCustomContact(id_A,id_B,sim_physics_newton+flag,dataInt,dataFloat);
        collides=(customHandleRes!=0);
        if (customHandleRes>0)
        {
            statFriction=dataFloat[0];
            kinFriction=dataFloat[1];
            restit=dataFloat[2];
        }
    }

    // TODO_NEWTON:
    // if collides is false, then we want to ignore this collision

    for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact))
    {
        NewtonMaterial* const material = NewtonContactGetMaterial(contact);
        NewtonMaterialSetContactElasticity(material, restit);
        NewtonMaterialSetContactFrictionCoef(material, statFriction, kinFriction, 0);
        NewtonMaterialSetContactFrictionCoef(material, statFriction, kinFriction, 1);
    }
}

void CRigidBodyContainerDyn::_applyGravity()
{ // gravity is scaled here!!

    C3Vector gravity;
    _simGetGravity(gravity.data);
    // in newton the gravity and all external forces and torque are applied in the force and torque callback 
}

NewtonWorld* CRigidBodyContainerDyn::getWorld() const
{
    return(_dynamicsWorld);
}

void CRigidBodyContainerDyn::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
    // TODO_NEWTON
}

void CRigidBodyContainerDyn::_createDependenciesBetweenJoints()
{
    // TODO_NEWTON
}

void CRigidBodyContainerDyn::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
    // TODO_NEWTON
}

bool CRigidBodyContainerDyn::_updateWorldFromCoppeliaSim()
{
    bool retVal=CRigidBodyContainerDyn_base::_updateWorldFromCoppeliaSim();
    if (((CRigidBodyContainerDyn*)this)->_rebuildSkeletons)
        ((CRigidBodyContainerDyn*)this)->_rebuildSkeletonList();
    return(retVal);
}

void* CRigidBodyContainerDyn::NewtonAllocMemory(int sizeInBytes)
{
    //return malloc (sizeInBytes);
    return simCreateBuffer (sizeInBytes);
}

void CRigidBodyContainerDyn::NewtonFreeMemory(void* const ptr, int sizeInBytes)
{
    //free (ptr);
    ptrSimReleaseBuffer((char*) ptr);
}

void CRigidBodyContainerDyn::_stepDynamics(double dt,int pass)
{
    NewtonUpdate(_dynamicsWorld,dt);
    _addNewtonContactPoints(pass);
}
