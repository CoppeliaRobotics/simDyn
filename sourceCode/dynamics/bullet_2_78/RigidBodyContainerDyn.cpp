#include "RigidBodyContainerDyn.h"
#include "CollShapeDyn.h"
#include "RigidBodyDyn.h"
#include "ConstraintDyn.h"
#include "simLib.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

// Modifications in Bullet v2.78 source:
// *******************************************************************************
// btHingeConstraint.h:                            BULLET_MOD_2_MODIFIED_BY_MARC
// btTypedConstraint.h:                            BULLET_MOD_4D_MODIFIED_BY_MARC
// btQuaternion.h:                                BULLET_BUG_FIX1_MODIFIED_BY_MARC
// btScalar.h:                                    BULLET_MOD_ATAN2_MODIFIED_BY_MARC
// btSolverConstraint.h:                        BULLET_MOD_4A_MODIFIED_BY_MARC
// btSequentialImpulseConstraintSolver.cpp:        BULLET_MOD_4B_MODIFIED_BY_MARC *
//                                                BULLET_MOD_4C_MODIFIED_BY_MARC *
//                                                BULLET_MOD_4E_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6B_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6C_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6D_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6E_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6F_MODIFIED_BY_MARC *
//                                                BULLET_MOD_6H_MODIFIED_BY_MARC *
// btCollisionShape.h:                            BULLET_MOD_6A_MODIFIED_BY_MARC
// btCollisionObject.h:                            BULLET_MOD_6G_MODIFIED_BY_MARC
// btInternalEdgeUtility.h:                        BULLET_MOD_7A_MODIFIED_BY_MARC
// btInternalEdgeUtility.cpp:                    BULLET_MOD_7B_MODIFIED_BY_MARC

bool CRigidBodyContainerDyn::_bulletContactCallback_useCustom;
double CRigidBodyContainerDyn::_bulletContactCallback_combinedFriction;
double CRigidBodyContainerDyn::_bulletContactCallback_combinedRestitution;

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine=sim_physics_bullet;
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
    delete _dynamicsWorld;
    delete _solver;
    delete _broadphase;
    delete _dispatcher;
    delete _collisionConfiguration;
    delete _filterCallback;

    // Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
    _particleCont->removeAllObjects();
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams,intParams);

    _collisionConfiguration=new btDefaultCollisionConfiguration();
    _dispatcher=new    btCollisionDispatcher(_collisionConfiguration);
    _broadphase=new btDbvtBroadphase();
    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    _solver=new btSequentialImpulseConstraintSolver;
    _dynamicsWorld=new btDiscreteDynamicsWorld(_dispatcher,_broadphase,_solver,_collisionConfiguration);
    _dynamicsWorld->getSolverInfo().m_numIterations=simGetEngineInt32Parameter(sim_bullet_global_constraintsolvingiterations,-1,nullptr,nullptr);
    _dynamicsWorld->getSolverInfo().m_solverMode=SOLVER_SIMD+SOLVER_USE_WARMSTARTING+SOLVER_RANDMIZE_ORDER;//+SOLVER_USE_2_FRICTION_DIRECTIONS; // new since 2010/04/04, to obtain better non-slipping contacts
    //register algorithm
    btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(_dynamicsWorld->getDispatcher());
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
    struct _myCollisionCallback : public btOverlapFilterCallback
    {
        virtual ~_myCollisionCallback()
        {}
        // return true when pairs need collision
        virtual bool needBroadphaseCollision(btBroadphaseProxy* childProxy0,btBroadphaseProxy* childProxy1) const
        {
            btBroadphaseProxy* multiProxy0 =childProxy0;
            btBroadphaseProxy* multiProxy1 =childProxy1;

            bool collides = (multiProxy0->m_collisionFilterGroup & multiProxy1->m_collisionFilterMask) != 0;
            collides = collides && (multiProxy1->m_collisionFilterGroup & multiProxy0->m_collisionFilterMask);

            if (collides)
            {
                int objID1;
                int objID2;
                btRigidBody* a=(btRigidBody*)multiProxy0->m_clientObject;
                btRigidBody* b=(btRigidBody*)multiProxy1->m_clientObject;
                int dataA=(unsigned long long)a->getUserPointer();
                int dataB=(unsigned long long)b->getUserPointer();
                CXShape* shapeA=(CXShape*)_simGetObject(dataA);
                CXShape* shapeB=(CXShape*)_simGetObject(dataB);
                bool canCollide=false;
                if ( (shapeA==nullptr)||(shapeB==nullptr) )
                { // particle-shape or particle-particle case:
                    CParticleObjectContainer_base* particleCont=CRigidBodyContainerDyn::getDynWorld()->getParticleCont();
                    if ( (shapeA==nullptr)&&(shapeB==nullptr) )
                    { // particle-particle case:
                        CParticleObject_base* pa=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                        CParticleObject_base* pb=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                        objID1=dataA;
                        objID2=dataB;
                        if ( (pa!=nullptr)&&(pb!=nullptr) ) // added this condition on 08/02/2011 because of some crashes when scaling some models
                            canCollide=pa->isParticleRespondable()&&pb->isParticleRespondable();
                        else
                            return(false);
                    }
                    else
                    { // particle-shape case:
                        unsigned int collFA=0;
                        if (shapeA!=nullptr)
                        {
                            collFA=_simGetDynamicCollisionMask(shapeA);
                            canCollide=_simIsShapeDynamicallyRespondable(shapeA)!=0;
                            objID1=_simGetObjectID(shapeA);
                        }
                        else
                        {
                            CParticleObject_base* po=particleCont->getObject(dataA-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                            if (po!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                                collFA=po->getShapeRespondableMask();
                            else
                                return(false);
                            objID1=dataA;
                        }
                        unsigned int collFB=0;
                        if (shapeB!=nullptr)
                        {
                            collFB=_simGetDynamicCollisionMask(shapeB);
                            canCollide=_simIsShapeDynamicallyRespondable(shapeB)!=0;
                            objID2=_simGetObjectID(shapeB);
                        }
                        else
                        {
                            CParticleObject_base* po=particleCont->getObject(dataB-CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart(),false);
                            if (po!=nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
                                collFB=po->getShapeRespondableMask();
                            else
                                return(false);
                            objID2=dataB;
                        }
                        canCollide=(canCollide&&(collFA&collFB&0xff00)); // we are global
                    }
                }
                else
                { // regular case (shape-shape)
                    unsigned int collFA=_simGetDynamicCollisionMask(shapeA);
                    unsigned int collFB=_simGetDynamicCollisionMask(shapeB);
                    canCollide=(_simIsShapeDynamicallyRespondable(shapeA)&&_simIsShapeDynamicallyRespondable(shapeB));
                    CXSceneObject* lastPA=(CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeA);
                    CXSceneObject* lastPB=(CXSceneObject*)_simGetLastParentForLocalGlobalCollidable(shapeB);
                    if (lastPA==lastPB)
                        canCollide=(canCollide&&(collFA&collFB&0x00ff)); // we are local
                    else
                        canCollide=(canCollide&&(collFA&collFB&0xff00)); // we are global
                    objID1=_simGetObjectID(shapeA);
                    objID2=_simGetObjectID(shapeB);
                }
                if (canCollide)
                {
                    int dataInt[3]={0,0,0};
                    double dataFloat[14]={1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

                    int customHandleRes=_simHandleCustomContact(objID1,objID2,sim_physics_bullet,dataInt,dataFloat);
                    bool canReallyCollide=(customHandleRes!=0);
                    if (customHandleRes>0)
                    {
                        _bulletContactCallback_useCustom=true;
                        _bulletContactCallback_combinedFriction=dataFloat[0];
                        _bulletContactCallback_combinedRestitution=dataFloat[1];
                    }
                    return(canReallyCollide);
                }
            }
            return(false);
        }
    };
    _filterCallback = new _myCollisionCallback();
    _dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setOverlapFilterCallback(_filterCallback);
    gContactAddedCallback=_bulletContactCallback; // For Bullet's custom contact callback
    return("");
}

bool CRigidBodyContainerDyn::_bulletContactCallback(btManifoldPoint& cp,const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{
    if (_bulletContactCallback_useCustom)
    { // We want a custom handling of this contact!
        cp.m_combinedFriction=_bulletContactCallback_combinedFriction;
        cp.m_combinedRestitution=_bulletContactCallback_combinedRestitution;
        return(true);
    }
    return(false);
}

std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    return("Bullet v2.78");
}

void CRigidBodyContainerDyn::_applyGravity()
{ // gravity is scaled here!!
    C3Vector gravity;
    _simGetGravity(gravity.data);

    if (_dynamicsWorld!=nullptr) // probably not needed anymore!
        _dynamicsWorld->setGravity(btVector3(gravity(0)*_gravityScalingFactorDyn,gravity(1)*_gravityScalingFactorDyn,gravity(2)*_gravityScalingFactorDyn));
}

btDiscreteDynamicsWorld* CRigidBodyContainerDyn::getWorld() const
{
    return(_dynamicsWorld);
}

void CRigidBodyContainerDyn::serializeDynamicContent(const std::string& filenameAndPath,int maxSerializeBufferSize)
{
    if (_dynamicsWorld==nullptr) // probably not needed
        return;
    btDefaultSerializer* serializer=new btDefaultSerializer(maxSerializeBufferSize);
    _dynamicsWorld->serialize(serializer);
    const unsigned char* buff=serializer->getBufferPointer();
    FILE* f;
    f=fopen(filenameAndPath.c_str(),"wt");
    if (f!=nullptr)
    {
        fwrite(buff,1,serializer->getCurrentBufferSize(),f);
        fclose(f);
    }
}


void CRigidBodyContainerDyn::_createDependenciesBetweenJoints()
{
}

void CRigidBodyContainerDyn::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
}

void CRigidBodyContainerDyn::addBulletContactPoints(int dynamicPassNumber)
{
    int numManifolds=_dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold=_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

        btRigidBody* obA=(btRigidBody*)(contactManifold->getBody0());
        btRigidBody* obB=(btRigidBody*)(contactManifold->getBody1());

        int numContacts=contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt=contactManifold->getContactPoint(j);
            btVector3 ptA=pt.getPositionWorldOnA();
            btVector3 ptB=pt.getPositionWorldOnB();

            SContactInfo ci;
            ci.subPassNumber=i;
            ci.objectID1=(unsigned long long)obA->getUserPointer();
            ci.objectID2=(unsigned long long)obB->getUserPointer();
            C3Vector pt1(ptA.x(),ptA.y(),ptA.z());
            C3Vector pt2(ptB.x(),ptB.y(),ptB.z());
            C3Vector avrg((pt1+pt2)*0.5);
            ci.position=avrg/_positionScalingFactorDyn; // ********** SCALING

            double force=pt.m_appliedImpulse/(_forceScalingFactorDyn*_dynamicsInternalStepSize); // ********** SCALING
            C3Vector d(pt.m_normalWorldOnB.x(),pt.m_normalWorldOnB.y(),pt.m_normalWorldOnB.z());
            ci.directionAndAmplitude=d*force;
            if (pt.m_lateralFrictionInitialized)
            {
                double ff1=pt.m_appliedImpulseLateral1/(_forceScalingFactorDyn*_dynamicsInternalStepSize); // ********** SCALING
                double ff2=pt.m_appliedImpulseLateral2/(_forceScalingFactorDyn*_dynamicsInternalStepSize); // ********** SCALING
                C3Vector fd1(pt.m_lateralFrictionDir1.x(),pt.m_lateralFrictionDir1.y(),pt.m_lateralFrictionDir1.z());
                C3Vector fd2(pt.m_lateralFrictionDir2.x(),pt.m_lateralFrictionDir2.y(),pt.m_lateralFrictionDir2.z());
                ci.directionAndAmplitude+=fd1*ff1+fd2*ff2;
                // pt.m_appliedImpulseLateral1, pt.m_appliedImpulseLateral2, pt.m_lateralFrictionDir1 and pt.m_lateralFrictionDir2
                // appear to be zero, as if there was no friction force... ?
            }
            ci.surfaceNormal=d;
            ci.subPassNumber=dynamicPassNumber;
            _contactInfo.push_back(ci);

            _contactPoints.push_back(avrg(0)/_positionScalingFactorDyn);
            _contactPoints.push_back(avrg(1)/_positionScalingFactorDyn);
            _contactPoints.push_back(avrg(2)/_positionScalingFactorDyn);
        }
    }
}

void CRigidBodyContainerDyn::_stepDynamics(double dt,int pass)
{
    _dynamicsWorld->stepSimulation(dt,10000,dt);
    addBulletContactPoints(pass);
}
