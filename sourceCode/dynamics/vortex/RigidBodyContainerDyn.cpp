#include <RigidBodyContainerDyn.h>
#include <CollShapeDyn.h>
#include <ConstraintDyn.h>
#include <simLib/simLib.h>
#include <iostream>
#include <Vx/VxFrame.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxPart.h>
#include <Vx/VxIntersectResult.h>
#include <Vx/VxDynamicsContactInput.h>
#include <Vx/VxDynamicsContact.h>
#include <Vx/VxContactMaterial.h>
#include <VortexConvertUtil.h>
#include <Vx/VxTriangleMeshBVTree.h>
#include <Vx/VxTriangleMeshUVGrid.h>
#include <Vx/VxSolverParameters.h>
#include <Vx/VxConstraint.h>
#include <Vx/VxRequest.h>
#include <Vx/VxIntersectFilter.h>
#include <Vx/VxMessage.h>
#include <Vx/VxPrismatic.h>
#include <Vx/VxVersion.h>
#include <Vx/VxBox.h>
#include <Vx/VxRigidBodyResponseModel.h>
#include <Vx/VxMaterialTable.h>
#ifdef LIN_SIM
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#endif

bool VxFrameReleased = true; // debug to use universe print content
bool gPrintContact = false;
bool gPrintConstraintDebug = false;
unsigned int vortexStartTimeTag;
bool CRigidBodyContainerDyn::_checkingLicense = false;
double CRigidBodyContainerDyn::gravityVectorLength = 0.0;

/* Those globals values are used to self parameterize Vortex so that it is stable without having to
 * tweak values.
 */
//const bool sAutoAngularDamping = true;
//const double sSkinThickness = 0.002;
//const bool sAutoSlip = true;
const Vx::VxReal spScaleL = 1; // scales default linear solver parameters
const Vx::VxReal spScaleA = 1; // scales default angular solver parameters

class VortexIntersectFilter : public Vx::VxIntersectFilter
{
  public:
    bool isPairEnabled(Vx::VxCollisionGeometry* cg0, Vx::VxCollisionGeometry* cg1)
    {
        bool canCollide = true;
        CRigidBodyContainerDyn::_vortexCollisionCallbackStatic((void*)&canCollide, cg0, cg1);
        return canCollide;
    }
};
static int VortexResponsePart = 0; // Vx::VxUniverse::kResponsePart; TODO: why the compiler doesn't find this

class VortexIntersectSubscriber : public Vx::VxUniverse::IntersectSubscriber
{
  public:
    virtual void notifyDisjoint(Vx::VxUniverse::eIntersectEventType, Vx::VxIntersectResult*)
    {
        // don't need
    }
    virtual void notifyIntersect(Vx::VxUniverse::eIntersectEventType type, Vx::VxIntersectResult* ires, Vx::VxDynamicsResponseInput* dres)
    {
        if (CRigidBodyContainerDyn::_checkingLicense)
            return;
        Vx::VxCollisionGeometry* cg[2];
        ires->getCollisionGeometryPair(cg, cg + 1);
        Vx::VxPart* part[2];
        ires->getPartPair(part, part + 1);

        CRigidBodyDyn* body1 = (CRigidBodyDyn*)part[0]->userData().getData("csim1").getPointerVoid();
        CRigidBodyDyn* body2 = (CRigidBodyDyn*)part[1]->userData().getData("csim1").getPointerVoid();
        double skin1 = 0.0;
        if (body1 != nullptr)
            skin1 = ((CRigidBodyDyn*)body1)->vortex_skinThickness;
        double skin2 = 0.0;
        if (body2 != nullptr)
            skin2 = ((CRigidBodyDyn*)body2)->vortex_skinThickness;
        bool autoSlip = (((body1 != nullptr) && ((CRigidBodyDyn*)body1)->vortex_autoSlip) || ((body2 != nullptr) && ((CRigidBodyDyn*)body2)->vortex_autoSlip));
        double skinThickness = skin1;
        if (skin2 > skinThickness)
            skinThickness = skin2;
        /*  If penetration < sSkinThickness, the contact stiffness will be proportional to the penetration.
         *  The goal is to help the solver to find the appropriate point of equilibrium during grasping.
         */
        if (skinThickness > 0.0)
        {
            Vx::VxDynamicsContactInput* vi = (Vx::VxDynamicsContactInput*)dres;
            Vx::VxDynamicsContactInput::DynamicsContactIterator it = vi->dynamicsContactBegin();
            Vx::VxDynamicsContactInput::DynamicsContactIterator ie = vi->dynamicsContactEnd();

            while (it != ie)
            {
                const Vx::VxReal p = (*it)->getPenetration();
                if (p <= skinThickness && p > 0)
                {
                    Vx::VxContactMaterial* m = (*it)->getContactMaterial();
                    const Vx::VxReal s = 1.0 / m->getCompliance() * p;
                    m->setCompliance(1.0 / s);
                    m->setDamping(s / 10.0);
                }
                ++it;
            }
        }
        const double sScale = 0.01;
        const double sGravity = CRigidBodyContainerDyn::gravityVectorLength;
        const double sMaxMassScale = 1000;
        if (autoSlip)
        {
            /*  The autoslip adapts the contact slip vs the pressure on the contacts.
             *  This contact slip may be problematic when a ligth object is under large pressure
             *  The slip is computed from the contact normal forces.
             */
            Vx::VxDynamicsContactInput* vi = (Vx::VxDynamicsContactInput*)dres;
            Vx::VxDynamicsContactInput::DynamicsContactIterator it = vi->dynamicsContactBegin();
            Vx::VxDynamicsContactInput::DynamicsContactIterator ie = vi->dynamicsContactEnd();
            int index = 0;
            int scale = 1;
            Vx::VxReal mass;
            if (part[0]->getControl() != Vx::VxPart::kControlDynamic)
            {
                index = 1;
                scale = -1;
                mass = part[1]->getMass();
            }
            else if (part[1]->getControl() != Vx::VxPart::kControlDynamic)
            {
                mass = part[0]->getMass();
            }
            else
            {
                mass = std::min<double>(part[0]->getMass(), part[1]->getMass());
            }
            Vx::VxReal dt = Vx::VxFrame::currentInstance()->getTimeStep();

            while (it != ie)
            {
                bool matched = (*it)->getMatched();
                if (matched)
                {
                    /* What we want to do here: the heavier the objects the smaller the slip we need.
                     * In the case of a car for example, the slip should not be based on the wheel but
                     * on the pressure the chassis exert on the wheel. This pressure would be read in the
                     * suspension constraint. The larger the pressure the smaller the slip. Now if the wheel
                     * is under a chassis with chassisMass > sMaxMassScale*wheelMass, we liimt the slip to avoid
                     * instability due to very large mass ratio.
                     *
                     * Similarly, in the case of the gripper, the slip decrease with the pressure.
                    */
                    Vx::VxVector3 n, f;
                    (*it)->getNormal(n);
                    (*it)->getForce(index, f);
                    const Vx::VxReal lambda = f.dot(n) * scale;
                    // compute apparent mass using lambda / gravity
                    Vx::VxReal apparentMass = lambda / sGravity;

                    {
                        // limit apparentMass vs the smallest mass in the pair of colliding parts.
                        apparentMass = std::max<double>(mass, apparentMass);
                        apparentMass = std::min<double>(sMaxMassScale * mass, apparentMass);
                        if (lambda > Vx::VX_MEDIUM_EPSILON)
                        {
                            const Vx::VxReal loss = sScale * dt / apparentMass;
                            //Vx::VxInfo(0, " computed loss = %g, original was %g\n", loss, (*it)->getContactMaterial()->getSlip(Vx::VxMaterial::kFrictionAxisLinearPrimary));
                            (*it)->getContactMaterial()->setSlip(Vx::VxMaterial::kFrictionAxisLinear, loss);
                        }
                    }
                }
                else
                {
                    const Vx::VxReal loss = sScale * dt / mass;
                    //Vx::VxInfo(0, " computed loss = %g, original was %g\n", loss, (*it)->getContactMaterial()->getSlip(Vx::VxMaterial::kFrictionAxisLinearPrimary));
                    (*it)->getContactMaterial()->setSlip(Vx::VxMaterial::kFrictionAxisLinear, loss);
                }
                ++it;
            }
        }
    }
};

void vortexInfoHandler(const int level, const char* const format, va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp += format;
    vsnprintf(buff, sizeof(buff), tmp.c_str(), ap);
    simAddLog(LIBRARY_NAME, sim_verbosity_infos, buff);
}

void vortexWarningHandler(const int level, const char* const format, va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp += format;
    vsnprintf(buff, sizeof(buff), tmp.c_str(), ap);
    simAddLog(LIBRARY_NAME, sim_verbosity_warnings, buff);
}

void vortexErrorHandler(const int level, const char* const format, va_list ap)
{
    char buff[2000];
    std::string tmp("msg from Vortex: ");
    tmp += format;
    vsnprintf(buff, sizeof(buff), tmp.c_str(), ap);
    simAddLog(LIBRARY_NAME, sim_verbosity_errors, buff);
}

Vx::VxPart* getCollisionGeometryPart(Vx::VxCollisionGeometry* cg)
{
    while (cg->getPart() == nullptr && cg->getParent() != nullptr)
    {
        cg = (Vx::VxCollisionGeometry*)cg->getParent();
    }
    return cg->getPart();
}

std::string CRigidBodyContainerDyn::getEngineInfo() const
{
    std::string v("Vortex v");
    v += Vx::VxGetVersion();
    return (v);
}

CRigidBodyContainerDyn::CRigidBodyContainerDyn()
{
    _engine = sim_physics_vortex;
    _engineVersion = 0;
}

CRigidBodyContainerDyn::~CRigidBodyContainerDyn()
{
    std::vector<CRigidBodyDyn*> allBodies;
    _getAllRigidBodies(allBodies);
    for (size_t i = 0; i < allBodies.size(); i++)
        _removeRigidBody(allBodies[i]->getShapeHandle());

    for (int i = 0; i < _simGetObjectListSize(sim_handle_all); i++)
    {
        CXSceneObject* it = (CXSceneObject*)_simGetObjectFromIndex(sim_handle_all, i);
        _simSetDynamicSimulationIconCode(it, sim_dynamicsimicon_none);
    }

    _particleCont->removeAllParticles();

    _dynamicsWorld->removeIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventFirst, vortexIntersectSubscriber);
    _dynamicsWorld->removeIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventActive, vortexIntersectSubscriber);
    delete vortexIntersectSubscriber;
    delete vortexIntersectFilter;
    CRigidBodyDyn::setVortexFilter(nullptr);
    // this will delete everything, frame, universe and all objects not removed from it
    Vx::VxFrame::currentInstance()->release();
    VxFrameReleased = true;
    _dynamicsWorld = nullptr;

    // Important to destroy it at the very end, otherwise we have memory leaks with bullet (b/c we first need to remove particles from the Bullet world!)
    _particleCont->removeAllObjects();
}

std::string CRigidBodyContainerDyn::init(const double floatParams[20], const int intParams[20])
{
    CRigidBodyContainerDyn_base::init(floatParams, intParams);

    int plugin_verbosity = sim_verbosity_default;
    simGetModuleInfo(LIBRARY_NAME, sim_moduleinfo_verbosity, nullptr, &plugin_verbosity);
    if (plugin_verbosity == sim_verbosity_none)
        Vx::LogSetLevel(Vx::kOff);
    if (plugin_verbosity >= sim_verbosity_errors)
        Vx::LogSetLevel(Vx::kError);
    if (plugin_verbosity >= sim_verbosity_warnings)
        Vx::LogSetLevel(Vx::kWarn);
    if (plugin_verbosity >= sim_verbosity_infos)
        Vx::LogSetLevel(Vx::kInfo);
    if (plugin_verbosity >= sim_verbosity_debug)
        Vx::LogSetLevel(Vx::kAll);

    vortexStartTimeTag = int(simGetSystemTime() * 1000.0);
    Vx::VxFrame* frame = Vx::VxFrame::instance();
    _dynamicsWorld = new Vx::VxUniverse();
    frame->addUniverse(_dynamicsWorld);
    // adjust contact matching epsilon
    for (int i = 0; i < Vx::VxCollisionPairRequest::kDefaultRequestCount; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            Vx::VxCollisionPairRequest* req = _dynamicsWorld->getPartPartCollisionPairRequest(i, j);
            req->setContactMatchingEpsilon(0.01);
        }
    }
    // ?? Vx::VxDynamicsContact::setForceContactMatching(true);
    static const Vx::VxReal initialVelocityThresholdScale = Vx::VxDynamicsContact::getSlidingVelocityThresholdScale();
    Vx::VxDynamicsContact::setSlidingVelocityThresholdScale(initialVelocityThresholdScale);
    _dynamicsWorld->getSolverParameters(0)->setScaleBoxMaxIteration(2);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    double fParams[10];
    int iParams[1];
    _simGetVortexParameters(nullptr, 3, fParams, iParams);

    //        const double stepSize=getVortexUnsignedDouble(fParams[0]);
    //        const double internalScalingFactor=getVortexUnsignedDouble(fParams[1]);
    const double contactTolerance = getVortexUnsignedDouble(fParams[2]);
    const double constraint_linear_compliance = getVortexUnsignedDouble(fParams[3]);
    const double constraint_linear_damping = getVortexUnsignedDouble(fParams[4]);
    const double constraint_linear_kineticLoss = getVortexUnsignedDouble(fParams[5]);
    const double constraint_angular_compliance = getVortexUnsignedDouble(fParams[6]);
    const double constraint_angular_damping = getVortexUnsignedDouble(fParams[7]);
    const double constraint_angular_kineticLoss = getVortexUnsignedDouble(fParams[8]);
    // fParams[9] is RESERVED!! (used to be the auto angular damping tension ratio)

    const bool autoSleep = ((iParams[0] & 1) != 0);
    const bool multiThreading = ((iParams[0] & 2) != 0);
    // (iParams[0]&4) is for full internal scaling
    // iParams[0]&8 is RESERVED!! (used to be the auto angular damping)

    // In ODE and Bullet we disable the auto-sleep functionality, because with those engines,
    // if you remove the floor under disabled boxes, they will not automatically wake.
    // This is different with Vortex (correctly handled), thus, in Vortex, we
    // can keep the auto-sleep functionality!
    _dynamicsWorld->setAutoSleep(autoSleep);

    // Constraint solver parameters:
    Vx::VxSolverParameters* sp = _dynamicsWorld->getSolverParameters();
    sp->setConstraintLinearCompliance(constraint_linear_compliance);     // Vortex default: 1.0e-10
    sp->setConstraintLinearDamping(constraint_linear_damping);           // Vortex default: 8.33e+8
    sp->setConstraintLinearKineticLoss(constraint_linear_kineticLoss);   // Vortex default: 6.0e-9
    sp->setConstraintAngularCompliance(constraint_angular_compliance);   // Vortex default: 1.0e-10
    sp->setConstraintAngularDamping(constraint_angular_damping);         // Vortex default: 8.33e+8
    sp->setConstraintAngularKineticLoss(constraint_angular_kineticLoss); // Vortex default: 6.0e-9

    /*
    sp->setConstraintLinearCompliance(1.0e-10*spScaleL); // Vortex default: 1.0e-10
    sp->setConstraintLinearDamping(8.33e+8/spScaleL); // Vortex default: 8.33e+8
    sp->setConstraintLinearKineticLoss(6.0e-9*spScaleL); // Vortex default: 6.0e-9
    sp->setConstraintAngularCompliance(1.0e-10*spScaleA); // Vortex default: 1.0e-10
    sp->setConstraintAngularDamping(8.33e+8/spScaleA); // Vortex default: 8.33e+8
    sp->setConstraintAngularKineticLoss(6.0e-9*spScaleA); // Vortex default: 6.0e-9
*/
    /*
    just before IROS/IREX. TODO for after:
    - Scale all vortex params (forgotten)
    - Add a box max force parameter (see email from Martin on 15/10/2013)
*/
    // Contact tolerance. Vortex default: 0.001
    //    frame->setContactTolerance(contactTolerance*linScaling);
    _dynamicsWorld->setDefaultContactTolerance(contactTolerance);

    frame->setAutomaticTimeStep(false);
    _dynamicsWorld->setDynamicsMultithreaded(multiThreading);
    _dynamicsWorld->setCollisionMultithreaded(multiThreading);

    vortexIntersectSubscriber = new VortexIntersectSubscriber;
    _dynamicsWorld->addIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventFirst, vortexIntersectSubscriber, 0);
    _dynamicsWorld->addIntersectSubscriber(VortexResponsePart, VortexResponsePart, Vx::VxUniverse::kEventActive, vortexIntersectSubscriber, 0);
    vortexIntersectFilter = new VortexIntersectFilter;
    CRigidBodyDyn::setVortexFilter(vortexIntersectFilter);
    _licenseCheck();
    return ("");
}

void CRigidBodyContainerDyn::_licenseCheck()
{
    static bool done = false;
    static bool licensePresent = false;

    if (!done)
    {
        simAddLog(LIBRARY_NAME, sim_verbosity_loadinfos, Vx::VxGetVersion());
        std::string year(Vx::VxGetVersion(), Vx::VxGetVersion() + 4);

        _checkingLicense = true;
        if (std::stoi(year) <= 2019)
        {
            Vx::VxPart* partDynamic = new Vx::VxPart(1.0);
            _dynamicsWorld->addPart(partDynamic);
            Vx::VxPart* partStatic = new Vx::VxPart(1.0);
            _dynamicsWorld->addPart(partStatic);
            partStatic->setControl(Vx::VxPart::kControlStatic);
            Vx::VxCollisionGeometry* cgDynamic = new Vx::VxCollisionGeometry(new Vx::VxBox(C3Vector2VxVector3(C3Vector(0.1, 0.1, 0.1))), nullptr);
            partDynamic->addCollisionGeometry(cgDynamic);
            Vx::VxCollisionGeometry* cgStatic = new Vx::VxCollisionGeometry(new Vx::VxBox(C3Vector2VxVector3(C3Vector(0.1, 0.1, 0.1))), nullptr);
            partStatic->addCollisionGeometry(cgStatic);
            partDynamic->setPosition(C3Vector2VxVector3(C3Vector(0, 0, 100.0)));
            partStatic->setPosition(C3Vector2VxVector3(C3Vector(0, 0, 99.0)));
            Vx::VxFrame::currentInstance()->setTimeStep(0.1);
            for (int i = 0; i < 10; i++)
                Vx::VxFrame::currentInstance()->step();
            double zCoord = partDynamic->getTransform().t()[2];
            _dynamicsWorld->removePart(partDynamic);
            _dynamicsWorld->removePart(partStatic);
            delete partDynamic;
            delete partStatic;
            licensePresent = (zCoord > 98.0);
        }
        else
        {
            Vx::VxPart* partDynamic = new Vx::VxPart(1.0);
            _dynamicsWorld->addPart(partDynamic);
            partDynamic->setPosition(C3Vector2VxVector3(C3Vector(0, 0, 100.0)));
            Vx::VxFrame::currentInstance()->setTimeStep(0.1);
            for (int i = 0; i < 10; i++)
                Vx::VxFrame::currentInstance()->step();
            double zCoord = partDynamic->getTransform().t()[2];
            _dynamicsWorld->removePart(partDynamic);
            delete partDynamic;
            licensePresent = (zCoord < 99.0);
        }
        done = true;
        _checkingLicense = false;
    }
    if (!licensePresent)
        _simMakeDynamicAnnouncement(sim_announce_vortexpluginisdemo);
}

void CRigidBodyContainerDyn::_vortexCollisionCallbackStatic(void* data, Vx::VxCollisionGeometry* o1, Vx::VxCollisionGeometry* o2)
{ // this function is static and will call the corresponding function of the current object:
    ((CRigidBodyContainerDyn*)_dynWorld)->_vortexCollisionCallback(data, o1, o2);
}

void CRigidBodyContainerDyn::_vortexCollisionCallback(void* data, Vx::VxCollisionGeometry* o1, Vx::VxCollisionGeometry* o2)
{
    Vx::VxPart* b1 = getCollisionGeometryPart(o1);
    Vx::VxPart* b2 = getCollisionGeometryPart(o2);

    CXShape* shapeA = b1 == nullptr ? nullptr : (CXShape*)_simGetObject(b1->userData().getData("csim").getValueInteger());
    CXShape* shapeB = b2 == nullptr ? nullptr : (CXShape*)_simGetObject(b2->userData().getData("csim").getValueInteger());

    bool canCollide = false;
    int objID1;
    int objID2;

    if ((shapeA != nullptr) && (shapeB != nullptr))
    { // regular case (shape-shape)
        unsigned int collFA = _simGetDynamicCollisionMask(shapeA);
        unsigned int collFB = _simGetDynamicCollisionMask(shapeB);
        canCollide = _simIsShapeDynamicallyRespondable(shapeA) && _simIsShapeDynamicallyRespondable(shapeB) && ((_simGetTreeDynamicProperty(shapeA) & sim_objdynprop_respondable) != 0) && ((_simGetTreeDynamicProperty(shapeB) & sim_objdynprop_respondable) != 0);
        if (_simGetLastParentForLocalGlobalCollidable(shapeA) == _simGetLastParentForLocalGlobalCollidable(shapeB))
            canCollide = canCollide && (collFA & collFB & 0x00ff); // we are local
        else
            canCollide = canCollide && (collFA & collFB & 0xff00); // we are global
        if ((_simIsShapeDynamicallyStatic(shapeA) || ((_simGetTreeDynamicProperty(shapeA) & sim_objdynprop_dynamic) == 0)) &&
            (_simIsShapeDynamicallyStatic(shapeB) || ((_simGetTreeDynamicProperty(shapeB) & sim_objdynprop_dynamic) == 0)))
            canCollide = false;
    }
    else
    { // particle-shape or particle-particle case:
        CParticleObjectContainer_base* particleCont = CRigidBodyContainerDyn::getDynWorld()->getParticleCont();
        int dataA = (uint64_t)b1->userData().getData("csim").getValueInteger();
        int dataB = (uint64_t)b2->userData().getData("csim").getValueInteger();
        if ((shapeA == nullptr) && (shapeB == nullptr))
        { // particle-particle case:
            CParticleObject_base* pa = particleCont->getObject(dataA - CRigidBodyContainerDyn::getDynamicParticlesIdStart(), false);
            CParticleObject_base* pb = particleCont->getObject(dataB - CRigidBodyContainerDyn::getDynamicParticlesIdStart(), false);

            if ((pa != nullptr) && (pb != nullptr)) // added this condition on 08/02/2011 because of some crashes when scaling some models
            {
                canCollide = pa->isParticleRespondable() && pb->isParticleRespondable();
            }
        }
        else
        { // particle-shape case:
            CXShape* shape;
            CParticleObject_base* particle;
            if (shapeA != nullptr)
            {
                shape = shapeA;
                objID1 = _simGetObjectID(shapeA);
            }
            else
            {
                particle = particleCont->getObject(dataA - CRigidBodyContainerDyn::getDynamicParticlesIdStart(), false);
                objID1 = dataA;
            }
            if (shapeB != nullptr)
            {
                shape = shapeB;
                objID2 = _simGetObjectID(shapeB);
            }
            else
            {
                particle = particleCont->getObject(dataB - CRigidBodyContainerDyn::getDynamicParticlesIdStart(), false);
                objID2 = dataB;
            }

            if (particle != nullptr) // added this condition on 08/02/2011 because of some crashes when scaling some models
            {
                canCollide = _simIsShapeDynamicallyRespondable(shape) && (_simGetDynamicCollisionMask(shape) & particle->getShapeRespondableMask() & 0xff00) && ((_simGetTreeDynamicProperty(shape) & sim_objdynprop_respondable) != 0); // we are global
            }
        }
    }

    bool disableThisContact = true;
    if (canCollide)
    {
        int dataInt[3] = {0, 0, 0};
        double dataFloat[14] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // For now we won't allow modification of contact parameters when using VORTEX
        bool canReallyCollide = (_simHandleCustomContact(objID1, objID2, sim_physics_vortex, dataInt, dataFloat) != 0);
        if (canReallyCollide)
            disableThisContact = false;
    }

    bool* pCanCollide = (bool*)data;
    *pCanCollide = !disableThisContact;
}

void CRigidBodyContainerDyn::_applyGravity()
{ // gravity is scaled here!!

    C3Vector gravity;
    _simGetGravity(gravity.data);

    _dynamicsWorld->setGravity(C3Vector2VxVector3(gravity));

    gravityVectorLength = (gravity).getLength();
}

Vx::VxUniverse* CRigidBodyContainerDyn::getWorld() const
{
    return (_dynamicsWorld);
}

void CRigidBodyContainerDyn::serializeDynamicContent(const std::string& filenameAndPath, int maxSerializeBufferSize)
{
    _dynamicsWorld->printContent(filenameAndPath.c_str());
}

void CRigidBodyContainerDyn::_createDependenciesBetweenJoints()
{
    std::vector<CConstraintDyn*> allConstraints;
    _getAllConstraints(allConstraints);
    for (size_t i = 0; i < allConstraints.size(); i++)
    {
        CConstraintDyn* constr1 = allConstraints[i];
        int linkedJoint;
        double fact, off;
        if (((CConstraintDyn*)constr1)->getVortexDependencyInfo(linkedJoint, fact, off))
        {                 // when we enter here, the dependency flag is automatically cleared in constr1
            fact *= -1.0; // when fact is >0, we want to turn into the same direction!
            CConstraintDyn* constr2 = _getConstraintFromObjectHandle(linkedJoint, -1);
            if (constr2 != nullptr)
            {
                Vx::VxConstraint* joint1 = ((CConstraintDyn*)constr1)->getVortexConstraint();
                Vx::VxConstraint* joint2 = ((CConstraintDyn*)constr2)->getVortexConstraint();

                // We have following equation:
                // joint1=off+joint2*fact

                /*
                * The gear ratio constraint is setup to be initially at rest.
                * off doesn't really make sense unless there is a backlash and off < backlash/2
                * so that we can position the teeth relative position otherwise the gear will violently snap
                * back in equilibrium
                */

                // need to identified the gear parts. This is not great, we should have a better criteria
                int i0 = joint1->getPart(0)->getControl() == Vx::VxPart::kControlDynamic ? 0 : 1;
                int i1 = joint2->getPart(0)->getControl() == Vx::VxPart::kControlDynamic ? 0 : 1;

                // get gear parts attachment info
                Vx::VxVector3 a00, a01, a10, a11, p0, p1;
                joint1->getPartAttachmentPositionRel(i0, p0);
                joint2->getPartAttachmentPositionRel(i1, p1);
                joint1->getPartAttachmentAxesRel(i0, a00, a01);
                joint2->getPartAttachmentAxesRel(i1, a10, a11);

                // set gear parts
                Vx::VxGearRatio* gr = new Vx::VxGearRatio();
                gr->setPartAndAttachmentRel(0, joint1->getPart(i0), p0, a00, a01);
                gr->setPartAndAttachmentRel(1, joint2->getPart(i1), p1, a10, a11);

                // set the gears reference parts and attachments.
                // note here if we only support geears being attached to the same part, se can simply call gr->setengageMode(true) instead.
                const int other[2] = {1, 0};
                gr->getPartAttachmentPosition(0, p0);
                gr->getPartAttachmentAxes(0, a00, a01);
                gr->getPartAttachmentPosition(1, p1);
                gr->getPartAttachmentAxes(1, a10, a11);
                gr->setPartAndAttachment(2, joint1->getPart(other[i0]), p0, a00, a01);
                gr->setPartAndAttachment(3, joint2->getPart(other[i1]), p1, a10, a11);

                // support linear and angular gear, a hinge supports a rotation gear while a prismatic supports a linear one
                if (joint1->isOfExactClassType(Vx::VxPrismatic::getStaticClassType()))
                {
                    gr->setMotionAngular(0, false);
                }
                if (joint2->isOfExactClassType(Vx::VxPrismatic::getStaticClassType()))
                {
                    gr->setMotionAngular(1, false);
                }

                // set this to false to simulate a belt that could be sliding on the gear.
                // use gr->setMaxTorque(maxTorque) and gr->setMinTorque(-maxTorque) to tune the belt friction force
                // note that setting the maxTorque on the positional gear allow simulate rubber teeth gear where teeth jumping may occur.
                gr->setUpdateMode(Vx::VxGearRatio::kPositional);

                // set the gear ratio. Not sure if I interpret correctly here as in the exemple the gear size not set, could be 1.0/fact
                gr->setGearRatio(fact);
                gr->resetPositions(); // for positional gear, this sets the gear at equilibrium position

                const Vx::VxReal backLash = 0;
                gr->setBacklash(backLash);
                if (fabs(off) < backLash)
                {
                    // not sure about the sign here
                    gr->setPartReferencePosition(0, off + gr->getPartReferencePosition(0));
                }

                // internally, the gear error is filter for smoother result. The is the constraint violation mean lifetime
                // which should be larger than the time step.
                gr->setPositionOffsetMeanLifeTime(Vx::VxFrame::currentInstance()->getTimeStep() * 10);

                if (joint1->getUniverse())
                {
                    //Vx::VxInfo(0, "gr added");
                    joint1->getUniverse()->addConstraint(gr);
                }
                else
                {
                    // in this case th egear will never be added to the universe, I guess this cannot happen?!?
                }

                // this is used to destroy the gear for now...
                SVortexJointDependency dep;
                dep.gear = gr;
                dep.constr1 = constr1;
                dep.constr2 = constr2;
                _gears.push_back(dep);
            }
        }
    }
}

void CRigidBodyContainerDyn::_removeDependenciesBetweenJoints(CConstraintDyn* theInvolvedConstraint)
{
    size_t i = 0;
    while (i < _gears.size())
    {
        if ((_gears[i].constr1 == theInvolvedConstraint) || (_gears[i].constr2 == theInvolvedConstraint))
        {
            Vx::VxConstraint* c = _gears[i].gear;
            if (c->getUniverse())
                c->getUniverse()->removeConstraint(c);
            delete c;
            _gears.erase(_gears.begin() + i);
        }
        else
            i++;
    }
}

void CRigidBodyContainerDyn::_addVortexContactPoints(int dynamicPassNumber)
{
    Vx::VxUniverse::DynamicsContactIterator it = _dynamicsWorld->dynamicsContactBegin();
    Vx::VxUniverse::DynamicsContactIterator ie = _dynamicsWorld->dynamicsContactEnd();

    while (it != ie)
    {
        if ((*it)->isEnabled())
        {
            Vx::VxReal3 pos, force, n;
            (*it)->getPosition(pos);
            (*it)->getNormal(n);
            C3Vector n2(VxVector32C3Vector(n));
            n2.normalize();
            C3Vector pos2(VxVector32C3Vector(pos));
            SContactInfo ci;
            ci.subPassNumber = dynamicPassNumber;
            Vx::VxPart* part1;
            Vx::VxPart* part2;
            (*it)->getPartPair(&part1, &part2);
            ci.objectID1 = part1->userData().getData("csim").getValueInteger();
            ci.objectID2 = part2->userData().getData("csim").getValueInteger();
            ci.position = pos2;
            if (part1->getControl() == Vx::VxPart::kControlDynamic)
                (*it)->getForce(0, force);
            else
                (*it)->getForce(1, force);
            C3Vector force2(VxVector32C3Vector(force));
            ci.directionAndAmplitude = force2;
            if (force2 * n2 < 0.0)
                n2 = n2 * -1.0;
            ci.surfaceNormal = n2;
            _contactInfo.push_back(ci);

            _contactPoints.push_back(pos2(0));
            _contactPoints.push_back(pos2(1));
            _contactPoints.push_back(pos2(2));
            //   _contactPoints.push_back(pos2(0)+n[0]*0.01);
            //   _contactPoints.push_back(pos2(1)+n[1]*0.01);
            //   _contactPoints.push_back(pos2(2)+n[2]*0.01);
        }
        ++it;
    }

    /*  auto angular damping on the parts. Angular damping helps reducing possible instabilities on light object
    *  submitted to large pressure or tension from heavier objetcs.
    */
    for (Vx::VxPartSet::const_iterator it = _dynamicsWorld->getParts().begin(), itE = _dynamicsWorld->getParts().end(); it != itE; ++it)

    {
        Vx::VxPart* part = *it;
        if (part->getControl() != Vx::VxPart::kControlDynamic)
            continue;

        CRigidBodyDyn* body = (CRigidBodyDyn*)part->userData().getData("csim1").getPointerVoid();
        if ((body != nullptr) && (((CRigidBodyDyn*)body)->vortex_autoAngularDampingTensionRatio != 0.0))
        { // auto angular damping is enabled
            const Vx::VxReal pAutoAngularDampingTensionAdded = ((CRigidBodyDyn*)body)->vortex_angularVelocityDamping;
            const Vx::VxReal pAutoAngularDampingTensionTreshold = part->getMass();
            const Vx::VxReal pAutoAngularDampingTensionRatio = ((CRigidBodyDyn*)body)->vortex_autoAngularDampingTensionRatio;

            Vx::VxReal sumF = 0;
            for (int i = 0; i < part->getConstraintCount(); ++i)
            {
                Vx::VxVector3 v;
                Vx::VxConstraint* c = part->getConstraint(i);
                int index = 0;
                if (c->getPart(0) != part)
                {
                    index = 1;
                }
                c->getPartForce(index, v);
                sumF += v.norm();
            }
            for (int i = 0; i < part->getContactCount(); ++i)
            {
                Vx::VxVector3 v;
                Vx::VxDynamicsContact* c = part->getContact(i)->getContact();
                int index = 0;
                Vx::VxPart* p[2];
                c->getPartPair(p, p + 1);
                if (p[0] != part)
                {
                    index = 1;
                }
                c->getForce(index, v);
                sumF += v.norm();
            }
            sumF /= gravityVectorLength;
            Vx::VxReal angularDamping = pAutoAngularDampingTensionAdded;
            if (sumF > pAutoAngularDampingTensionTreshold)
            {
                const Vx::VxReal excess = sumF - pAutoAngularDampingTensionTreshold;
                angularDamping += (excess * pAutoAngularDampingTensionRatio);
            }
            part->setAngularVelocityDamping(angularDamping);
        }
    }
}

void CRigidBodyContainerDyn::_stepDynamics(double dt, int pass)
{
    if (VxFrameReleased)
    {
        VxFrameReleased = false;
        //_dynamicsWorld->printContent("vxuniverse.txt");
    }
    //Vx::VxInfo(0, "trial=%d, time remaining=%g\n", Vx::LicensingManager::isRunningTrial(), Vx::LicensingManager::getTrialRemainingSeconds());
    Vx::VxFrame::currentInstance()->setTimeStep((Vx::VxReal)dt);
    Vx::VxFrame::currentInstance()->step();
    _addVortexContactPoints(pass);
}
