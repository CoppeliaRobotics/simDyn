#include <ParticleDyn.h>
#include <RigidBodyContainerDyn.h>
#include <simLib/simLib.h>
#include <Vx/VxUniverse.h>
#include <Vx/VxSphere.h>
#include <Vx/VxPart.h>
#include <Vx/VxCollisionGeometry.h>
#include <Vx/VxMaterialTable.h>
#include <Vx/VxResponseModel.h>
#include <Vx/VxRigidBodyResponseModel.h>
#include <VortexConvertUtil.h>
#include <boost/lexical_cast.hpp>

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,double size,double massOverVolume,double killTime,float addColor[3]) : CParticleDyn_base(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(double parameters[18],int objectID)
{ // return value indicates if there are particles that need to be simulated

    if (_initializationState!=0)
        return(_initializationState==1);
    _initializationState=1;

    CRigidBodyContainerDyn* rbc=(CRigidBodyContainerDyn*)(CRigidBodyContainerDyn::getDynWorld());
    Vx::VxUniverse* vortexWorld=rbc->getWorld();

    // Create a simple sphere that will act as a particle:
    double mass=_massOverVolume*(4.0*piValue*(_size*0.5)*(_size*0.5)*(_size*0.5)/3.0);

    double I=2.0*(_size*0.5)*(_size*0.5)/5.0;
    _vortexRigidBody = new Vx::VxPart(mass);
    Vx::VxReal33 inertia = { {I, 0, 0}, {0, I, 0}, {0, 0, I} };
    _vortexRigidBody->setMassAndInertia(mass, inertia);

    _vortexRigidBody->setPosition(C3Vector2VxVector3(_currentPosition));
    _vortexRigidBody->userData().setData(Vx::VxUserData("csim",CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart()+objectID));


    std::string materialString("CSIMM");
    materialString+=boost::lexical_cast<std::string>(objectID);
    Vx::VxSmartPtr<Vx::VxRigidBodyResponseModel> responseModel=vortexWorld->getRigidBodyResponseModel();
    Vx::VxMaterial* material=responseModel->getMaterialTable()->getMaterial(materialString.c_str());

    if (material==nullptr)
    {
        material = responseModel->getMaterialTable()->registerMaterial(materialString.c_str());

        if (parameters[9]!=0.0)
        {
            material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinear,Vx::VxMaterial::kFrictionModelScaledBoxFast);
        }
        else
            material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinear,Vx::VxMaterial::kFrictionModelNone);

        // Adhesive force. Vortex default: 0.0
        material->setAdhesiveForce(parameters[14]);

        // Compliance. Vortex default: 0.0
        material->setCompliance(parameters[12]);

        // Damping. vortex default: 0.0
        material->setDamping(parameters[13]);

        // Restitution. Vortex default: 0.0
        material->setRestitution(parameters[10]);

        // Restitution threshold. Vortex default: 0.001
        material->setRestitutionThreshold(parameters[11]);

        // Slide. Vortex default: 0.0
        // material->setSlide(Vx::VxMaterial::kFrictionAxisLinear,slide_primary_linearAxis);

        // friction coeff. Vortex default: 0.0
        // material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisLinear,frictionCoeff_primary_linearAxis);

        // Slip. Vortex default: 0.0
        // material->setSlip(Vx::VxMaterial::kFrictionAxisLinear,slip_primary_linearAxis);
    }

    Vx::VxCollisionGeometry* cg = new Vx::VxCollisionGeometry(new Vx::VxSphere(_size/2.0), material);
    _vortexRigidBody->addCollisionGeometry(cg);
    cg->enableFastMoving(true);
    vortexWorld->addPart(_vortexRigidBody);
 
    // For now, we disable the auto-disable functionality (because there are problems when removing a kinematic object during simulation(e.g. removing the floor, nothing falls)):
    //dBodySetAutoDisableFlag(_vortexRigidBody,0);

    _vortexRigidBody->setLinearVelocity(C3Vector2VxVector3(_initialVelocityVector));
    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,double linearFluidFrictionCoeff,double quadraticFluidFrictionCoeff,double linearAirFrictionCoeff,double quadraticAirFrictionCoeff)
{
    bool isWaterButInAir=false;
    if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
    {
        double mass=_massOverVolume*((piValue*_size*_size*_size)/6.0);
        C3Vector f=gravity*-mass;
        bool reallyIgnoreGravity=true;
        if (_objectType&sim_particle_water)
        { // We ignore gravity only if we are in the water (z<0) (New since 27/6/2011):
            if (double(_vortexRigidBody->getTransform().t()[2])>=0.0)
            {
                reallyIgnoreGravity=false;
                isWaterButInAir=true;
            }
        }

        if (reallyIgnoreGravity)
        {
            _vortexRigidBody->addForce(C3Vector2VxVector3(f)); // TODO f is -gravity?
        }
    }
    if ((linearFluidFrictionCoeff!=0.0)||(quadraticFluidFrictionCoeff!=0.0)||(linearAirFrictionCoeff!=0.0)||(quadraticAirFrictionCoeff!=0.0))
    { // New since 27/6/2011
        double lfc=linearFluidFrictionCoeff;
        double qfc=quadraticFluidFrictionCoeff;
        if (isWaterButInAir)
        {
            lfc=linearAirFrictionCoeff;
            qfc=quadraticAirFrictionCoeff;
        }
        C3Vector vVect;

        vVect = VxVector32C3Vector(_vortexRigidBody->getLinearVelocity());

        double v=vVect.getLength();
        if (v!=0.0)
        {
            C3Vector nv(vVect.getNormalized()*-1.0);
            C3Vector f(nv*(v*lfc+v*v*qfc));
            _vortexRigidBody->addForce(C3Vector2VxVector3(f));
        }
    }
}

void CParticleDyn::removeFromEngine()
{
    if (_initializationState==1)
    {
        if (_vortexRigidBody->getUniverse())
            _vortexRigidBody->getUniverse()->removePart(_vortexRigidBody);
        delete _vortexRigidBody;
        _initializationState=2;
    }
}

void CParticleDyn::updatePosition()
{
    if (_initializationState==1)
    {
        const Vx::VxVector3& pos=_vortexRigidBody->getTransform().t();
        _currentPosition(0)=pos[0];
        _currentPosition(1)=pos[1];
        _currentPosition(2)=pos[2];
    }
}
