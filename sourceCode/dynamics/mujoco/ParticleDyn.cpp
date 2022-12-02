#include "ParticleDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"

CXmlSer* CParticleDyn::xmlDoc=nullptr;
std::vector<SMjGeom>* CParticleDyn::allGeoms=nullptr;
mjModel* CParticleDyn::mjModel=nullptr;
mjData* CParticleDyn::mjData=nullptr;

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,double size,double massOverVolume,double killTime,float addColor[3]) : CParticleDyn_base(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
    _initVelSet=false;
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(double parameters[18],int objectID)
{
    if (_initializationState>1)
        return(false);
    _initializationState=1;

    _name="_particle_";
    _name+=std::to_string(objectID)+"_";
    _name+=std::to_string(_uniqueID);

    xmlDoc->pushNewNode("body");
    xmlDoc->setAttr("name",_name.c_str());
    xmlDoc->setPosAttr("pos",_currentPosition.data);

    xmlDoc->pushNewNode("geom");
    xmlDoc->setAttr("name",_name.c_str());
    xmlDoc->setAttr("type","sphere");
    xmlDoc->setAttr("size",_size*0.5);
    xmlDoc->setAttr("density",_massOverVolume);
    xmlDoc->setAttr("pos",0.0,0.0,0.0);
    // Following 2 for frictionless contacts:
    xmlDoc->setAttr("condim",1);
    xmlDoc->setAttr("priority",1);
    xmlDoc->popNode();

    xmlDoc->pushNewNode("freejoint");
    xmlDoc->setAttr("name",(_name+"_freejoint").c_str());
    xmlDoc->popNode();
    xmlDoc->popNode();

    SMjGeom g;
    g.belongsToStaticItem=false;
    g.itemType=particleItem;
    g.objectHandle=-2;//CRigidBodyContainerDyn::getDynWorld()->getDynamicParticlesIdStart()+_uniqueID;
    g.name=_name;
    g.respondableMask=0;
    if (_objectType&sim_particle_particlerespondable)
        g.respondableMask|=0x00ff;
    if (_objectType&sim_particle_respondable1to4)
        g.respondableMask|=0x0f00;
    if (_objectType&sim_particle_respondable5to8)
        g.respondableMask|=0xf000;
    allGeoms->push_back(g);
    return(true);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,double linearFluidFrictionCoeff,double quadraticFluidFrictionCoeff,double linearAirFrictionCoeff,double quadraticAirFrictionCoeff)
{
    if (_initializationState==1)
    {
        if (!_initVelSet)
        {
            _body_mjId=mj_name2id(mjModel,mjOBJ_BODY,_name.c_str());

            int nvadr=mjModel->body_dofadr[_body_mjId];
            if (_initialVelocityVector.getLength()>0.0)
            {
                mjData->qvel[nvadr+0]=_initialVelocityVector(0);
                mjData->qvel[nvadr+1]=_initialVelocityVector(1);
                mjData->qvel[nvadr+2]=_initialVelocityVector(2);
                _initialVelocityVector.clear();
            }
            _initVelSet=true;
        }

        bool isWaterButInAir=false;
        if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
        {
            double mass=_massOverVolume*((piValue*_size*_size*_size)/6.0);
            C3Vector f=gravity*-mass;
            bool reallyIgnoreGravity=true;
            if (_objectType&sim_particle_water)
            { // We ignore gravity only if we are in the water (z<0):
                if (mjData->xpos[3*_body_mjId+2]>=0.0)
                {
                    reallyIgnoreGravity=false;
                    isWaterButInAir=true;
                }
            }

            if (reallyIgnoreGravity)
            {
                mjData->xfrc_applied[6*_body_mjId+0]=f(0);
                mjData->xfrc_applied[6*_body_mjId+1]=f(1);
                mjData->xfrc_applied[6*_body_mjId+2]=f(2);
            }
        }

        if ((linearFluidFrictionCoeff!=0.0)||(quadraticFluidFrictionCoeff!=0.0)||(linearAirFrictionCoeff!=0.0)||(quadraticAirFrictionCoeff!=0.0))
        {
            double lfc=linearFluidFrictionCoeff;
            double qfc=quadraticFluidFrictionCoeff;
            if (isWaterButInAir)
            {
                lfc=linearAirFrictionCoeff;
                qfc=quadraticAirFrictionCoeff;
            }
            C3Vector vVect(mjData->cvel[6*_body_mjId+3],mjData->cvel[6*_body_mjId+4],mjData->cvel[6*_body_mjId+5]);
            double v=vVect.getLength();
            if (v!=0.0)
            {
                C3Vector nv(vVect.getNormalized()*-1.0);
                C3Vector f(nv*(v*lfc+v*v*qfc));

                mjData->xfrc_applied[6*_body_mjId+0]+=f(0);
                mjData->xfrc_applied[6*_body_mjId+1]+=f(1);
                mjData->xfrc_applied[6*_body_mjId+2]+=f(2);
            }
        }
    }
}

void CParticleDyn::removeFromEngine()
{
    if (_initializationState==1)
        _initializationState=2;
}

void CParticleDyn::updatePosition()
{
    if (_initializationState==1)
    {
        _currentPosition(0)=mjData->xpos[3*_body_mjId+0];
        _currentPosition(1)=mjData->xpos[3*_body_mjId+1];
        _currentPosition(2)=mjData->xpos[3*_body_mjId+2];
    }
}
