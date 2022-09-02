#pragma once

#include "ParticleDyn_base.h"
#include "xmlser.h"
#include <mujoco/mujoco.h>

struct SMjGeom;
struct SMjShape;

class CParticleDyn : public CParticleDyn_base
{
public:
    CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]);
    virtual ~CParticleDyn();

    bool addToEngineIfNeeded(float parameters[18],int objectID);
    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff);
    void updatePosition();
    void removeFromEngine();

    std::string _name;
    bool _initVelSet;
    int _body_mjId;


    static CXmlSer* xmlDoc;
    static std::vector<SMjGeom>* allGeoms;
    static mjModel* mjModel;
    static mjData* mjData;
};
