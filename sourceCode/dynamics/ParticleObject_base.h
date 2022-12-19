#pragma once

#include "ParticleDyn.h"
#include <vector>

class CParticleObject_base
{
public:
    CParticleObject_base(int theObjectType,sReal size,sReal massVolumic,const void* params,sReal lifeTime,int maxItemCount);
    virtual ~CParticleObject_base();

    void setObjectId(int newID);
    void addParticle(sReal simulationTime,const sReal* itemData);
    int getOtherFloatsPerItem();

    bool isParticleRespondable();
    int getShapeRespondableMask();
    bool canBeDestroyed();
    void flagForDestruction();
    bool isFlaggedForDestruction();
    sReal getLifeTime();
    sReal getSize();

    bool addParticlesIfNeeded();
    void removeKilledParticles();
    void removeAllParticles();
    void updateParticlesPosition(sReal simulationTime);

    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

    void** getParticles(int* particlesCount,int* objectType,float** col);

    float color[12];
    sReal parameters[18];

protected:
    int _objectId;
    int _nextUniqueIDForParticle;

    int _objectType;
    sReal _size;
    sReal _massVolumic;
    sReal _particlesLifeTime;
    int _maxItemCount;
    bool _flaggedForDestruction;

    std::vector<CParticleDyn*> _particles;
    std::vector<CParticleDyn*> _particlesToDestroy;
};
