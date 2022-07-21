#pragma once

#include "ParticleDyn.h"
#include <vector>

class CParticleObject_base
{
public:
    CParticleObject_base(int theObjectType,float size,float massVolumic,const void* params,float lifeTime,int maxItemCount);
    virtual ~CParticleObject_base();

    void setObjectId(int newID);
    void addParticle(float simulationTime,const float* itemData);
    int getOtherFloatsPerItem();

    bool isParticleRespondable();
    int getShapeRespondableMask();
    bool canBeDestroyed();
    void flagForDestruction();
    bool isFlaggedForDestruction();
    float getLifeTime();
    float getSize();

    bool addParticlesIfNeeded();
    void removeKilledParticles();
    void removeAllParticles();
    void updateParticlesPosition(float simulationTime);

    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

    void** getParticles(int* particlesCount,int* objectType,float** col);

    float color[12];
    float parameters[18];

protected:
    int _objectId;
    int _nextUniqueIDForParticle;

    int _objectType;
    float _size;
    float _massVolumic;
    float _particlesLifeTime;
    int _maxItemCount;
    bool _flaggedForDestruction;

    std::vector<CParticleDyn*> _particles;
    std::vector<CParticleDyn*> _particlesToDestroy;
};
