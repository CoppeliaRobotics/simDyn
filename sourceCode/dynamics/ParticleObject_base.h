#pragma once

#include "ParticleDyn.h"
#include <vector>

class CParticleObject_base
{
public:
    CParticleObject_base(int theObjectType,double size,double massVolumic,const void* params,double lifeTime,int maxItemCount);
    virtual ~CParticleObject_base();

    void setObjectId(int newID);
    void addParticle(double simulationTime,const double* itemData);
    int getOtherFloatsPerItem();

    bool isParticleRespondable();
    int getShapeRespondableMask();
    bool canBeDestroyed();
    void flagForDestruction();
    bool isFlaggedForDestruction();
    double getLifeTime();
    double getSize();

    bool addParticlesIfNeeded();
    void removeKilledParticles();
    void removeAllParticles();
    void updateParticlesPosition(double simulationTime);

    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

    void** getParticles(int* particlesCount,int* objectType,float** col);

    float color[12];
    double parameters[18];

protected:
    int _objectId;
    int _nextUniqueIDForParticle;

    int _objectType;
    double _size;
    double _massVolumic;
    double _particlesLifeTime;
    int _maxItemCount;
    bool _flaggedForDestruction;

    std::vector<CParticleDyn*> _particles;
    std::vector<CParticleDyn*> _particlesToDestroy;
};
