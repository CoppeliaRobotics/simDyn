#pragma once

#include <ParticleObject_base.h>
#include <map>

class CParticleObjectContainer_base
{
  public:
    CParticleObjectContainer_base();
    virtual ~CParticleObjectContainer_base();

    int addObject(CParticleObject_base* it);
    CParticleObject_base* getObject(int objectId, bool getAlsoTheOnesFlaggedForDestruction);
    void removeAllObjects();
    void removeObject(int objectId);
    void** getParticles(int index, int* particlesCount, int* objectType, float** cols);

    bool addParticlesIfNeeded();
    void removeKilledParticles();
    void removeAllParticles();
    void updateParticlesPosition(sReal simulationTime);

    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

  private:
    int nextParticleObjectId;
    std::map<int, CParticleObject_base*> _allParticleObjects;
};
