#include <ParticleObjectContainer_base.h>
#include <RigidBodyContainerDyn.h>
#include <simLib/simLib.h>

CParticleObjectContainer_base::CParticleObjectContainer_base()
{
    nextParticleObjectId=0;
}

CParticleObjectContainer_base::~CParticleObjectContainer_base()
{
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        delete it->second;
        _allParticleObjects.erase(it);
        it=_allParticleObjects.begin();
    }
}

CParticleObject_base* CParticleObjectContainer_base::getObject(int objectId,bool getAlsoTheOnesFlaggedForDestruction)
{
    CParticleObject_base* retVal=nullptr;
    auto it=_allParticleObjects.find(objectId);
    if (it!=_allParticleObjects.end())
    {
        if ( getAlsoTheOnesFlaggedForDestruction||(!it->second->isFlaggedForDestruction()) )
            retVal=it->second;
    }
    return(retVal);
}

int CParticleObjectContainer_base::addObject(CParticleObject_base* it)
{
    int retVal=nextParticleObjectId++;
    _allParticleObjects[retVal]=it;
    it->setObjectId(retVal);
    if ( (it->getLifeTime()<0.0)&&(it->getSize()<-100.0) )
        return(131183);
    return(retVal);
}

void CParticleObjectContainer_base::removeAllObjects()
{
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        delete it->second;
        _allParticleObjects.erase(it);
        it=_allParticleObjects.begin();
    }
}

void CParticleObjectContainer_base::removeObject(int objectId)
{
    auto it=_allParticleObjects.find(objectId);
    if (it!=_allParticleObjects.end())
        it->second->flagForDestruction();
}

void** CParticleObjectContainer_base::getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
    auto it=_allParticleObjects.begin();
    int cnt=0;
    particlesCount[0]=-1;
    while (it!=_allParticleObjects.end())
    {
        if (cnt==index)
        {
            if (!it->second->isFlaggedForDestruction())
                return(it->second->getParticles(particlesCount,objectType,cols));
            return(nullptr);
        }
        cnt++;
        ++it;
    }
    return(nullptr);
}

bool CParticleObjectContainer_base::addParticlesIfNeeded()
{ // return value indicates if there are particles that need to be simulated
    bool particlesPresent=false;
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        if (!it->second->isFlaggedForDestruction())
            particlesPresent|=it->second->addParticlesIfNeeded();
        ++it;
    }
    return(particlesPresent);
}

void CParticleObjectContainer_base::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity)
{
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        if (!it->second->isFlaggedForDestruction())
            it->second->handleAntiGravityForces_andFluidFrictionForces(gravity);
        ++it;
    }
}

void CParticleObjectContainer_base::removeKilledParticles()
{
    std::vector<int> toRemove;
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        if (it->second->isFlaggedForDestruction())
            toRemove.push_back(it->first);
        else
            it->second->removeKilledParticles();
        ++it;
    }
    for (size_t i=0;i<toRemove.size();i++)
    {
        auto it=_allParticleObjects.find(toRemove[i]);
        it->second->removeAllParticles();
        delete it->second;
        _allParticleObjects.erase(it);
    }
}

void CParticleObjectContainer_base::removeAllParticles()
{
    std::vector<int> toRemove;
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        if (it->second->isFlaggedForDestruction())
            toRemove.push_back(it->first);
        else
            it->second->removeAllParticles();
        ++it;
    }
    for (size_t i=0;i<toRemove.size();i++)
    {
        auto it=_allParticleObjects.find(toRemove[i]);
        it->second->removeAllParticles();
        delete it->second;
        _allParticleObjects.erase(it);
    }
}

void CParticleObjectContainer_base::updateParticlesPosition(sReal simulationTime)
{
    auto it=_allParticleObjects.begin();
    while (it!=_allParticleObjects.end())
    {
        if (!it->second->isFlaggedForDestruction())
            it->second->updateParticlesPosition(simulationTime);
        ++it;
    }
}



