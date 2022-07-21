#include "ParticleObject_base.h"
#include "simLib.h"
#include "ParticleDyn.h"

CParticleObject_base::CParticleObject_base(int theObjectType,float size,float massVolumic,const void* params,float lifeTime,int maxItemCount)
{
    _objectId=0;
    if (size>100.0f)
        size=100.0f;
    _size=size;
    _massVolumic=massVolumic;
    _particlesLifeTime=lifeTime;
    if (maxItemCount>10000000)
        maxItemCount=10000000;
    if (maxItemCount<1)
        maxItemCount=1;
    _maxItemCount=maxItemCount;

    _objectType=theObjectType;
    _nextUniqueIDForParticle=0;
    _flaggedForDestruction=false;
    parameters[0]=0.0f; // Bullet friction
    parameters[1]=0.0f; // Bullet restitution
    parameters[2]=0.0f; // ODE friction
    parameters[3]=0.2f; // ODE soft ERP
    parameters[4]=0.0f; // ODE soft CFM
    parameters[5]=0.0f; // ODE, Bullet, Newton and Vortex linear friction (water and air friction, or just water if sim_particle_water)
    parameters[6]=0.0f; // ODE, Bullet, Newton and Vortex quadratic friction (water and air friction, or just water if sim_particle_water)
    parameters[7]=0.0f; // ODE, Bullet, Newton and Vortex linear friction (air friction if sim_particle_water)
    parameters[8]=0.0f; // ODE, Bullet, Newton and Vortex quadratic friction (air friction if sim_particle_water)

    parameters[9]=0.0f; // Vortex friction
    parameters[10]=0.0f; // Vortex restitution
    parameters[11]=0.001f; // Vortex restitution threshold
    parameters[12]=0.0f; // Vortex compliance
    parameters[13]=0.0f; // Vortex damping
    parameters[14]=0.0f; // Vortex adhesive force

    parameters[15]=0.0f; // Newton static friction
    parameters[16]=0.0f; // Newton kinematic friction
    parameters[17]=0.0f; // Newton restitution
    if (params!=nullptr)
    {
        int cnt=((int*)params)[0];
        for (int i=0;i<cnt;i++)
        {
            int p=((int*)params)[1+2*i+0];
            float f=((float*)params)[1+2*i+1];
            if ((p>=0)&&(p<18))
                parameters[p]=f;
        }
    }
}

CParticleObject_base::~CParticleObject_base()
{
    for (size_t i=0;i<_particles.size();i++)
        delete _particles[i];
    for (size_t i=0;i<_particlesToDestroy.size();i++)
        delete _particlesToDestroy[i];
}

bool CParticleObject_base::canBeDestroyed()
{
    for (size_t i=0;i<_particles.size();i++)
    {
        if (_particles[i]!=nullptr)
        {
            if (_particles[i]->getInitializationState()==1)
                return(false);
        }
    }
    for (size_t i=0;i<_particlesToDestroy.size();i++)
    {
        if (_particlesToDestroy[i]!=nullptr)
        {
            if (_particlesToDestroy[i]->getInitializationState()==1)
                return(false);
        }
    }
    return(true);
}

float CParticleObject_base::getLifeTime()
{
    return(_particlesLifeTime);
}

float CParticleObject_base::getSize()
{
    return(_size);
}

void CParticleObject_base::flagForDestruction()
{
    _flaggedForDestruction=true;
}

bool CParticleObject_base::isFlaggedForDestruction()
{
    return(_flaggedForDestruction);
}

void CParticleObject_base::setObjectId(int newID)
{
    _objectId=newID;
}

int CParticleObject_base::getOtherFloatsPerItem()
{
    int retVal=0;
    if (_objectType&sim_particle_itemsizes)
        retVal++;
    if (_objectType&sim_particle_itemdensities)
        retVal++;
    if (_objectType&sim_particle_itemcolors)
        retVal+=3;
    return(retVal);
}

void CParticleObject_base::addParticle(float simulationTime,const float* itemData)
{
    if (itemData==nullptr)
    { // We wanna remove all particles!
        for (size_t i=0;i<_particles.size();i++)
        {
            if (_particles[i]!=nullptr)
            {
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // This still needs removal from the physics engine!
                else
                    delete _particles[i]; // ok to delete it here
            }
        }
        _particles.clear();
        return;
    }
    
    // Now we first wanna remove particles that have timed out or that are not active anymore:
    int oldestIndex=-1;
    int firstFreeIndex=-1;
    int smallestUniqueID=_nextUniqueIDForParticle+2;
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=nullptr)
        {
            if ( _particles[i]->didTimeOut(simulationTime)||(_particles[i]->getInitializationState()==2) )
            { // We have to remove this one from _particles:
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // We cannot destroy this one now
                else
                    delete _particles[i]; // We can directly destroy this one here
                _particles[i]=nullptr;
                if (firstFreeIndex==-1)
                    firstFreeIndex=i;
            }
            else
            { // this one stays a bit more (or will be removed just after if the list is full))
                int uniqueID=_particles[i]->getUniqueID();
                if (smallestUniqueID>uniqueID)
                {
                    smallestUniqueID=uniqueID;
                    oldestIndex=i;
                }
            }
        }
        else
        {
            if (firstFreeIndex==-1)
                firstFreeIndex=i;
        }
    }

    int positionToAddNewParticle=firstFreeIndex;
    
    if (positionToAddNewParticle==-1)
    { // we don't have a free spot anymore:
        if (int(_particles.size())>=_maxItemCount)
        { // the buffer is full
            if (_objectType&sim_particle_cyclic)
            { // the buffer is cyclic. We remove the oldest element:
                _particlesToDestroy.push_back(_particles[oldestIndex]);
                _particles[oldestIndex]=nullptr;
                positionToAddNewParticle=oldestIndex;
            }
            else
                return; // saturated
        }
        else
        { // The buffer is not yet full!
            positionToAddNewParticle=_particles.size();
            _particles.push_back(nullptr);
        }
    }


    float size=_size;
    float massOverVolume=_massVolumic;
    int off=6;
    if (_objectType&sim_particle_itemsizes)
        size=itemData[off++];
    if (_objectType&sim_particle_itemdensities)
        massOverVolume=itemData[off++];
    float addColor[3];
    float* additionalColor=nullptr;
    if (_objectType&sim_particle_itemcolors)
    {
        additionalColor=addColor;
        addColor[0]=itemData[off++];
        addColor[1]=itemData[off++];
        addColor[2]=itemData[off++];
    }
    float killTime=SIM_MAX_FLOAT;
    if (_particlesLifeTime!=0.0f)
        killTime=simulationTime+_particlesLifeTime;
    C3Vector pos(itemData);
    C3Vector vel(itemData+3);
    vel-=pos;
    _particles[positionToAddNewParticle]=new CParticleDyn(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
    _particles[positionToAddNewParticle]->setUniqueID(_nextUniqueIDForParticle++);
}

void** CParticleObject_base::getParticles(int* particlesCount,int* objectType,float** col)
{
    particlesCount[0]=int(_particles.size());
    objectType[0]=_objectType;
    col[0]=color;
    if (particlesCount[0]>0)
        return((void**)&_particles[0]);
    particlesCount[0]=0;
    return(nullptr);
}

bool CParticleObject_base::isParticleRespondable()
{
    return((_objectType&sim_particle_particlerespondable)!=0);
}

int CParticleObject_base::getShapeRespondableMask()
{
    int retVal=0;
    if (_objectType&sim_particle_respondable1to4)
        retVal=0x0f00;
    if (_objectType&sim_particle_respondable5to8)
        retVal|=0xf000;
    return(retVal);
}

bool CParticleObject_base::addParticlesIfNeeded()
{ // return value indicates if there are particles that need to be simulated
    bool particlesPresent=false;
    for (size_t i=0;i<_particles.size();i++)
    {
        if (_particles[i]!=nullptr)
            particlesPresent|=_particles[i]->addToEngineIfNeeded(parameters,_objectId);
    }
    return(particlesPresent);
}

void CParticleObject_base::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity)
{
    for (size_t i=0;i<_particles.size();i++)
    {
        if (_particles[i]!=nullptr)
            _particles[i]->handleAntiGravityForces_andFluidFrictionForces(gravity,parameters[5],parameters[6],parameters[7],parameters[8]);
    }
}

void CParticleObject_base::removeKilledParticles()
{
    for (size_t i=0;i<_particlesToDestroy.size();i++)
    {
        _particlesToDestroy[i]->removeFromEngine();
        delete _particlesToDestroy[i];
    }
    _particlesToDestroy.clear();
}

void CParticleObject_base::removeAllParticles()
{
    for (size_t i=0;i<_particles.size();i++)
    {
        if (_particles[i]!=nullptr)
        {
            _particles[i]->removeFromEngine();
            delete _particles[i];
        }
    }
    _particles.clear();
    removeKilledParticles();
}

void CParticleObject_base::updateParticlesPosition(float simulationTime)
{
    // We first want to remove particles that have timed out or that are not active anymore:
    int oldestIndex=-1;
    int firstFreeIndex=-1;
    int smallestUniqueID=_nextUniqueIDForParticle+2;
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=nullptr)
        {
            if ( _particles[i]->didTimeOut(simulationTime)||(_particles[i]->getInitializationState()==2) )
            { // We have to remove this one from _particles:
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // We cannot destroy this one now
                else
                    delete _particles[i]; // We can directly destroy this one here
                _particles[i]=nullptr;
                if (firstFreeIndex==-1)
                    firstFreeIndex=i;
            }
            else
            { // this one stays a bit more (or will be removed just after if the list is full))
                int uniqueID=_particles[i]->getUniqueID();
                if (smallestUniqueID>uniqueID)
                {
                    smallestUniqueID=uniqueID;
                    oldestIndex=i;
                }
            }
        }
        else
        {
            if (firstFreeIndex==-1)
                firstFreeIndex=i;
        }
    }

    // Update the other particles:
    for (size_t i=0;i<_particles.size();i++)
    {
        if (_particles[i]!=nullptr)
            _particles[i]->updatePosition();
    }
}
