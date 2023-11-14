#pragma once

#include <utils.h>
#include <vector>
#include <simMath/7Vector.h>

class CCollShapeDyn_base
{
public:
    CCollShapeDyn_base();
    virtual ~CCollShapeDyn_base();

    virtual void init(CXShape* shape,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);
};
