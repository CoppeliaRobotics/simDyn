#pragma once

#include "dummyClasses.h"
#include <vector>
#include "7Vector.h"

class CCollShapeDyn_base
{
public:
    CCollShapeDyn_base();
    virtual ~CCollShapeDyn_base();

    virtual void init(CXShape* shape,CXGeomProxy* geomData,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);
};
