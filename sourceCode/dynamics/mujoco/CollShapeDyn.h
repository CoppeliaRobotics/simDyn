#pragma once

#include <CollShapeDyn_base.h>

class CCollShapeDyn : public CCollShapeDyn_base
{
public:
    CCollShapeDyn();
    virtual ~CCollShapeDyn();

    void init(CXShape* shape,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);
};
