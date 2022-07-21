#pragma once

#include "CollShapeDyn_base.h"
#include "Newton.h"
#include "dMatrix.h"
#include "CustomJoint.h"
#include "CustomHinge.h"
#include "CustomSlider.h"
#include "CustomBallAndSocket.h"
#include "CustomHingeActuator.h"
#include "CustomSliderActuator.h"

class CCollShapeDyn : public CCollShapeDyn_base
{
public:
    CCollShapeDyn();
    virtual ~CCollShapeDyn();

    void init(CXShape* shape,CXGeomProxy* geomData,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);

    NewtonCollision* getNewtonCollision();

protected:    
    void _setNewtonParameters(CXShape* shape);
    NewtonCollision* _shape;
    std::vector<float> _newtonHeightfieldData;

    // Following few maybe need to remain valid throughout the lifetime of Newton CollShapeDyn:
    std::vector<dynReal> _meshVertices_scaled;
    std::vector<int> _meshIndices;
};
