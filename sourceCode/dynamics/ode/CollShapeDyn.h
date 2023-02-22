#pragma once

#include "CollShapeDyn_base.h"
#include "ode/ode.h"

class CCollShapeDyn : public CCollShapeDyn_base
{
public:
    CCollShapeDyn();
    virtual ~CCollShapeDyn();

    void init(CXShape* shape,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);

    dGeomID getOdeGeoms(int index);
    void setOdeMeshLastTransform();

protected:
    std::vector<dGeomID> _odeGeoms; // if more than 1 element, then it is a compound object
    dTriMeshDataID _trimeshDataID;
    dHeightfieldDataID _odeHeightfieldDataID;
    dReal* _odeMeshLastTransformThingMatrix;
    unsigned char _odeMeshLastTransformThingIndex;

    // Following few need to remain valid throughout the lifetime of ODE CollShapeDyn:
    std::vector<double> _meshVertices_scaled;
    std::vector<int> _meshIndices;
    std::vector<float> _odeHeightfieldData_scaled;
    std::vector<dReal> _odeConvexPlanes_scaled;
    std::vector<unsigned int> _odeConvexPolygons;
    std::vector<std::vector<dReal>* > _odeMmeshVertices_scaled;
    std::vector<std::vector<dReal>* > _odeMconvexPlanes_scaled;
    std::vector<std::vector<unsigned int>* > _odeMconvexPolygons;
};
