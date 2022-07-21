#pragma once

#include "CollShapeDyn_base.h"
#include "btBulletDynamicsCommon.h"

class CCollShapeDyn : public CCollShapeDyn_base
{
public:
    CCollShapeDyn();
    virtual ~CCollShapeDyn();

    void init(CXShape* shape,CXGeomProxy* geomData,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled);

    btCollisionShape* getBtCollisionShape();

protected:    
    btTriangleIndexVertexArray* _indexVertexArrays; // for meshes
    std::vector<btCollisionShape*> _compoundChildShapes;
    btCollisionShape* _collisionShape;

    // Following few maybe need to remain valid throughout the lifetime of Bullet CollShapeDyn:
    std::vector<dynReal> _meshVertices_scaled;
    std::vector<int> _meshIndices;
};
