#include "CollShapeDyn.h"
#include "RigidBodyContainerDyn.h"
#include "simLib.h"
#include "4X4FullMatrix.h"
#include "Vx/VxCollisionGeometry.h"
#include "Vx/VxCompositeCollisionGeometry.h"
#include "Vx/VxFrame.h"
#include "Vx/VxUniverse.h"
#include "Vx/VxMaterialTable.h"
#include "Vx/VxTransform.h"
#include "Vx/VxQuaternion.h"
#include "Vx/VxPlane.h"
#include "Vx/VxBox.h"
#include "Vx/VxSphere.h"
#include "Vx/VxCylinder.h"
#include "Vx/VxBoxHole.h"
#include "Vx/VxSphereHole.h"
#include "Vx/VxCylinderHole.h"
#include "Vx/VxCapsule.h"
#include "Vx/VxConvexMesh.h"
#include "Vx/VxHeightField.h"
#include "Vx/VxTriangleMeshBVTree.h"
#include "Vx/VxTriangleMeshUVGrid.h"
#include "Vx/VxBoundingBox.h"
#include "Vx/VxResponseModel.h"
#include "Vx/VxRigidBodyResponseModel.h"
#include "VortexConvertUtil.h"
#include <boost/lexical_cast.hpp>

// when adding the triangles, we want to make sure we can merge vertices shared by more than one triangle.
// because of float to double conversion, we better have a value > 0 here.
static const Vx::VxReal MergeEpsilon = 0.0;//0.001; see email from Martin on 29/1/2014

CCollShapeDyn::CCollShapeDyn()
{
}

CCollShapeDyn::~CCollShapeDyn()
{
    for (size_t i=0;i<_vortexGeoms.size();i++)
    {
        if (_vortexGeoms[i]->getPart() != nullptr)
        {
            _vortexGeoms[i]->getPart()->removeCollisionGeometry(_vortexGeoms[i]);
        }
        delete _vortexGeoms[i];
    }

    _vortexGeoms.clear();

    for (size_t i=0;i<_vortexMmeshVertices_scaled.size();i++)
        delete _vortexMmeshVertices_scaled[i];
    for (size_t i=0;i<_vortexMconvexPlanes_scaled.size();i++)
        delete _vortexMconvexPlanes_scaled[i];
    for (size_t i=0;i<_vortexMconvexPolygons.size();i++)
        delete _vortexMconvexPolygons[i];
}

void CCollShapeDyn::init(CXShape* shape,CXGeomProxy* geomData,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled)
{ // This is the init of a collision shape wrapper. The wrapper can contain several collision objects grouped in a compound
    CCollShapeDyn_base::init(shape,geomData,willBeStatic,inverseLocalInertiaFrame_scaled);
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);

    Vx::VxUniverse* universe=CRigidBodyContainerDyn::getDynWorld()->getWorld();

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float floatParams[36];
    int intParams[8];
    _simGetVortexParameters(shape,3,floatParams,intParams);

//    const double frictionCoeff_primary_linearAxis=getVortexUnsignedDouble(floatParams[0]);
//    const double frictionCoeff_secondary_linearAxis=getVortexUnsignedDouble(floatParams[1]);
//    const double frictionCoeff_primary_angularAxis=getVortexUnsignedDouble(floatParams[2]);
//    const double frictionCoeff_secondary_angularAxis=getVortexUnsignedDouble(floatParams[3]);
//    const double frictionCoeff_normal_angularAxis=getVortexUnsignedDouble(floatParams[4]);
//    const double staticFrictionScale_primary_linearAxis=getVortexUnsignedDouble(floatParams[5]);
//    const double staticFrictionScale_secondary_linearAxis=getVortexUnsignedDouble(floatParams[6]);
//    const double staticFrictionScale_primary_angularAxis=getVortexUnsignedDouble(floatParams[7]);
//    const double staticFrictionScale_secondary_angularAxis=getVortexUnsignedDouble(floatParams[8]);
//    const double staticFrictionScale_normal_angularAxis=getVortexUnsignedDouble(floatParams[9]);
//    const double compliance=getVortexUnsignedDouble(floatParams[10]);
//    const double damping=getVortexUnsignedDouble(floatParams[11]);
//    const double restitution=getVortexUnsignedDouble(floatParams[12]);
//    const double restitutionThreshold=getVortexUnsignedDouble(floatParams[13]);
//    const double adhesiveForce=getVortexUnsignedDouble(floatParams[14]);
//    const double linearVelocityDamping=getVortexUnsignedDouble(floatParams[15]);
//    const double angularVelocityDamping=getVortexUnsignedDouble(floatParams[16]);
//    const double slide_primary_linearAxis=getVortexUnsignedDouble(floatParams[17]);
//    const double slide_secondary_linearAxis=getVortexUnsignedDouble(floatParams[18]);
//    const double slide_primary_angularAxis=getVortexUnsignedDouble(floatParams[19]);
//    const double slide_secondary_angularAxis=getVortexUnsignedDouble(floatParams[20]);
//    const double slide_normal_angularAxis=getVortexUnsignedDouble(floatParams[21]);
//    const double slip_primary_linearAxis=getVortexUnsignedDouble(floatParams[22]);
//    const double slip_secondary_linearAxis=getVortexUnsignedDouble(floatParams[23]);
//    const double slip_primary_angularAxis=getVortexUnsignedDouble(floatParams[24]);
//    const double slip_secondary_angularAxis=getVortexUnsignedDouble(floatParams[25]);
//    const double slip_normal_angularAxis=getVortexUnsignedDouble(floatParams[26]);
//    const double autoSleep_linear_speed_threshold=getVortexUnsignedDouble(floatParams[27]);
//    const double autoSleep_linear_accel_threshold=getVortexUnsignedDouble(floatParams[28]);
//    const double autoSleep_angular_speed_threshold=getVortexUnsignedDouble(floatParams[29]);
//    const double autoSleep_angular_accel_threshold=getVortexUnsignedDouble(floatParams[30]);
//    const double skinThickness=getVortexUnsignedDouble(floatParams[31]);
//    const double autoAngularDampingTensionRatio=getVortexUnsignedDouble(floatParams[32]);
//    const double primaryLinearAxisVectorX=floatParams[33];
//    const double primaryLinearAxisVectorY=floatParams[34];
//    const double primaryLinearAxisVectorZ=floatParams[35];

//    Vx::VxMaterial::FrictionModel frictModels[5];
//    for (int i=0;i<5;i++)
//    {
//        switch (intParams[i])
//        {
//            case 0:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelBox;
//                break;
//            case 1:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelScaledBox;
//                break;
//            case 2:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelBoxProportionalLow;
//                break;
//            case 3:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelBoxProportionalHigh;
//                break;
//            case 4:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelScaledBoxFast;
//                break;
//            case 5:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelNeutral;
//                break;
//            default:
//                frictModels[i]=Vx::VxMaterial::kFrictionModelNone;
//                break;
//        }
//    }
    const bool treatPureShapesAsConvexShapes=((intParams[5]&1)!=0);
    const bool treatConvexShapesAsRandomShapes=((intParams[5]&2)!=0);
    const bool treatRandomShapesAsTerrain=((intParams[5]&4)!=0);
//    const bool fastMoving=((intParams[5]&8)!=0);
//    const bool autoSlip=((intParams[5]&16)!=0);
//    const bool autoAngularDamping=((intParams[5]&256)!=0);

//    const int autoSleepStepLiveThreshold=intParams[6];
//    const int materialUniqueId=intParams[7];


    //    const double autoAngularDampingTensionRatio=getVortexUnsignedDouble(floatParams[32]);

    // Do we have a pure primitive?
    int primType=_simGetPurePrimitiveType(geomInfo);
    bool isPrimitive=(primType!=sim_primitiveshape_none);
    bool isActuallyPrimitive=isPrimitive;
    if (treatPureShapesAsConvexShapes&&(primType!=sim_primitiveshape_heightfield))
        isPrimitive=false;
    if (isPrimitive)
    { // We have a pure primitive here. It is either a simple collision objects, or a compound of simple collision objects.
        // A simple collision object can be: sphere, cuboid, cylinder (previous 3 also with hollow parts (negative collision object), or heightfield
        if (!_simIsGeomWrapGeometric(geomInfo))
        { // We a have a pure MULTISHAPE!! Here we have a compound of simple collision objects
            int componentListSize=_simGetGeometricCount(geomInfo);
            CXGeometric** componentList=new CXGeometric*[componentListSize];
            _simGetAllGeometrics(geomInfo,(simVoid**)componentList);

            Vx::VxCompositeCollisionGeometry* vortexCompositeCollisionGeom = new Vx::VxCompositeCollisionGeometry();

            for (int i=0;i<componentListSize;i++)
            { // we go through all individual collision object components and add them to a compound collision object:
                CXGeometric* sc=componentList[i];
                int pType=_simGetPurePrimitiveType(sc);
                float hollowScaling=_simGetPureHollowScaling(sc); // this represents a value between 0.0 and 0.9999. When multiplying the shape dimensions with that value, we obtain the negative shape dimensions (if 0.0, there is no negative shape).
                C3Vector s;
                _simGetPurePrimitiveSizes(sc,s.data);

                int loopCnt=1;

                if (hollowScaling!=0.0f)
                    loopCnt=2;

                for (int j=0;j<loopCnt;j++)
                {
                    if (j>0)
                        hollowScaling=0.0f;

                    Vx::VxGeometry* vortexGeom = _createVortexSimpleGeometry(pType, s,hollowScaling, nullptr, 1.0);

                    C7Vector aax;
                    _simGetVerticesLocalFrame(sc,aax.X.data,aax.Q.data); // for pure shapes, the vertice frame also indicates the pure shape origin
                    C7Vector xxx(inverseLocalInertiaFrame_scaled*aax);

                    Vx::VxTransform tm;
                    Vx::VxQuaternion quat(xxx.Q(0), xxx.Q(1), xxx.Q(2), xxx.Q(3));
                    tm.makeRotation(quat);
                    tm.setTranslation(C3Vector2VxVector3(xxx.X));

                    Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeom,tm,floatParams,intParams);
                    vortexCompositeCollisionGeom->addCollisionGeometry(vortexCollisionGeom);
                }
            }
            delete[] componentList;
            _vortexGeoms.push_back(vortexCompositeCollisionGeom);
        }
        else
        { // we have a SIMPLE pure shape. This is a single simple collision object (e.g. sphere, cuboid, cylinder or heightfield)
            C3Vector s;
            _simGetPurePrimitiveSizes(geomInfo,s.data);
            Vx::VxCompositeCollisionGeometry* vortexCompositeCollisionGeom=nullptr;

            float hollowScaling=_simGetPureHollowScaling((CXGeometric*)geomInfo); // this represents a value between 0.0 and 0.9999. When multiplying the shape dimensions with that value, we obtain the negative shape dimensions (if 0.0, there is no negative shape).
            int loopCnt=1;
            if (hollowScaling!=0.0f)
            {
                loopCnt=2;
                vortexCompositeCollisionGeom=new Vx::VxCompositeCollisionGeometry();
            }

            for (int j=0;j<loopCnt;j++)
            {
                if (j>0)
                    hollowScaling=0.0f;
                Vx::VxGeometry* vortexGeom=_createVortexSimpleGeometry(primType, s, hollowScaling, geomInfo, 1.0);

                C7Vector aax;
                aax.setIdentity();
                if (primType!=sim_primitiveshape_heightfield) // that condition was forgotten and corrected on 16/1/2013
                    _simGetVerticesLocalFrame(geomInfo,aax.X.data,aax.Q.data);  // for pure shapes (except for heightfields!!), the vertice frame also indicates the pure shape origin.
                C7Vector xxx(inverseLocalInertiaFrame_scaled*aax);

                Vx::VxTransform tm;
                Vx::VxQuaternion quat(xxx.Q(0), xxx.Q(1), xxx.Q(2), xxx.Q(3));
                tm.makeRotation(quat);
                tm.setTranslation(C3Vector2VxVector3(xxx.X));

                Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeom,tm,floatParams,intParams);

                if (loopCnt==2)
                    vortexCompositeCollisionGeom->addCollisionGeometry(vortexCollisionGeom);
                else
                    _vortexGeoms.push_back(vortexCollisionGeom);
            }
            if (loopCnt==2)
                _vortexGeoms.push_back(vortexCompositeCollisionGeom);
        }
    }
    else
    { // Here we have either:
        // 1. a random shape/multishape (i.e. a simple random mesh)
        // 2. a convex shape (i.e. a simple convex mesh)
        // 3. a convex multishape (i.e. a compound of simple convex meshes)
        if (_simIsGeomWrapConvex(geomInfo)==0)
        {     // We have a general-type geom object (trimesh)
            if (!_simIsGeomWrapGeometric(geomInfo))
            { // We a have a grouping of random meshes.
                int componentListSize=_simGetGeometricCount(geomInfo);
                CXGeometric** componentList=new CXGeometric*[componentListSize];
                _simGetAllGeometrics(geomInfo,(simVoid**)componentList);
                for (int comp=0;comp<componentListSize;comp++)
                {
                    CXGeometric* sc=componentList[comp];

                    float* allVertices;
                    int allVerticesSize;
                    int* allIndices;
                    int allIndicesSize;
                    _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                    Vx::VxGeometry* vortexGeometry = nullptr;

                    if (!treatRandomShapesAsTerrain)
                    { // we want a random mesh here!

                        Vx::VxReal3* vortexVertices_scaled = new Vx::VxReal3[allVerticesSize/3];

                        for (int i=0;i<allVerticesSize/3;i++)
                        { // We need to scale the vertices
                            C3Vector v(allVertices+3*i+0);
                            vortexVertices_scaled[i][0]=v(0);
                            vortexVertices_scaled[i][1]=v(1);
                            vortexVertices_scaled[i][2]=v(2);
                        }
                        simReleaseBuffer((simChar*)allVertices);

                        Vx::VxTriangleMeshBVTree* bvTreeMesh = new Vx::VxTriangleMeshBVTree(allIndicesSize/3);
                        for (int i=0;i<allIndicesSize/3;i++)
                            bvTreeMesh->addMergedTriangleByVertexCopy(vortexVertices_scaled[allIndices[3*i+0]],vortexVertices_scaled[allIndices[3*i+1]],vortexVertices_scaled[allIndices[3*i+2]],MergeEpsilon);
                        vortexGeometry = bvTreeMesh;

                        delete[] vortexVertices_scaled;
                        simReleaseBuffer((simChar*)allIndices);
                    }
                    else
                    { // we want a terrain here!
                        vortexGeometry = _createVortexUVGridMesh(allVertices,allVerticesSize,allIndices,allIndicesSize,1.0);
                        //vortexGeometry = _createVortexBVTreeMesh(allVertices,allVerticesSize,allIndices,allIndicesSize,linScaling);
                        simReleaseBuffer((simChar*)allIndices);
                        simReleaseBuffer((simChar*)allVertices);
                    }

                    Vx::VxTransform tm;
                    Vx::VxQuaternion quat(inverseLocalInertiaFrame_scaled.Q(0), inverseLocalInertiaFrame_scaled.Q(1), inverseLocalInertiaFrame_scaled.Q(2), inverseLocalInertiaFrame_scaled.Q(3));
                    tm.makeRotation(quat);
                    tm.setTranslation(C3Vector2VxVector3(inverseLocalInertiaFrame_scaled.X));

                    Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeometry,tm,floatParams,intParams);

                    _vortexGeoms.push_back(vortexCollisionGeom);
                }
                delete[] componentList;
            }
            else
            {    // Single random mesh
                float* allVertices;
                int allVerticesSize;
                int* allIndices;
                int allIndicesSize;
                _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                Vx::VxGeometry* vortexGeometry = nullptr;

                if (!treatRandomShapesAsTerrain)
                { // we want a random mesh here
                    Vx::VxTriangleMeshBVTree* bvTreeMesh = new Vx::VxTriangleMeshBVTree(allIndicesSize/3);
                    vortexGeometry=bvTreeMesh;
                    for (int i=0;i<allIndicesSize/3;i++)
                    {
                        int ind[3]={allIndices[3*i+0],allIndices[3*i+1],allIndices[3*i+2]};
                        C3Vector v0(allVertices+3*ind[0]);
                        C3Vector v1(allVertices+3*ind[1]);
                        C3Vector v2(allVertices+3*ind[2]);
                        bvTreeMesh->addMergedTriangleByVertexCopy(  C3Vector2VxVector3(v0),
                                                                C3Vector2VxVector3(v1),
                                                                C3Vector2VxVector3(v2),
                                                                MergeEpsilon);
                        /* Vx::VxInfo(0, "Btri%d %g %g %g, %g %g %g, %g %g %g\n", i,
                            C3Vector2VxVector3(v0)[0],
                            C3Vector2VxVector3(v0)[1],
                            C3Vector2VxVector3(v0)[2],
                            C3Vector2VxVector3(v1)[0],
                            C3Vector2VxVector3(v1)[1],
                            C3Vector2VxVector3(v1)[2],
                            C3Vector2VxVector3(v2)[0],
                            C3Vector2VxVector3(v2)[1],
                            C3Vector2VxVector3(v2)[2]); */
                    }
                    simReleaseBuffer((simChar*)allVertices);
                    simReleaseBuffer((simChar*)allIndices);
                }
                else
                { // we want a terrain here!
                    vortexGeometry = _createVortexUVGridMesh(allVertices,allVerticesSize,allIndices,allIndicesSize,1.0);
                    //vortexGeometry = _createVortexBVTreeMesh(allVertices,allVerticesSize,allIndices,allIndicesSize,linScaling);
                    simReleaseBuffer((simChar*)allIndices);
                    simReleaseBuffer((simChar*)allVertices);
                }

                Vx::VxTransform tm;
                Vx::VxQuaternion quat(inverseLocalInertiaFrame_scaled.Q(0), inverseLocalInertiaFrame_scaled.Q(1), inverseLocalInertiaFrame_scaled.Q(2), inverseLocalInertiaFrame_scaled.Q(3));
                tm.makeRotation(quat);
                tm.setTranslation(C3Vector2VxVector3(inverseLocalInertiaFrame_scaled.X));

                Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeometry,tm,floatParams,intParams);

                _vortexGeoms.push_back(vortexCollisionGeom);
            }
        }
        else
        { // We have a convex shape or multishape:
            if (!_simIsGeomWrapGeometric(geomInfo))
            { // We a have a convex MULTISHAPE!! This is a compound of convex meshes
                int componentListSize=_simGetGeometricCount(geomInfo);
                CXGeometric** componentList=new CXGeometric*[componentListSize];
                _simGetAllGeometrics(geomInfo,(simVoid**)componentList);
                for (int comp=0;comp<componentListSize;comp++)
                {
                    CXGeometric* sc=componentList[comp];

                    float* allVertices;
                    int allVerticesSize;
                    int* allIndices;
                    int allIndicesSize;
                    _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);

                    Vx::VxReal3* vortexVertices_scaled = new Vx::VxReal3[allVerticesSize/3];

                    for (int i=0;i<allVerticesSize/3;i++)
                    { // We need to scale the vertices
                        C3Vector v(allVertices+3*i+0);
                        vortexVertices_scaled[i][0]=v(0);
                        vortexVertices_scaled[i][1]=v(1);
                        vortexVertices_scaled[i][2]=v(2);
                    }
                    simReleaseBuffer((simChar*)allVertices);

                    Vx::VxGeometry* vortexGeometry = nullptr;

                    if ((!treatConvexShapesAsRandomShapes)||isActuallyPrimitive)
                        vortexGeometry = new Vx::VxConvexMesh(vortexVertices_scaled,allVerticesSize/3);
                    else // USE VxTriangleMeshBVTree
                    {
                        Vx::VxTriangleMeshBVTree* bvTreeMesh = new Vx::VxTriangleMeshBVTree(allIndicesSize/3);
                        for (unsigned int i=0;i<allIndicesSize/3;i++)
                        {
                            bvTreeMesh->addMergedTriangleByVertexCopy(  vortexVertices_scaled[allIndices[3*i+0]],
                                                                        vortexVertices_scaled[allIndices[3*i+1]],
                                                                        vortexVertices_scaled[allIndices[3*i+2]],
                                                                        MergeEpsilon);
                            /* Vx::VxInfo(0, "Ctri%d %g %g %g, %g %g %g, %g %g %g\n", i,
                                    vortexVertices_scaled[allIndices[3*i+0]][0],
                                    vortexVertices_scaled[allIndices[3*i+0]][1],
                                    vortexVertices_scaled[allIndices[3*i+0]][2],
                                    vortexVertices_scaled[allIndices[3*i+1]][0],
                                    vortexVertices_scaled[allIndices[3*i+1]][1],
                                    vortexVertices_scaled[allIndices[3*i+1]][2],
                                    vortexVertices_scaled[allIndices[3*i+2]][0],
                                    vortexVertices_scaled[allIndices[3*i+2]][1],
                                    vortexVertices_scaled[allIndices[3*i+2]][2]);*/
                        }
                        vortexGeometry = bvTreeMesh;
                    }

                    delete[] vortexVertices_scaled;
                    simReleaseBuffer((simChar*)allIndices);

                    Vx::VxTransform tm;
                    Vx::VxQuaternion quat(inverseLocalInertiaFrame_scaled.Q(0), inverseLocalInertiaFrame_scaled.Q(1), inverseLocalInertiaFrame_scaled.Q(2), inverseLocalInertiaFrame_scaled.Q(3));
                    tm.makeRotation(quat);
                    tm.setTranslation(C3Vector2VxVector3(inverseLocalInertiaFrame_scaled.X));

                    Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeometry,tm,floatParams,intParams);

                    _vortexGeoms.push_back(vortexCollisionGeom);

                }
                delete[] componentList;
            }
            else
            { // We have a convex SHAPE. This is a single convex mesh
                float* allVertices;
                int allVerticesSize;
                int* allIndices;
                int allIndicesSize;
                _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);

                Vx::VxReal3* vortexVertices_scaled = new Vx::VxReal3[allVerticesSize/3];

                for (int i=0;i<allVerticesSize/3;i++)
                { // We need to scale the vertices
                    C3Vector v(allVertices+3*i+0);
                    vortexVertices_scaled[i][0]=v(0);
                    vortexVertices_scaled[i][1]=v(1);
                    vortexVertices_scaled[i][2]=v(2);
                }
                simReleaseBuffer((simChar*)allVertices);

                Vx::VxGeometry* vortexGeometry = nullptr;

                if ((!treatConvexShapesAsRandomShapes)||isActuallyPrimitive)
                    vortexGeometry = new Vx::VxConvexMesh(vortexVertices_scaled, allVerticesSize/3);
                else // USE VxTriangleMeshBVTree
                {
                    Vx::VxTriangleMeshBVTree* bvTreeMesh = new Vx::VxTriangleMeshBVTree(allIndicesSize/3);
                    for (unsigned int i=0;i<allIndicesSize/3;i++)
                    {
                        bvTreeMesh->addMergedTriangleByVertexCopy(  vortexVertices_scaled[allIndices[3*i+0]],
                                                                    vortexVertices_scaled[allIndices[3*i+1]],
                                                                    vortexVertices_scaled[allIndices[3*i+2]],
                                                                    MergeEpsilon);
                        /*Vx::VxInfo(0, "Dtri%d %g %g %g, %g %g %g, %g %g %g\n", i,
                                vortexVertices_scaled[allIndices[3*i+0]][0],
                                vortexVertices_scaled[allIndices[3*i+0]][1],
                                vortexVertices_scaled[allIndices[3*i+0]][2],
                                vortexVertices_scaled[allIndices[3*i+1]][0],
                                vortexVertices_scaled[allIndices[3*i+1]][1],
                                vortexVertices_scaled[allIndices[3*i+1]][2],
                                vortexVertices_scaled[allIndices[3*i+2]][0],
                                vortexVertices_scaled[allIndices[3*i+2]][1],
                                vortexVertices_scaled[allIndices[3*i+2]][2]);*/
                    }
                    vortexGeometry = bvTreeMesh;
                }

                delete[] vortexVertices_scaled;
                simReleaseBuffer((simChar*)allIndices);

                Vx::VxTransform tm;
                Vx::VxQuaternion quat(inverseLocalInertiaFrame_scaled.Q(0), inverseLocalInertiaFrame_scaled.Q(1), inverseLocalInertiaFrame_scaled.Q(2), inverseLocalInertiaFrame_scaled.Q(3));
                tm.makeRotation(quat);
                tm.setTranslation(C3Vector2VxVector3(inverseLocalInertiaFrame_scaled.X));

                Vx::VxCollisionGeometry* vortexCollisionGeom=_createVortexCollisionGeometry(universe,geomInfo,vortexGeometry,tm,floatParams,intParams);

                _vortexGeoms.push_back(vortexCollisionGeom);
            }
        }
    }
}

Vx::VxGeometry* CCollShapeDyn::_createVortexSimpleGeometry(int pType, const C3Vector& ins,float hollowScaling, CXGeomWrap* geomInfo, float linScaling)
{
    Vx::VxGeometry* vortexGeom = nullptr;

    C3Vector s = ins * linScaling; // ********** SCALING

    switch (pType)
    {
    case sim_primitiveshape_plane:
    case sim_primitiveshape_cuboid:
        if (s(2)<0.0001f)
            s(2)=0.0001f;
        if (hollowScaling==0.0f)
            vortexGeom = new Vx::VxBox(C3Vector2VxVector3(s));
        else
            vortexGeom = new Vx::VxBoxHole(C3Vector2VxVector3(s*hollowScaling));
        break;
    case sim_primitiveshape_cone:
        _simMakeDynamicAnnouncement(sim_announce_pureconenotsupported);
    case sim_primitiveshape_disc:
    case sim_primitiveshape_cylinder:
        if (s(2)<0.0001f)
            s(2)=0.0001f;
        if (hollowScaling==0.0f)
            vortexGeom = new Vx::VxCylinder((Vx::VxReal)(s(0)*0.5f),(Vx::VxReal)s(2));
        else
        {
            vortexGeom = new Vx::VxCylinderHole((Vx::VxReal)(s(0)*0.5f*hollowScaling),(Vx::VxReal)s(2));
            ((Vx::VxCylinderHole*)vortexGeom)->setFaceRemoved(true,true);
        }
        break;
    case sim_primitiveshape_spheroid:
        // Here we have a spheroid (or sphere)
        if ( ( ((s(0)-s(1))/s(0))>0.01f )||( ((s(0)-s(2))/s(0))>0.01f ) ) // Pure spheroids are not (yet) supported by Vortex
            _simMakeDynamicAnnouncement(sim_announce_purespheroidnotsupported);
        if (hollowScaling==0.0f)
            vortexGeom = new Vx::VxSphere((s(0)+s(1)+s(2))/6.0f);
        else
            vortexGeom = new Vx::VxSphereHole((s(0)+s(1)+s(2))*hollowScaling/6.0f);
        break;
    case sim_primitiveshape_capsule:
        {
            float r=(s(0)+s(1))/4.0f;
            vortexGeom = new Vx::VxCapsule((Vx::VxReal)r,(Vx::VxReal)(s(2)-r*2.0f));
        }
        break;
    case sim_primitiveshape_heightfield:
        {
            int xCnt,yCnt;
            float minH,maxH;
            const float* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH);
            Vx::VxArray<Vx::VxReal> _vortexHeightfieldData_scaled;
            float vShift=-(minH+maxH)/2.0f;
            for (int i=0;i<yCnt;i++)
            {
                for (int j=0;j<xCnt;j++)
                    _vortexHeightfieldData_scaled.push_back((Vx::VxReal)(hData[i*xCnt+j]+vShift)*linScaling); // ********** SCALING
            }

            const Vx::VxReal sizeCellX = s(0)/(xCnt-1), sizeCellY = s(1)/(yCnt-1), originX = -s(0)/2.0f, originY = -s(1)/2.0f;
            vortexGeom = new Vx::VxHeightField(xCnt-1, yCnt-1, sizeCellX, sizeCellY, originX, originY, _vortexHeightfieldData_scaled);
            static_cast<Vx::VxHeightField*>(vortexGeom)->setUpdateAdjacentTriangles(true);
        }
        break;
        default:
            //_simMakeDynamicAnnouncement(ANNOUNCE_GEOMETRY_TYPE_NOT_SUPPORTED);
        break;
    }

    return vortexGeom;
}

Vx::VxTriangleMeshUVGrid* CCollShapeDyn::_createVortexUVGridMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling)
{
    Vx::VxTriangleMeshUVGrid* uvGridMesh = new Vx::VxTriangleMeshUVGrid();

    Vx::VxReal3* vortexVertices_scaled = new Vx::VxReal3[allVerticesSize/3];
    Vx::VxReal3Ptr* verticesPtr = new Vx::VxReal3Ptr[allIndicesSize];

    /*
    *  The VxTriangleMeshUVGrid is a database where the triangles are organized into cells in a UV plane projection of the mesh.
    *  This UV plane are usually fit to the largest extension of the mesh for best efficiency. A Bounding box will be used for this
    *  Then the number of sell subdivision should be done so that there are not too many tringle in a cell.
    */
    C3Vector minS(SIM_MAX_FLOAT,SIM_MAX_FLOAT,SIM_MAX_FLOAT);
    C3Vector maxS(-SIM_MAX_FLOAT,-SIM_MAX_FLOAT,-SIM_MAX_FLOAT);

    for (int i=0;i<allVerticesSize/3;i++)
    { // We need to scale the vertices
        C3Vector v(allVertices+3*i);
        v*=linScaling; // ********** SCALING
        vortexVertices_scaled[i][0]=v(0);
        vortexVertices_scaled[i][1]=v(1);
        vortexVertices_scaled[i][2]=v(2);
        minS.keepMin(v);
        maxS.keepMax(v);
    }
    // note that here we would be better fitting an OBB instead, this will do for now:
    Vx::VxBoundingBox bbox((double)minS(0),(double)minS(1),(double)minS(2),(double)maxS(0),(double)maxS(1),(double)maxS(2));


    for (unsigned int i=0;i<allIndicesSize;i++)
    {
        verticesPtr[i] = vortexVertices_scaled[allIndices[i]];
    }

    int triCount = allIndicesSize/3;
    uvGridMesh->setExternalVertices(vortexVertices_scaled, verticesPtr, allVerticesSize, triCount);
    uvGridMesh->takeVertexOwnership(); // so that we don't have to delete the vortexVertices_scaled, verticesPtr.

    Vx::VxVector3 s = bbox.getBBSize();

    int minsize = s[0]>s[1] ? (s[0]>s[2] ? 0 : 2) : (s[1]>s[2] ? 1 : 2);
    int u = (minsize+1)%3;
    int v = (minsize+2)%3;

    int nu=1, nv=1;
    int tripercell = 4;
    do
    {
        // let say we want an average of tpercell triangle per sell
        // nu=cells along u, nv=cells long v: (1) nu*nv = cell count ~ triCount / tripercell = T
        // s is the bbox size. we want square cell as much as possible: (2) s[u]/nu = s[v]/nv = a
        // we then have by substitution nv = sqrt(T sv / su) etc.
        nv = int(VxSqrt(Vx::VxReal(triCount) / Vx::VxReal(tripercell) * s[v] / s[u]));
        nu = int(s[u]/s[v] * Vx::VxReal(nv));

        if (nu > 1 && nv > 1)
        {
            break;
        }
        else
        {
            // not enough cells, tri reducing tripercell
            tripercell /= 2;
            nu = nv = 1;
        }
    }
    while (tripercell > 1);
    // stop at tripercell == 1. The dbase is not realy justified: not enough triangles in there.
    // Could return nullptr and do a BVTree instead.

    int maxDepth = 0; // this would allow to create hierarchical terrain if a cell containts too many triangles
    uvGridMesh->createTerrain(u, v, nu, nv, maxDepth, 0, 0);

    //Vx::VxInfo(0, "creating VxTriangleMeshUVGrid with %d per %d cells.\n", nu, nv );

    return uvGridMesh;
}

Vx::VxTriangleMeshBVTree* CCollShapeDyn::_createVortexBVTreeMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling)
{
    Vx::VxTriangleMeshBVTree* bvTreeMesh = new Vx::VxTriangleMeshBVTree(allIndicesSize/3);

    Vx::VxReal3* vortexVertices_scaled = new Vx::VxReal3[allVerticesSize/3];

    for (int i=0;i<allVerticesSize/3;i++)
    { // We need to scale the vertices
        C3Vector v(allVertices+3*i);
        v*=linScaling; // ********** SCALING
        vortexVertices_scaled[i][0]=v(0);
        vortexVertices_scaled[i][1]=v(1);
        vortexVertices_scaled[i][2]=v(2);
    }


    for (unsigned int i=0;i<allIndicesSize/3;i++)
    {
        /*Vx::VxInfo(0, "Atri%d %g %g %g, %g %g %g, %g %g %g\n", i,
                vortexVertices_scaled[allIndices[3*i+0]][0],
                vortexVertices_scaled[allIndices[3*i+0]][1],
                vortexVertices_scaled[allIndices[3*i+0]][2],
                vortexVertices_scaled[allIndices[3*i+1]][0],
                vortexVertices_scaled[allIndices[3*i+1]][1],
                vortexVertices_scaled[allIndices[3*i+1]][2],
                vortexVertices_scaled[allIndices[3*i+2]][0],
                vortexVertices_scaled[allIndices[3*i+2]][1],
                vortexVertices_scaled[allIndices[3*i+2]][2]);*/
        bvTreeMesh->addMergedTriangleByVertexCopy(  vortexVertices_scaled[allIndices[3*i+0]],
                                                    vortexVertices_scaled[allIndices[3*i+1]],
                                                    vortexVertices_scaled[allIndices[3*i+2]],
                                                    MergeEpsilon);
    }
    delete[] vortexVertices_scaled;

    return bvTreeMesh;
}

Vx::VxCollisionGeometry* CCollShapeDyn::_createVortexCollisionGeometry(Vx::VxUniverse* universe,CXGeomWrap* geomInfo,Vx::VxGeometry* vxGeometry,const Vx::VxTransform& vxTransform,const float* floatParams,const int* intParams)
{
    const double frictionCoeff_primary_linearAxis=getVortexUnsignedDouble(floatParams[0]);
    const double frictionCoeff_secondary_linearAxis=getVortexUnsignedDouble(floatParams[1]);
    const double frictionCoeff_primary_angularAxis=getVortexUnsignedDouble(floatParams[2]);
    const double frictionCoeff_secondary_angularAxis=getVortexUnsignedDouble(floatParams[3]);
    const double frictionCoeff_normal_angularAxis=getVortexUnsignedDouble(floatParams[4]);
    const double staticFrictionScale_primary_linearAxis=getVortexUnsignedDouble(floatParams[5]);
    const double staticFrictionScale_secondary_linearAxis=getVortexUnsignedDouble(floatParams[6]);
    const double staticFrictionScale_primary_angularAxis=getVortexUnsignedDouble(floatParams[7]);
    const double staticFrictionScale_secondary_angularAxis=getVortexUnsignedDouble(floatParams[8]);
    const double staticFrictionScale_normal_angularAxis=getVortexUnsignedDouble(floatParams[9]);
    const double compliance=getVortexUnsignedDouble(floatParams[10]);
    const double damping=getVortexUnsignedDouble(floatParams[11]);
    const double restitution=getVortexUnsignedDouble(floatParams[12]);
    const double restitutionThreshold=getVortexUnsignedDouble(floatParams[13]);
    const double adhesiveForce=getVortexUnsignedDouble(floatParams[14]);
//    const double linearVelocityDamping=getVortexUnsignedDouble(floatParams[15]);
//    const double angularVelocityDamping=getVortexUnsignedDouble(floatParams[16]);
    const double slide_primary_linearAxis=getVortexUnsignedDouble(floatParams[17]);
    const double slide_secondary_linearAxis=getVortexUnsignedDouble(floatParams[18]);
    const double slide_primary_angularAxis=getVortexUnsignedDouble(floatParams[19]);
    const double slide_secondary_angularAxis=getVortexUnsignedDouble(floatParams[20]);
    const double slide_normal_angularAxis=getVortexUnsignedDouble(floatParams[21]);
    const double slip_primary_linearAxis=getVortexUnsignedDouble(floatParams[22]);
    const double slip_secondary_linearAxis=getVortexUnsignedDouble(floatParams[23]);
    const double slip_primary_angularAxis=getVortexUnsignedDouble(floatParams[24]);
    const double slip_secondary_angularAxis=getVortexUnsignedDouble(floatParams[25]);
    const double slip_normal_angularAxis=getVortexUnsignedDouble(floatParams[26]);
//    const double autoSleep_linear_speed_threshold=getVortexUnsignedDouble(floatParams[27]);
//    const double autoSleep_linear_accel_threshold=getVortexUnsignedDouble(floatParams[28]);
//    const double autoSleep_angular_speed_threshold=getVortexUnsignedDouble(floatParams[29]);
//    const double autoSleep_angular_accel_threshold=getVortexUnsignedDouble(floatParams[30]);
//    const double skinThickness=getVortexUnsignedDouble(floatParams[31]);
//    const double autoAngularDampingTensionRatio=getVortexUnsignedDouble(floatParams[32]);

    Vx::VxMaterial::FrictionModel frictModels[5];
    for (int i=0;i<5;i++)
    {
        switch (intParams[i])
        {
            case 0:
                frictModels[i]=Vx::VxMaterial::kFrictionModelBox;
                break;
            case 1:
                frictModels[i]=Vx::VxMaterial::kFrictionModelScaledBox;
                break;
            case 2:
                frictModels[i]=Vx::VxMaterial::kFrictionModelBoxProportionalLow;
                break;
            case 3:
                frictModels[i]=Vx::VxMaterial::kFrictionModelBoxProportionalHigh;
                break;
            case 4:
                frictModels[i]=Vx::VxMaterial::kFrictionModelScaledBoxFast;
                break;
            case 5:
                frictModels[i]=Vx::VxMaterial::kFrictionModelNeutral;
                break;
            default:
                frictModels[i]=Vx::VxMaterial::kFrictionModelNone;
                break;
        }
    }
//    const bool treatPureShapesAsConvexShapes=((intParams[5]&1)!=0);
//    const bool treatConvexShapesAsRandomShapes=((intParams[5]&2)!=0);
//    const bool treatRandomShapesAsTerrain=((intParams[5]&4)!=0);
//    const bool fastMoving=((intParams[5]&8)!=0);
//    const bool autoSlip=((intParams[5]&16)!=0);
//    const bool autoAngularDamping=((intParams[5]&256)!=0);
//    const int autoSleepStepLiveThreshold=intParams[6];
    const int materialUniqueId=intParams[7];
//*
    std::string materialString("CSIM");
    materialString+=boost::lexical_cast<std::string>(materialUniqueId);
    Vx::VxSmartPtr<Vx::VxRigidBodyResponseModel> responseModel=universe->getRigidBodyResponseModel();

    Vx::VxMaterial* material=responseModel->getMaterialTable()->getMaterial(materialString.c_str());

    if (material==nullptr)
    {
        material = responseModel->getMaterialTable()->registerMaterial(materialString.c_str());

        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinearPrimary,frictModels[0]);
        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinearSecondary,frictModels[1]);
        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisAngularPrimary,frictModels[2]);
        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisAngularSecondary,frictModels[3]);
        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisAngularNormal,frictModels[4]);
        // Friction scale. Vortex default: 1.0
        // When 2.0: Barrett hand sometimes can't close
        // When 1.5: Same as above, but less important. Boxes sometimes slightly slide on floor.
        // When 1.0: Boxes sometimes slightly slide on floor.
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisLinearPrimary,staticFrictionScale_primary_linearAxis);
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisLinearSecondary,staticFrictionScale_secondary_linearAxis);
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisAngularPrimary,staticFrictionScale_primary_angularAxis);
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisAngularSecondary,staticFrictionScale_secondary_angularAxis);
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisAngularNormal,staticFrictionScale_normal_angularAxis);

        // Adhesive force. Vortex default: 0.0
        material->setAdhesiveForce(adhesiveForce);

        // Compliance. Vortex default: 0.0
        // When 0.000005: Random meshes sink into pure shapes
        // When 0.0000005: Same as above, but not that strong. But stacked boxes slide and Barrett hand can't grasp
        // When 0.00000005: Stacked boxes slide. Barrett hand can't grasp. Grasping not good
        // When 0.000000005: Stacked boxes slide. object grasping doesn't work well. Hexapod jumps
        // When 0.0: object grasping makes the box jump away. Hexapod jumps/explodes
        material->setCompliance(compliance);

        // Damping. vortex default: 0.0
        material->setDamping(damping);

        // Restitution. Vortex default: 0.0
        material->setRestitution(restitution);

        // Restitution threshold. Vortex default:0.001
        // When 0.001: restitution of 0.5 makes everything jittery
        // When 0.5: restitution of 0.5 is still stable
        material->setRestitutionThreshold(restitutionThreshold);

        // Slide. Vortex default: 0.0
        material->setSlide(Vx::VxMaterial::kFrictionAxisLinearPrimary,slide_primary_linearAxis);
        material->setSlide(Vx::VxMaterial::kFrictionAxisLinearSecondary,slide_secondary_linearAxis);
        material->setSlide(Vx::VxMaterial::kFrictionAxisAngularPrimary,slide_primary_angularAxis);
        material->setSlide(Vx::VxMaterial::kFrictionAxisAngularSecondary,slide_secondary_angularAxis);
        material->setSlide(Vx::VxMaterial::kFrictionAxisAngularNormal,slide_normal_angularAxis);

        // friction coeff. Vortex default: 0.0
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisLinearPrimary,frictionCoeff_primary_linearAxis);
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisLinearSecondary,frictionCoeff_secondary_linearAxis);
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisAngularPrimary,frictionCoeff_primary_angularAxis);
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisAngularSecondary,frictionCoeff_secondary_angularAxis);
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisAngularNormal,frictionCoeff_normal_angularAxis);

        // Slip. Vortex default: 0.0
        // When 0.0: hexapod jumps
        // When 0.0001: hexapod doesn't jump (in combination with "compliance")
        material->setSlip(Vx::VxMaterial::kFrictionAxisLinearPrimary,slip_primary_linearAxis);
        material->setSlip(Vx::VxMaterial::kFrictionAxisLinearSecondary,slip_secondary_linearAxis);
        material->setSlip(Vx::VxMaterial::kFrictionAxisAngularPrimary,slip_primary_angularAxis);
        material->setSlip(Vx::VxMaterial::kFrictionAxisAngularSecondary,slip_secondary_angularAxis);
        material->setSlip(Vx::VxMaterial::kFrictionAxisAngularNormal,slip_normal_angularAxis);

        material->setIntegratedSlipDisplacement(Vx::VxMaterial::kIntegratedSlipDisplacementActivated);

//*/
/*
        Vx::VxMaterial* material = universe->getMaterial("default");
        material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinear,Vx::VxMaterial::kFrictionModelScaledBoxFast);//Fast);

        // Friction scale. Vortex default: 1.0
        // When 2.0: Barrett hand sometimes can't close
        // When 1.5: Same as above, but less important. Boxes sometimes slightly slide on floor.
        // When 1.0: Boxes sometimes slightly slide on floor.
        material->setStaticFrictionScale(Vx::VxMaterial::kFrictionAxisLinear,1.0);

        // Adhesive force. Vortex default: 0.0
        material->setAdhesiveForce(0.0);

        // Compliance. Vortex default: 0.0
        // When 0.000005: Random meshes sink into pure shapes
        // When 0.0000005: Same as above, but not that strong. But stacked boxes slide and Barrett hand can't grasp
        // When 0.00000005: Stacked boxes slide. Barrett hand can't grasp. Grasping not good
        // When 0.000000005: Stacked boxes slide. object grasping doesn't work well. Hexapod jumps
        // When 0.0: object grasping makes the box jump away. Hexapod jumps/explodes
        material->setCompliance(0.000005);

        // Damping. vortex default: 0.0
        material->setDamping(0.0);

        // Restitution. Vortex default: 0.0
        material->setRestitution(0.0); // (default is 0.0)

        // Restitution threshold. Vortex default: 0.001
        // When 0.001: restitution of 0.5 makes everything jittery
        // When 0.5: restitution of 0.5 is still stable
        material->setRestitutionThreshold(0.5);

        // Slide. Vortex default: 0.0
        material->setSlide(Vx::VxMaterial::kFrictionAxisLinear,0.0);

        // friction coeff. Vortex default: 0.0
        material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisLinear,0.5f);

        // Slip. Vortex default: 0.0
        // When 0.0: hexapod jumps
        // When 0.0001: hexapod doesn't jump (in combination with "compliance")
        material->setSlip(Vx::VxMaterial::kFrictionAxisLinear,0.0001);
*/
    }
    return(new Vx::VxCollisionGeometry(vxGeometry, material, vxTransform));
}

Vx::VxCollisionGeometry* CCollShapeDyn::getVortexGeoms(int index)
{ // return the individual items of a collision compound
    if (index>=int(_vortexGeoms.size()))
        return(nullptr);
    return(_vortexGeoms[index]);
}
