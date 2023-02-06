#include "CollShapeDyn.h"
#include "RigidBodyContainerDyn.h"
#include <simLib/simLib.h>

CCollShapeDyn::CCollShapeDyn()
{
}

CCollShapeDyn::~CCollShapeDyn()
{
    for (size_t i=0;i<_odeGeoms.size();i++)
        dGeomDestroy(_odeGeoms[i]);
    _odeGeoms.clear();
    if (_trimeshDataID!=0)
        dGeomTriMeshDataDestroy(_trimeshDataID);
    if (_odeHeightfieldDataID!=0)
        dGeomHeightfieldDataDestroy(_odeHeightfieldDataID);
    delete[] _odeMeshLastTransformThingMatrix;

    for (size_t i=0;i<_odeMmeshVertices_scaled.size();i++)
        delete _odeMmeshVertices_scaled[i];
    for (size_t i=0;i<_odeMconvexPlanes_scaled.size();i++)
        delete _odeMconvexPlanes_scaled[i];
    for (size_t i=0;i<_odeMconvexPolygons.size();i++)
        delete _odeMconvexPolygons[i];
}

void CCollShapeDyn::init(CXShape* shape,CXGeomProxy* geomData,bool willBeStatic,const C7Vector& inverseLocalInertiaFrame_scaled)
{
    CCollShapeDyn_base::init(shape,geomData,willBeStatic,inverseLocalInertiaFrame_scaled);

    dSpaceID space=CRigidBodyContainerDyn::getDynWorld()->getOdeSpace();

    _trimeshDataID=0;
    _odeHeightfieldDataID=0;
    _odeMeshLastTransformThingMatrix=nullptr;
    _odeMeshLastTransformThingIndex=2;


    double linScaling=CRigidBodyContainerDyn::getDynWorld()->getPositionScalingFactorDyn();
    CXGeomWrap* geomInfo=(CXGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);
    // Do we have a pure primitive?
    int primType=_simGetPurePrimitiveType(geomInfo);
    if ( (primType!=sim_primitiveshape_none) )//&&(primType!=sim_primitiveshape_heightfield) ) // second part is temporary (ODE heightfields are buggy right now, so we use meshes instead)
    { // We have a pure primitive here:
        if (!_simIsGeomWrapGeometric(geomInfo))
        { // We a have a pure MULTISHAPE!!
            int componentListSize=_simGetGeometricCount(geomInfo);
            CXGeometric** componentList=new CXGeometric*[componentListSize];
            _simGetAllGeometrics(geomInfo,(void**)componentList);
            for (int i=0;i<componentListSize;i++)
            {
                CXGeometric* sc=componentList[i];
                int pType=_simGetPurePrimitiveType(sc);
                double hollowScaling=_simGetPureHollowScaling(sc);
                if (hollowScaling!=0.0)
                    _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);

                C3Vector s;
                _simGetPurePrimitiveSizes(sc,s.data);
                s*=linScaling; // ********** SCALING
                dGeomID odeGeom=0;
                dGeomID _odeGeom; // this will encapsulate odeGeom!
                if ( (pType==sim_primitiveshape_plane)||(pType==sim_primitiveshape_cuboid) )
                {
                    _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    double z=s(2);
                    if (z<0.0001)
                        z=0.0001;
                    odeGeom=dCreateBox(0,s(0),s(1),z);
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                }
                if ( (pType==sim_primitiveshape_disc)||(pType==sim_primitiveshape_cylinder) )
                {
                    _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    double z=s(2);
                    if (z<0.0001)
                        z=0.0001;
                    odeGeom=dCreateCylinder(0,s(0)*0.5,z);
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                }
                if (pType==sim_primitiveshape_cone)
                { // Pure cones are not (yet) supported by ODE
                    _simMakeDynamicAnnouncement(sim_announce_pureconenotsupported);
                    // We generate a cylinder instead:
                    _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    odeGeom=dCreateCylinder(0,s(0)*0.5,s(2));
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                }
                if (pType==sim_primitiveshape_spheroid)
                {
                    if ( ( ((s(0)-s(1))/s(0))>0.01 )||( ((s(0)-s(2))/s(0))>0.01 ) ) // Pure spheroids are not (yet) supported by ODE
                        _simMakeDynamicAnnouncement(sim_announce_purespheroidnotsupported);
                    _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    odeGeom=dCreateSphere(0,(s(0)+s(1)+s(2))/6.0); // in case we have a spheroid, we take the average sphere
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                }
                if (pType==sim_primitiveshape_capsule)
                {
                    _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    double r=(s(0)+s(1))/4.0;
                    odeGeom=dCreateCapsule(0,r,s(2)-r*2.0);
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                }
                C7Vector aax;
                _simGetVerticesLocalFrame(sc,aax.X.data,aax.Q.data); // for pure shapes, the vertice frame also indicates the pure shape origin
                aax.X*=linScaling; // ********** SCALING
                C7Vector xxx(inverseLocalInertiaFrame_scaled*aax);
                dGeomSetPosition(odeGeom,xxx.X(0),xxx.X(1),xxx.X(2));
                dQuaternion dQ;
                dQ[0]=xxx.Q.data[0];
                dQ[1]=xxx.Q.data[1];
                dQ[2]=xxx.Q.data[2];
                dQ[3]=xxx.Q.data[3];
                dGeomSetQuaternion(odeGeom,dQ);
                _odeGeoms.push_back(_odeGeom);
            }
            delete[] componentList;
        }
        else
        { // we have a SIMPLE pure shape
            double hollowScaling=_simGetPureHollowScaling((CXGeometric*)geomInfo);
            if (hollowScaling!=0.0)
                _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);
            C3Vector s;
            _simGetPurePrimitiveSizes(geomInfo,s.data);
            s*=linScaling; // ********** SCALING
            dGeomID odeGeom=0;
            dGeomID _odeGeom; // this will encapsulate odeGeom!
            C4Vector additionalRotation_forHeightfieldOnly;
            additionalRotation_forHeightfieldOnly.setIdentity();
            if ( (primType==sim_primitiveshape_plane)||(primType==sim_primitiveshape_cuboid) )
            {
                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                double z=s(2);
                if (z<0.0001)
                    z=0.0001;
                odeGeom=dCreateBox(0,s(0),s(1),z);
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            if ( (primType==sim_primitiveshape_disc)||(primType==sim_primitiveshape_cylinder) )
            {
                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                double z=s(2);
                if (z<0.0001)
                    z=0.0001;
                odeGeom=dCreateCylinder(0,s(0)*0.5,z);
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            if (primType==sim_primitiveshape_cone)
            { // Pure cones are not (yet) supported by ODE
                _simMakeDynamicAnnouncement(sim_announce_pureconenotsupported);
                // We generate a cylinder instead:
                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                odeGeom=dCreateCylinder(0,s(0)*0.5,s(2));
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            if (primType==sim_primitiveshape_spheroid)
            {
                if ( ( ((s(0)-s(1))/s(0))>0.01 )||( ((s(0)-s(2))/s(0))>0.01 ) ) // Pure spheroids are not (yet) supported by ODE
                    _simMakeDynamicAnnouncement(sim_announce_purespheroidnotsupported);
                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                odeGeom=dCreateSphere(0,(s(0)+s(1)+s(2))/6.0); // in case we have a spheroid, we take the average sphere
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            if (primType==sim_primitiveshape_capsule)
            {
                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                double r=(s(0)+s(1))/4.0;
                odeGeom=dCreateCapsule(0,r,s(2)-r*2.0);
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            if (primType==sim_primitiveshape_heightfield)
            {
                int xCnt,yCnt;
                double minH,maxH;
                const double* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH);
                _odeHeightfieldDataID=dGeomHeightfieldDataCreate();
                // Following corrects for the different diagonals between the Coppelia Simulator and ODE by internally rotating the HF by 90 degrees
                for (int j=0;j<xCnt;j++)
                {
                    for (int i=0;i<yCnt;i++)
                    {
                        double h=hData[i*xCnt+j]-(minH+(maxH-minH)*0.5); // Second part is because our hf mesh's origin (after the local transf) is in its center, and ODE needs data from bottom
                        _odeHeightfieldData_scaled.push_back(float(h*linScaling)); // ********** SCALING
                    }
                }
                dGeomHeightfieldDataBuildSingle(_odeHeightfieldDataID,_odeHeightfieldData_scaled.data(),0,s(1),s(0),yCnt,xCnt,1.0,0.0,(s(0)+s(1))*0.2,0);
                //      do not forget the offset if using this      dGeomHeightfieldDataSetBounds(_odeHeightfieldDataID,minH*linScaling,maxH*linScaling);
                odeGeom=dCreateHeightfield(0,_odeHeightfieldDataID,1);
                // Rotate so Z is up, not Y (which is the default orientation)
                additionalRotation_forHeightfieldOnly.setEulerAngles(1.57079632,0.0,0.0);
                C4Vector zRot;
                zRot.setEulerAngles(0.0,0.0,1.57079632);
                additionalRotation_forHeightfieldOnly=zRot*additionalRotation_forHeightfieldOnly;


                _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                dGeomTransformSetGeom(_odeGeom,odeGeom);
            }
            C7Vector aax;
            aax.setIdentity();
            if (primType!=sim_primitiveshape_heightfield) // that condition was forgotten and corrected on 16/1/2013
                _simGetVerticesLocalFrame(geomInfo,aax.X.data,aax.Q.data);  // for pure shapes (except for heightfields!!), the vertice frame also indicates the pure shape origin.
            aax.X*=linScaling; // ********** SCALING
            C7Vector xxx(inverseLocalInertiaFrame_scaled*aax);
            xxx.Q*=additionalRotation_forHeightfieldOnly;

            dGeomSetPosition(odeGeom,xxx.X(0),xxx.X(1),xxx.X(2));
            dQuaternion dQ;
            dQ[0]=xxx.Q.data[0];
            dQ[1]=xxx.Q.data[1];
            dQ[2]=xxx.Q.data[2];
            dQ[3]=xxx.Q.data[3];
            dGeomSetQuaternion(odeGeom,dQ);
            _odeGeoms.push_back(_odeGeom);
        }
    }
    else
    { // Here we have either:
        // 1. a random shape/multishape
        // 2. a convex shape
        // 3. a convex multishape
        if (_simIsGeomWrapConvex(geomInfo)==0)
        {     // We have a general-type geom object (trimesh) (or a heightfield that we treat as such since ODE heightfields are still buggy!!):
            double* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
            _meshIndices.assign(allIndices,allIndices+allIndicesSize);

            for (int i=0;i<allVerticesSize/3;i++)
            { // We need to take into account the position of the inertia frame
                C3Vector v(allVertices+3*i+0);
                v*=linScaling; // ********** SCALING
                v*=inverseLocalInertiaFrame_scaled;
                _meshVertices_scaled.push_back(v(0));
                _meshVertices_scaled.push_back(v(1));
                _meshVertices_scaled.push_back(v(2));
            }
            simReleaseBuffer((char*)allVertices);
            simReleaseBuffer((char*)allIndices);

            _trimeshDataID=dGeomTriMeshDataCreate();
            dGeomTriMeshDataBuildSingle(_trimeshDataID,&_meshVertices_scaled[0],3*sizeof(double),_meshVertices_scaled.size()/3,&_meshIndices[0],_meshIndices.size(),3*sizeof(int));

            dGeomID odeGeom=dCreateTriMesh(space,_trimeshDataID,nullptr,nullptr,nullptr);

            dGeomTriMeshEnableTC(odeGeom,dSphereClass,false); // disable caching to avoid strange effects
            dGeomTriMeshEnableTC(odeGeom,dBoxClass,false); // disable caching to avoid strange effects

            C7Vector xxx;
            xxx.setIdentity();
            dGeomSetPosition(odeGeom,xxx.X(0),xxx.X(1),xxx.X(2));
            dQuaternion dQ;
            dQ[0]=xxx.Q.data[0];
            dQ[1]=xxx.Q.data[1];
            dQ[2]=xxx.Q.data[2];
            dQ[3]=xxx.Q.data[3];
            dGeomSetQuaternion(odeGeom,dQ);
            _odeGeoms.push_back(odeGeom);
            _odeMeshLastTransformThingMatrix=new dReal[16*2];
        }
        else
        { // We have a convex shape or multishape:
            if (!_simIsGeomWrapGeometric(geomInfo))
            { // We a have a convex MULTISHAPE!!
                int componentListSize=_simGetGeometricCount(geomInfo);
                CXGeometric** componentList=new CXGeometric*[componentListSize];
                _simGetAllGeometrics(geomInfo,(void**)componentList);
                for (int comp=0;comp<componentListSize;comp++)
                {
                    CXGeometric* sc=componentList[comp];

                    double* allVertices;
                    int allVerticesSize;
                    int* allIndices;
                    int allIndicesSize;
                    _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                    _meshVertices_scaled.clear();
                    _odeConvexPolygons.clear();
                    _odeConvexPlanes_scaled.clear();
                    _meshIndices.assign(allIndices,allIndices+allIndicesSize);

                    // We need to find a point inside of the shape, and shift the data to have that point at the origin, otherwise ODE complains:
                    C3Vector c;// this is the inside point
                    c.clear();
                    for (int i=0;i<allVerticesSize/3;i++)
                    {
                        C3Vector v(allVertices+3*i);
                        v*=linScaling; // ********** SCALING
                        c+=v;
                    }
                    c/=double(allVerticesSize/3);

                    C7Vector tr;
                    tr.X=c;
                    tr.Q.setIdentity();
                    C7Vector _inverseLocalInertiaFrame2_scaled(inverseLocalInertiaFrame_scaled*tr);

                    for (int i=0;i<allVerticesSize/3;i++)
                    { // We need to take into account the position of the inertia frame
                        C3Vector v(allVertices+3*i+0);
                        v*=linScaling; // ********** SCALING
                        v-=c; // we recenter the convex mesh (will be corrected further down)
                        v=_inverseLocalInertiaFrame2_scaled.Q*v; // the translational part is taken care below (we have to provide a centered shape (i.e. origin inside the convex) to ODE)
                        _meshVertices_scaled.push_back(v(0));
                        _meshVertices_scaled.push_back(v(1));
                        _meshVertices_scaled.push_back(v(2));
                    }
                    simReleaseBuffer((char*)allVertices);
                    simReleaseBuffer((char*)allIndices);

                    // TODO: identify triangular faces that belong to the same logical plane. Same for triangular polygons. That will speed-up slightly calculations, and increase stability!
                    for (size_t i=0;i<_meshIndices.size()/3;i++)
                    {
                        _odeConvexPolygons.push_back(3);
                        _odeConvexPolygons.push_back(_meshIndices[3*i+0]);
                        _odeConvexPolygons.push_back(_meshIndices[3*i+1]);
                        _odeConvexPolygons.push_back(_meshIndices[3*i+2]);

#ifdef dDOUBLE
                        C3Vector p0(_meshVertices_scaled[3*_meshIndices[3*i+0]+0],_meshVertices_scaled[3*_meshIndices[3*i+0]+1],_meshVertices_scaled[3*_meshIndices[3*i+0]+2]);
                        C3Vector p1(_meshVertices_scaled[3*_meshIndices[3*i+1]+0],_meshVertices_scaled[3*_meshIndices[3*i+1]+1],_meshVertices_scaled[3*_meshIndices[3*i+1]+2]);
                        C3Vector p2(_meshVertices_scaled[3*_meshIndices[3*i+2]+0],_meshVertices_scaled[3*_meshIndices[3*i+2]+1],_meshVertices_scaled[3*_meshIndices[3*i+2]+2]);
#else
                        C3Vector p0(&_meshVertices_scaled[3*_meshIndices[3*i+0]]);
                        C3Vector p1(&_meshVertices_scaled[3*_meshIndices[3*i+1]]);
                        C3Vector p2(&_meshVertices_scaled[3*_meshIndices[3*i+2]]);
#endif
                        C3Vector v0(p1-p0);
                        C3Vector v1(p2-p0);
                        C3Vector n(v0^v1);
                        n.normalize();
                        double d=p0*n;
                        _odeConvexPlanes_scaled.push_back(n(0));
                        _odeConvexPlanes_scaled.push_back(n(1));
                        _odeConvexPlanes_scaled.push_back(n(2));
                        _odeConvexPlanes_scaled.push_back(d);
                    }

                    std::vector<dReal>* a1=new std::vector<dReal>(_meshVertices_scaled);
                    std::vector<unsigned int>* a2=new std::vector<unsigned int>(_odeConvexPolygons);
                    std::vector<dReal>* a3=new std::vector<dReal>(_odeConvexPlanes_scaled);
                    _odeMmeshVertices_scaled.push_back(a1);
                    _odeMconvexPolygons.push_back(a2);
                    _odeMconvexPlanes_scaled.push_back(a3);
                    dGeomID odeGeom=dCreateConvex(0,&a3->at(0),a3->size()/4,&a1->at(0),a1->size()/3,&a2->at(0));

                    dGeomID _odeGeom=dCreateGeomTransform(space);
                    dGeomTransformSetCleanup(_odeGeom,1);
                    dGeomTransformSetGeom(_odeGeom,odeGeom);
                    // We need to take into account the position/orientation of the inertia frame (orientation is taken care above, since dGeomSetQuaternion doesn't seem to be working for convexs???)
                    dGeomSetPosition(odeGeom,_inverseLocalInertiaFrame2_scaled.X(0),_inverseLocalInertiaFrame2_scaled.X(1),_inverseLocalInertiaFrame2_scaled.X(2));
                    // dGeomSetQuaternion(odeGeom,_inverseLocalInertiaFrame2_scaled.Q.data); strangely, that doesn't work here, but works fine with other primitives
                    _odeGeoms.push_back(_odeGeom);
                }
                delete[] componentList;
            }
            else
            { // We have a convex SHAPE
                double* allVertices;
                int allVerticesSize;
                int* allIndices;
                int allIndicesSize;
                _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                _meshIndices.assign(allIndices,allIndices+allIndicesSize);

                // We need to find a point inside of the shape, and shift the data to have that point at the origin, otherwise ODE complains:
                C3Vector c;// this is the inside point
                c.clear();
                for (int i=0;i<allVerticesSize/3;i++)
                {
                    C3Vector v(allVertices+3*i);
                    v*=linScaling; // ********** SCALING
                    c+=v;
                }
                c/=double(allVerticesSize/3);

                C7Vector tr;
                tr.X=c;
                tr.Q.setIdentity();
                C7Vector _inverseLocalInertiaFrame2_scaled(inverseLocalInertiaFrame_scaled*tr);

                for (int i=0;i<allVerticesSize/3;i++)
                { // We need to take into account the position of the inertia frame
                    C3Vector v(allVertices+3*i+0);
                    v*=linScaling; // ********** SCALING
                    v-=c; // we recenter the convex mesh (will be corrected further down)
                    v=_inverseLocalInertiaFrame2_scaled.Q*v; // the translational part is taken care below (we have to provide a centered shape (i.e. origin inside the convex) to ODE)
                    _meshVertices_scaled.push_back(v(0));
                    _meshVertices_scaled.push_back(v(1));
                    _meshVertices_scaled.push_back(v(2));
                }

                simReleaseBuffer((char*)allVertices);
                simReleaseBuffer((char*)allIndices);

                // TODO: identify triangular faces that belong to the same logical plane. Same for triangular polygons. That will speed-up slightly calculations, and increase stability!
                for (size_t i=0;i<_meshIndices.size()/3;i++)
                {
                    _odeConvexPolygons.push_back(3);
                    _odeConvexPolygons.push_back(_meshIndices[3*i+0]);
                    _odeConvexPolygons.push_back(_meshIndices[3*i+1]);
                    _odeConvexPolygons.push_back(_meshIndices[3*i+2]);

#ifdef dDOUBLE
                    C3Vector p0(_meshVertices_scaled[3*_meshIndices[3*i+0]+0],_meshVertices_scaled[3*_meshIndices[3*i+0]+1],_meshVertices_scaled[3*_meshIndices[3*i+0]+2]);
                    C3Vector p1(_meshVertices_scaled[3*_meshIndices[3*i+1]+0],_meshVertices_scaled[3*_meshIndices[3*i+1]+1],_meshVertices_scaled[3*_meshIndices[3*i+1]+2]);
                    C3Vector p2(_meshVertices_scaled[3*_meshIndices[3*i+2]+0],_meshVertices_scaled[3*_meshIndices[3*i+2]+1],_meshVertices_scaled[3*_meshIndices[3*i+2]+2]);
#else
                    C3Vector p0(&_meshVertices_scaled[3*_meshIndices[3*i+0]]);
                    C3Vector p1(&_meshVertices_scaled[3*_meshIndices[3*i+1]]);
                    C3Vector p2(&_meshVertices_scaled[3*_meshIndices[3*i+2]]);
#endif
                    C3Vector v0(p1-p0);
                    C3Vector v1(p2-p0);
                    C3Vector n(v0^v1);
                    n.normalize();
                    double d=p0*n;
                    _odeConvexPlanes_scaled.push_back(n(0));
                    _odeConvexPlanes_scaled.push_back(n(1));
                    _odeConvexPlanes_scaled.push_back(n(2));
                    _odeConvexPlanes_scaled.push_back(d);
                }
                int convexPlanesSize=int(_meshIndices.size()/3);
                int ptsSize=allVerticesSize/3;

                dGeomID odeGeom=dCreateConvex(0,&_odeConvexPlanes_scaled[0],convexPlanesSize,&_meshVertices_scaled[0],ptsSize,&_odeConvexPolygons[0]);
                dGeomID _odeGeom=dCreateGeomTransform(space);
                dGeomTransformSetCleanup(_odeGeom,1);
                dGeomTransformSetGeom(_odeGeom,odeGeom);
                // We need to take into account the position/orientation of the inertia frame (orientation is taken care above, since dGeomSetQuaternion doesn't seem to be working for convexs???)
                dGeomSetPosition(odeGeom,_inverseLocalInertiaFrame2_scaled.X(0),_inverseLocalInertiaFrame2_scaled.X(1),_inverseLocalInertiaFrame2_scaled.X(2));
                // dGeomSetQuaternion(odeGeom,inverseLocalInertiaFrame_scaled.Q.data); strangely, that doesn't work here, but works fine with other primitives
                _odeGeoms.push_back(_odeGeom);
            }
        }
    }
}

dGeomID CCollShapeDyn::getOdeGeoms(int index)
{
    if (index>=int(_odeGeoms.size()))
        return(nullptr);
    return(_odeGeoms[index]);
}

void CCollShapeDyn::setOdeMeshLastTransform()
{
    if (_odeMeshLastTransformThingMatrix!=nullptr)
    {
        const dReal* pos=dGeomGetPosition(_odeGeoms[0]);
        const dReal* rot=dGeomGetRotation(_odeGeoms[0]);
        bool first=_odeMeshLastTransformThingIndex==2;
        if (first)
            _odeMeshLastTransformThingIndex=0;
        dReal* m=_odeMeshLastTransformThingMatrix+_odeMeshLastTransformThingIndex*16;

        m[0]=rot[0];
        m[1]=rot[1];
        m[2]=rot[2];
        m[3]=0.0;

        m[4]=rot[4];
        m[5]=rot[5];
        m[6]=rot[6];
        m[7]=0.0;

        m[8]=rot[8];
        m[9]=rot[9];
        m[10]=rot[10];
        m[11]=0.0;

        m[12]=pos[0];
        m[13]=pos[1];
        m[14]=pos[2];
        m[15]=1.0;

        _odeMeshLastTransformThingIndex=!_odeMeshLastTransformThingIndex;
        dReal* m2=_odeMeshLastTransformThingMatrix+_odeMeshLastTransformThingIndex*16;

        if (first)
        { // first time, we initialize both matrices:
            for (size_t i=0;i<16;i++)
                m2[i]=m[i];
        }
        dGeomTriMeshSetLastTransform(_odeGeoms[0],m2);
    }
}
