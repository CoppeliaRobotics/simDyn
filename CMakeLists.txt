cmake_minimum_required(VERSION 3.18)
project(simExtDyn)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.5.0.0 REQUIRED)

if(WIN32)
    # bullet-2-83 fails to compile with:
    # error LNK2038: mismatch detected for 'RuntimeLibrary': value
    #                'MT_StaticRelease' doesn't match value 'MD_DynamicRelease'
    if(POLICY CMP0091)
        cmake_policy(SET CMP0091 NEW)
    endif()
endif()

set(Bullet_2_78_default ON)
set(Bullet_2_83_default ON)
set(ODE_default ON)
set(Newton_default ON)
set(Mujoco_default ON)
if(APPLE)
    set(Vortex_default OFF)
else()
    set(Vortex_default ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
        # using clang with clang-cl front end
    #elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    else()
        # using clang with regular front end
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -Wno-narrowing")
        # following flags cause instability of Bullet-2.78 plugin:
        #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=sse")
        #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
        #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -freciprocal-math")
        #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funsafe-math-optimizations")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # using GCC
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -Wno-narrowing")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # using Visual Studio C++
endif()

if(UNIX AND NOT APPLE)
    add_definitions(-D_POSIX_VER_64)
endif()

if(APPLE)
    add_definitions(-D_MACOSX_VER)
endif()

set(LIBCOPPELIASIM_SOURCE_DIR NOT-FOUND CACHE PATH "Location of coppeliaSimLib source tree")
find_path(LIBCOPPELIASIM_SOURCE_DIR
    sourceCode/interfaces/dummyClasses.h
    PATHS ${LIBCOPPELIASIM_SOURCE_DIR} ../coppeliaSimLib ../../coppeliaSimLib
    DOC "Location of coppeliaSimLib/sourceCode/interfaces"
    REQUIRED NO_DEFAULT_PATH
)

include_directories(sourceCode)
include_directories(sourceCode/dynamics)
include_directories(${LIBCOPPELIASIM_SOURCE_DIR}/sourceCode/interfaces)
include_directories(${COPPELIASIM_INCLUDE_DIR})
include_directories(${COPPELIASIM_INCLUDE_DIR}/stack)
include_directories(${COPPELIASIM_INCLUDE_DIR}/simMath)

set(COMMON_SOURCES
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/7Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3X3Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4X4Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MMatrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MyMath.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackArray.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackBool.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackMap.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackNull.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackNumber.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackObject.cpp
    ${COPPELIASIM_INCLUDE_DIR}/stack/stackString.cpp
    sourceCode/dynamics/CollShapeDyn_base.cpp
    sourceCode/dynamics/ConstraintDyn_base.cpp
    sourceCode/dynamics/ParticleObjectContainer_base.cpp
    sourceCode/dynamics/ParticleObject_base.cpp
    sourceCode/dynamics/ParticleDyn_base.cpp
    sourceCode/dynamics/RigidBodyDyn_base.cpp
    sourceCode/dynamics/RigidBodyContainerDyn_base.cpp
    sourceCode/simExtDyn.cpp
)

set(Bullet_2_78 ${Bullet_2_78_default} CACHE BOOL "")
if(Bullet_2_78)
coppeliasim_add_plugin(
    simExtBullet-2-78
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/bullet_2_78/CollShapeDyn.cpp
    sourceCode/dynamics/bullet_2_78/RigidBodyDyn.cpp
    sourceCode/dynamics/bullet_2_78/ConstraintDyn.cpp
    sourceCode/dynamics/bullet_2_78/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/bullet_2_78/ParticleDyn.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDispatcher.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvt.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btUnionFind.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btManifoldResult.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btGhostObject.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionObject.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btUniformScalingShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMesh.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleCallback.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleBuffer.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTetrahedronShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btSphereShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btShapeHull.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultiSphereShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btEmptyShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCylinderShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexHullShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvex2dShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConeShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConcaveShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCompoundShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCapsuleShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBoxShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBox2dShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_tri_collision.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_memory.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_contact.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_set.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btTriangleShapeEx.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactShape.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactBvh.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGenericPoolAllocator.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btContactProcessing.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btRigidBody.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btSerializer.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuickprof.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btGeometryUtil.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btConvexHull.cpp
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedAllocator.cpp
)
target_compile_definitions(simExtBullet-2-78 PRIVATE SIM_MATH_DOUBLE)
target_compile_definitions(simExtBullet-2-78 PRIVATE sReal=double)
target_compile_definitions(simExtBullet-2-78 PRIVATE BT_USE_DOUBLE_PRECISION)
target_compile_definitions(simExtBullet-2-78 PRIVATE INCLUDE_BULLET_2_78_CODE)
target_compile_definitions(simExtBullet-2-78 PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtBullet-2-78 PRIVATE LIBRARY_NAME=\"Bullet-2-78\")
target_compile_definitions(simExtBullet-2-78 PRIVATE ENGINE_NAME=\"Bullet\")
target_include_directories(simExtBullet-2-78 PRIVATE sourceCode/dynamics/bullet_2_78)
target_include_directories(simExtBullet-2-78 PRIVATE sourceCode/dynamics/bullet_2_78/bullet_2_78)
endif(Bullet_2_78)

set(Bullet_2_83 ${Bullet_2_83_default} CACHE BOOL "")
if(Bullet_2_83)
coppeliasim_add_plugin(
    simExtBullet-2-83
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/bullet_2_83/CollShapeDyn.cpp
    sourceCode/dynamics/bullet_2_83/RigidBodyDyn.cpp
    sourceCode/dynamics/bullet_2_83/ConstraintDyn.cpp
    sourceCode/dynamics/bullet_2_83/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/bullet_2_83/ParticleDyn.cpp
)
target_compile_definitions(simExtBullet-2-83 PRIVATE SIM_MATH_DOUBLE)
target_compile_definitions(simExtBullet-2-83 PRIVATE sReal=double)
target_compile_definitions(simExtBullet-2-83 PRIVATE BT_USE_DOUBLE_PRECISION)
target_compile_definitions(simExtBullet-2-83 PRIVATE INCLUDE_BULLET_2_83_CODE)
target_compile_definitions(simExtBullet-2-83 PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtBullet-2-83 PRIVATE LIBRARY_NAME=\"Bullet-2-83\")
target_compile_definitions(simExtBullet-2-83 PRIVATE ENGINE_NAME=\"Bullet\")
target_include_directories(simExtBullet-2-83 PRIVATE ${BULLET_INCLUDEPATH})
target_include_directories(simExtBullet-2-83 PRIVATE sourceCode/dynamics/bullet_2_83)
if(WIN32)
    target_link_libraries(simExtBullet-2-83
        ${BULLET_BUILD_DIR}/lib/Release/BulletDynamics.lib
        ${BULLET_BUILD_DIR}/lib/Release/BulletCollision.lib
        ${BULLET_BUILD_DIR}/lib/Release/LinearMath.lib
    )
    # see about error LNK2038 at the top of this file:
    set_property(TARGET simExtBullet-2-83 PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
elseif(UNIX)
    target_link_libraries(simExtBullet-2-83
        ${BULLET_BUILD_DIR}/src/BulletDynamics/libBulletDynamics.a
        ${BULLET_BUILD_DIR}/src/BulletCollision/libBulletCollision.a
        ${BULLET_BUILD_DIR}/src/LinearMath/libLinearMath.a
    )
endif()
endif(Bullet_2_83)

set(ODE ${ODE_default} CACHE BOOL "")
if(ODE)
coppeliasim_add_plugin(
    simExtODE
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/ode/CollShapeDyn.cpp
    sourceCode/dynamics/ode/RigidBodyDyn.cpp
    sourceCode/dynamics/ode/ConstraintDyn.cpp
    sourceCode/dynamics/ode/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/ode/ParticleDyn.cpp
    sourceCode/dynamics/ode/ode/src/util.cpp
    sourceCode/dynamics/ode/ode/src/timer.cpp
    sourceCode/dynamics/ode/ode/src/step.cpp
    sourceCode/dynamics/ode/ode/src/sphere.cpp
    sourceCode/dynamics/ode/ode/src/rotation.cpp
    sourceCode/dynamics/ode/ode/src/ray.cpp
    sourceCode/dynamics/ode/ode/src/quickstep.cpp
    sourceCode/dynamics/ode/ode/src/plane.cpp
    sourceCode/dynamics/ode/ode/src/odetls.cpp
    sourceCode/dynamics/ode/ode/src/odeou.cpp
    sourceCode/dynamics/ode/ode/src/odemath.cpp
    sourceCode/dynamics/ode/ode/src/odeinit.cpp
    sourceCode/dynamics/ode/ode/src/ode.cpp
    sourceCode/dynamics/ode/ode/src/obstack.cpp
    sourceCode/dynamics/ode/ode/src/misc.cpp
    sourceCode/dynamics/ode/ode/src/memory.cpp
    sourceCode/dynamics/ode/ode/src/matrix.cpp
    sourceCode/dynamics/ode/ode/src/mat.cpp
    sourceCode/dynamics/ode/ode/src/mass.cpp
    sourceCode/dynamics/ode/ode/src/lcp.cpp
    sourceCode/dynamics/ode/ode/src/heightfield.cpp
    sourceCode/dynamics/ode/ode/src/fastltsolve.c
    sourceCode/dynamics/ode/ode/src/fastlsolve.c
    sourceCode/dynamics/ode/ode/src/fastldlt.c
    sourceCode/dynamics/ode/ode/src/fastdot.c
    sourceCode/dynamics/ode/ode/src/export-dif.cpp
    sourceCode/dynamics/ode/ode/src/error.cpp
    sourceCode/dynamics/ode/ode/src/cylinder.cpp
    sourceCode/dynamics/ode/ode/src/convex.cpp
    sourceCode/dynamics/ode/ode/src/collision_util.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh_new.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_sphere.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ray.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_plane.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_opcode.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_gimpact.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_distance.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_disabled.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ccylinder.cpp
    sourceCode/dynamics/ode/ode/src/collision_trimesh_box.cpp
    sourceCode/dynamics/ode/ode/src/collision_transform.cpp
    sourceCode/dynamics/ode/ode/src/collision_space.cpp
    sourceCode/dynamics/ode/ode/src/collision_sapspace.cpp
    sourceCode/dynamics/ode/ode/src/collision_quadtreespace.cpp
    sourceCode/dynamics/ode/ode/src/collision_kernel.cpp
    sourceCode/dynamics/ode/ode/src/collision_cylinder_trimesh.cpp
    sourceCode/dynamics/ode/ode/src/collision_cylinder_sphere.cpp
    sourceCode/dynamics/ode/ode/src/collision_cylinder_plane.cpp
    sourceCode/dynamics/ode/ode/src/collision_cylinder_box.cpp
    sourceCode/dynamics/ode/ode/src/capsule.cpp
    sourceCode/dynamics/ode/ode/src/box.cpp
    sourceCode/dynamics/ode/ode/src/array.cpp
    sourceCode/dynamics/ode/ode/src/joints/universal.cpp
    sourceCode/dynamics/ode/ode/src/joints/slider.cpp
    sourceCode/dynamics/ode/ode/src/joints/pu.cpp
    sourceCode/dynamics/ode/ode/src/joints/pr.cpp
    sourceCode/dynamics/ode/ode/src/joints/plane2d.cpp
    sourceCode/dynamics/ode/ode/src/joints/piston.cpp
    sourceCode/dynamics/ode/ode/src/joints/null.cpp
    sourceCode/dynamics/ode/ode/src/joints/lmotor.cpp
    sourceCode/dynamics/ode/ode/src/joints/joint.cpp
    sourceCode/dynamics/ode/ode/src/joints/hinge2.cpp
    sourceCode/dynamics/ode/ode/src/joints/hinge.cpp
    sourceCode/dynamics/ode/ode/src/joints/fixed.cpp
    sourceCode/dynamics/ode/ode/src/joints/contact.cpp
    sourceCode/dynamics/ode/ode/src/joints/ball.cpp
    sourceCode/dynamics/ode/ode/src/joints/amotor.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Opcode.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_VolumeCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeBuilders.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Picking.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OptimizedTree.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OBBCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Model.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_MeshInterface.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_HybridModel.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Common.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Collider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BaseModel.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBTree.cpp
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBCollider.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceUtils.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriangle.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceSegment.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRevisitedRadix.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRay.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRandom.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePoint.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePlane.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceOBB.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix4x4.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix3x3.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceIndexedTriangle.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceHPoint.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceContainer.cpp
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAABB.cpp
    sourceCode/dynamics/ode/ode/src/collision_libccd.cpp
    sourceCode/dynamics/ode/ode/src/nextafterf.c
    sourceCode/dynamics/ode/ode/libccd/src/alloc.c
    sourceCode/dynamics/ode/ode/libccd/src/ccd.c
    sourceCode/dynamics/ode/ode/libccd/src/mpr.c
    sourceCode/dynamics/ode/ode/libccd/src/polytope.c
    sourceCode/dynamics/ode/ode/libccd/src/support.c
    sourceCode/dynamics/ode/ode/libccd/src/vec3.c
)
target_compile_definitions(simExtODE PRIVATE SIM_MATH_DOUBLE)
target_compile_definitions(simExtODE PRIVATE sReal=double)
target_compile_definitions(simExtODE PRIVATE dDOUBLE)
target_compile_definitions(simExtODE PRIVATE CCD_DOUBLE)
target_compile_definitions(simExtODE PRIVATE INCLUDE_ODE_CODE)
target_compile_definitions(simExtODE PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtODE PRIVATE LIBRARY_NAME=\"ODE\")
target_compile_definitions(simExtODE PRIVATE ENGINE_NAME=\"ODE\")
target_compile_definitions(simExtODE PRIVATE dNODEBUG)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_ENABLED)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_CYL_CYL)
target_compile_definitions(simExtODE PRIVATE ODE_LIB)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_CONVEX_BOX)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_CONVEX_CYL)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_CONVEX_SPHERE)
target_compile_definitions(simExtODE PRIVATE dLIBCCD_CONVEX_CONVEX)
target_compile_definitions(simExtODE PRIVATE dTRIMESH_ENABLED)
target_compile_definitions(simExtODE PRIVATE dTRIMESH_OPCODE)
target_include_directories(simExtODE PRIVATE sourceCode/dynamics/ode)
target_include_directories(simExtODE PRIVATE sourceCode/dynamics/ode/ode)
target_include_directories(simExtODE PRIVATE sourceCode/dynamics/ode/ode/src)
target_include_directories(simExtODE PRIVATE sourceCode/dynamics/ode/ode/OPCODE)
target_include_directories(simExtODE PRIVATE sourceCode/dynamics/ode/ode/libccd/src)
endif(ODE)

set(Newton ${Newton_default} CACHE BOOL "")
if(Newton)
if(WIN32)
    set(NEWTON_PLATFORM_SPECIFIC_SOURCES
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2/pthread.c
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAMP.cpp
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpSolver.cpp
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpInstance.cpp
    )
endif()
coppeliasim_add_plugin(
    simExtNewton
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/newton/CollShapeDyn.cpp
    sourceCode/dynamics/newton/RigidBodyDyn.cpp
    sourceCode/dynamics/newton/ConstraintDyn.cpp
    sourceCode/dynamics/newton/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/newton/ParticleDyn.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/Newton.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonClass.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dg.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAABBPolygonSoup.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAsyncThread.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull3d.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull4d.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgCRC.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDebug.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDelaunayTetrahedralization.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralMatrix.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralVector.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGoogol.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgIntersections.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMatrix.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMemory.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMutexThread.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgNode.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgObb.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupBuilder.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedra.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedraMassProperties.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgQuaternion.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRandom.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRef.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRefCounter.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSmallDeterminant.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSPDMatrix.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThread.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadHive.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTree.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTypes.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBallConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBilateralConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBody.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBodyMasterList.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhase.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseAggregate.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseDefault.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhasePersistent.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollision.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBox.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBVH.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCapsule.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionChamferCylinder.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompound.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompoundFractured.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCone.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvex.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexHull.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexPolygon.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCylinder.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableClothPatch.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableMesh.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableSolidMesh.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionHeightField.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionInstance.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionMesh.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionNull.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionScene.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionSphere.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCapsule.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCylinder.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionUserMesh.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgContact.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCorkscrewConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBodiesUpdate.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBody.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableContact.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDynamicBody.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgHingeConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgKinematicBody.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgNarrowPhaseCollision.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSkeletonContainer.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSlidingConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUniversalConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUpVectorConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUserConstraint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorld.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsParallelSolver.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsSimpleSolver.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicUpdate.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect1.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect2.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect3.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect4.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect5.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect6.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dLinearAlgebra.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMathDefines.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMatrix.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dQuaternion.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dStdAfxMath.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dVector.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/Custom6DOF.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomAlloc.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomArcticulatedTransformManager.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomBallAndSocket.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomControllerManager.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomCorkScrew.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomDryRollingFriction.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomGear.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHinge.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHingeActuator.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomInputManager.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJoint.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJointLibraryStdAfx.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomKinematicController.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPathFollow.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPulley.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomRackAndPinion.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlider.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSliderActuator.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlidingContact.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomTriggerManager.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversal.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversalActuator.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUpVector.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUserBlank.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dBaseHierarchy.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dClassInfo.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersAlloc.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersStdAfx.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dCRC.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRefCounter.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dString.cpp
    sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dTree.cpp
    ${NEWTON_PLATFORM_SPECIFIC_SOURCES}
)
#target_compile_definitions(simExtNewton PRIVATE USE_THREAD_EMULATION) # not recomended. Use only if you need to handle Newton contacts in a contact callback script
#target_compile_definitions(simExtNewton PRIVATE SIM_MATH_DOUBLE)
#target_compile_definitions(simExtNewton PRIVATE _NEWTON_USE_DOUBLE)
target_compile_definitions(simExtNewton PRIVATE SIM_INTERFACE_SINGLE)
target_compile_definitions(simExtNewton PRIVATE sReal=float)
target_compile_definitions(simExtNewton PRIVATE INCLUDE_NEWTON_CODE)
target_compile_definitions(simExtNewton PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtNewton PRIVATE LIBRARY_NAME=\"Newton\")
target_compile_definitions(simExtNewton PRIVATE ENGINE_NAME=\"Newton\")
target_compile_definitions(simExtNewton PRIVATE _CUSTOM_JOINTS_STATIC_LIB)
target_compile_definitions(simExtNewton PRIVATE _NEWTON_STATIC_LIB)
target_compile_definitions(simExtNewton PRIVATE PTW32_STATIC_LIB)
target_compile_definitions(simExtNewton PRIVATE _ASSERTE=void)
if(USE_THREAD_EMULATION)
    target_compile_definitions(simExtNewton PRIVATE DG_USE_THREAD_EMULATION)
endif()
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints)
target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers)
if(WIN32)
    target_include_directories(simExtNewton PRIVATE sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2)
endif()
endif(Newton)

set(Vortex ${Vortex_default} CACHE BOOL "")
if(Vortex)
coppeliasim_add_plugin(
    simExtVortex
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/vortex/CollShapeDyn.cpp
    sourceCode/dynamics/vortex/RigidBodyDyn.cpp
    sourceCode/dynamics/vortex/ConstraintDyn.cpp
    sourceCode/dynamics/vortex/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/vortex/ParticleDyn.cpp
)
target_compile_definitions(simExtVortex PRIVATE SIM_MATH_DOUBLE)
target_compile_definitions(simExtVortex PRIVATE sReal=double)
target_compile_definitions(simExtVortex PRIVATE INCLUDE_VORTEX_CODE)
target_compile_definitions(simExtVortex PRIVATE VX_DLL)
target_compile_definitions(simExtVortex PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtVortex PRIVATE LIBRARY_NAME=\"Vortex\")
target_compile_definitions(simExtVortex PRIVATE ENGINE_NAME=\"Vortex\")
target_compile_definitions(simExtVortex PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
if(WIN32)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/VxCore.lib)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/VxFoundation.lib)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/VxPlatform.lib)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/VxMath.lib)
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/libVxCore.so)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/libVxPlatform.so)
    target_link_libraries(simExtVortex ${VORTEX_LIBPATH}/libVxMath.so)
endif()
target_include_directories(simExtVortex PRIVATE ${VORTEX_INCLUDEPATH})
target_include_directories(simExtVortex PRIVATE sourceCode/dynamics/vortex)
endif(Vortex)

set(Mujoco ${Mujoco_default} CACHE BOOL "")
if(Mujoco)
coppeliasim_add_plugin(
    simExtMujoco
    LEGACY
    SOURCES
    ${COMMON_SOURCES}
    sourceCode/dynamics/mujoco/CollShapeDyn.cpp
    sourceCode/dynamics/mujoco/RigidBodyDyn.cpp
    sourceCode/dynamics/mujoco/ConstraintDyn.cpp
    sourceCode/dynamics/mujoco/RigidBodyContainerDyn.cpp
    sourceCode/dynamics/mujoco/ParticleDyn.cpp
    sourceCode/dynamics/mujoco/xmlser.cpp
    sourceCode/dynamics/mujoco/tinyxml2.cpp
)
target_compile_definitions(simExtMujoco PRIVATE SIM_MATH_DOUBLE)
target_compile_definitions(simExtMujoco PRIVATE sReal=double)
target_compile_definitions(simExtMujoco PRIVATE INCLUDE_MUJOCO_CODE)
target_compile_definitions(simExtMujoco PRIVATE DYNAMICS_PLUGIN_VERSION=20)
target_compile_definitions(simExtMujoco PRIVATE LIBRARY_NAME=\"Mujoco\")
target_compile_definitions(simExtMujoco PRIVATE ENGINE_NAME=\"Mujoco\")
target_include_directories(simExtMujoco PRIVATE ${MUJOCO_INCLUDEPATH})
target_include_directories(simExtMujoco PRIVATE sourceCode/dynamics/mujoco)
if(WIN32)
    target_link_libraries(simExtMujoco ${MUJOCO_LIBPATH}/mujoco.lib)
elseif(UNIX AND APPLE)
    target_link_libraries(simExtMujoco ${MUJOCO_LIBPATH}/libmujoco.dylib)
elseif(UNIX)
    target_link_libraries(simExtMujoco ${MUJOCO_LIBPATH}/mujoco_nogl.a)
endif()
coppeliasim_add_lua(simMujoco.lua)
endif(Mujoco)

