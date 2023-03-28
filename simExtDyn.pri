TEMPLATE = lib
QT -= core
QT -= gui
DEFINES -= UNICODE
CONFIG += shared plugin

*-msvc* {
    QMAKE_CXXFLAGS += /std:c++17
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}

*-g++* {
    CONFIG += c++17
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings
    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing
    QMAKE_CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0 # requirement for recent Vortex versions
    QMAKE_CXXFLAGS += -fpermissive

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

*-clang* {
    CONFIG += c++17
    QMAKE_CXXFLAGS += -Wno-narrowing
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.13
}

win32 {
    DEFINES += NOMINMAX
    DEFINES += WIN_SIM
    LIBS += -lwinmm
    LIBS += -luser32
}

macx {
    DEFINES += _MACOSX_VER
    DEFINES += MAC_SIM
}

unix:!macx {
    contains(QMAKE_HOST.arch, x86_64):{ # 64 Bit
        DEFINES += _POSIX_VER_64
    }
    DEFINES += LIN_SIM
}

INCLUDEPATH += $$BOOST_INCLUDEPATH
INCLUDEPATH += "sourceCode"
INCLUDEPATH += "sourceCode/dynamics"
INCLUDEPATH += "../../coppeliaSimLib/sourceCode/interfaces"
INCLUDEPATH += "../include"

BULLET_2_78_ENGINE {
    TARGET = simExtBullet-2-78
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += BT_USE_DOUBLE_PRECISION
    DEFINES += INCLUDE_BULLET_2_78_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Bullet-2-78\\\"
    DEFINES += ENGINE_NAME=\\\"Bullet\\\"

    INCLUDEPATH += "sourceCode/dynamics/bullet_2_78"
    INCLUDEPATH += "sourceCode/dynamics/bullet_2_78/bullet_2_78"
}

BULLET_2_83_ENGINE {
    TARGET = simExtBullet-2-83
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += BT_USE_DOUBLE_PRECISION
    DEFINES += INCLUDE_BULLET_2_83_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Bullet-2-83\\\"
    DEFINES += ENGINE_NAME=\\\"Bullet\\\"
    *-msvc* {
        QMAKE_CFLAGS_RELEASE += -MT
        QMAKE_CXXFLAGS_RELEASE += -MT
    }
    win32 {
        LIBS += $${BULLET_BUILD_DIR}/lib/Release/BulletDynamics.lib
        LIBS += $${BULLET_BUILD_DIR}/lib/Release/BulletCollision.lib
        LIBS += $${BULLET_BUILD_DIR}/lib/Release/LinearMath.lib
    }
    unix {
        LIBS += $${BULLET_BUILD_DIR}/src/BulletDynamics/libBulletDynamics.a
        LIBS += $${BULLET_BUILD_DIR}/src/BulletCollision/libBulletCollision.a
        LIBS += $${BULLET_BUILD_DIR}/src/LinearMath/libLinearMath.a
    }

    INCLUDEPATH += $$BULLET_INCLUDEPATH
    INCLUDEPATH += "sourceCode/dynamics/bullet_2_83"
}

ODE_ENGINE {
    TARGET = simExtODE
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += dDOUBLE
    DEFINES += CCD_DOUBLE
    DEFINES += INCLUDE_ODE_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"ODE\\\"
    DEFINES += ENGINE_NAME=\\\"ODE\\\"
    DEFINES += dNODEBUG
    DEFINES += dLIBCCD_ENABLED
    DEFINES += dLIBCCD_CYL_CYL
    DEFINES += ODE_LIB
    DEFINES += dLIBCCD_CONVEX_BOX
    DEFINES += dLIBCCD_CONVEX_CYL
    DEFINES += dLIBCCD_CONVEX_SPHERE
    DEFINES += dLIBCCD_CONVEX_CONVEX
    DEFINES += dTRIMESH_ENABLED
    DEFINES += dTRIMESH_OPCODE

    INCLUDEPATH += "sourceCode/dynamics/ode"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/src"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/OPCODE"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/libccd/src"
}

NEWTON_ENGINE {
    TARGET = simExtNewton
    #something is broken with Newton and double precision... revert to single precision for now
    #DEFINES += SIM_MATH_DOUBLE
    #DEFINES += _NEWTON_USE_DOUBLE
    DEFINES += SIM_INTERFACE_SINGLE
    DEFINES += sReal=float
    #DEFINES += DG_USE_THREAD_EMULATION # not recomended. Use only if you need to handle Newton contacts in a contact callback script
    DEFINES += INCLUDE_NEWTON_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Newton\\\"
    DEFINES += ENGINE_NAME=\\\"Newton\\\"
    DEFINES += _CUSTOM_JOINTS_STATIC_LIB
    DEFINES += _NEWTON_STATIC_LIB
    DEFINES += PTW32_STATIC_LIB
    DEFINES += _ASSERTE\\\(x\\\) #for _ASSERTE(x)
    !win32 {
        QMAKE_CXXFLAGS += -msse2 -msse3 -g -msse -msse2 -msse3 -msse4 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant
    }
    *-msvc* {
        DEFINES += _CRT_SECURE_NO_WARNINGS
    }

    INCLUDEPATH += "sourceCode/dynamics/newton"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers"
    win32 {
        INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2"
    }
}

VORTEX_ENGINE {
    TARGET = simExtVortex
    CONFIG += c++11
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += INCLUDE_VORTEX_CODE
    DEFINES += VX_DLL
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Vortex\\\"
    DEFINES += ENGINE_NAME=\\\"Vortex\\\"
    win32 {
        LIBS += "$${VORTEX_LIBPATH}/VxCore.lib"
        LIBS += "$${VORTEX_LIBPATH}/VxFoundation.lib"
        LIBS += "$${VORTEX_LIBPATH}/VxPlatform.lib"
        LIBS += "$${VORTEX_LIBPATH}/VxMath.lib"
    }
    unix:!macx {
        LIBS += "$${VORTEX_LIBPATH}/libVxCore.so"
        LIBS += "$${VORTEX_LIBPATH}/libVxPlatform.so"
        LIBS += "$${VORTEX_LIBPATH}/libVxMath.so"
    }

    INCLUDEPATH += $$VORTEX_INCLUDEPATH
    INCLUDEPATH += "sourceCode/dynamics/vortex"
}

MUJOCO_ENGINE {
    TARGET = simExtMujoco
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += INCLUDE_MUJOCO_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Mujoco\\\"
    DEFINES += ENGINE_NAME=\\\"Mujoco\\\"
    *-msvc* {
        QMAKE_CFLAGS_RELEASE += -MT
        QMAKE_CXXFLAGS_RELEASE += -MT
        LIBS += $${MUJOCO_LIBPATH}/mujoco.lib
    }
    unix {
        LIBS += $${MUJOCO_LIBPATH}/libmujoco.so
    }

    INCLUDEPATH += $$MUJOCO_INCLUDEPATH
    INCLUDEPATH += "sourceCode/dynamics/mujoco"
}

PHYSX_ENGINE {
    TARGET = simExtPhysx
    DEFINES += SIM_MATH_DOUBLE
    DEFINES += sReal=double
    DEFINES += INCLUDE_PHYSX_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=22
    DEFINES += LIBRARY_NAME=\\\"Physx\\\"
    DEFINES += ENGINE_NAME=\\\"Physx\\\"
    *-msvc* {
        QMAKE_CFLAGS_RELEASE += -MT
        QMAKE_CXXFLAGS_RELEASE += -MT
        LIBS += $${PHYSX_LIBPATH}/PhysX_64.lib
        LIBS += $${PHYSX_LIBPATH}/PhysXFoundation_64.lib
        LIBS += $${PHYSX_LIBPATH}/PhysXCommon_64.lib
        LIBS += $${PHYSX_LIBPATH}/PhysXExtensions_static_64.lib
        LIBS += $${PHYSX_LIBPATH}/PhysXCooking_64.lib
        LIBS += $${PHYSX_LIBPATH}/PhysXPvdSDK_static_64.lib
    }
    unix {
        LIBS += $${PHYSX_LIBPATH}/PhysX.so
        LIBS += $${PHYSX_LIBPATH}/PhysXFoundation.so
        LIBS += $${PHYSX_LIBPATH}/PhysXCommon.so
        LIBS += $${PHYSX_LIBPATH}/PhysXExtensions_static.so
        LIBS += $${PHYSX_LIBPATH}/PhysXCooking.so
        LIBS += $${PHYSX_LIBPATH}/PhysXPvdSDK_static.so
    }

    INCLUDEPATH += $$PHYSX_INCLUDEPATH
    INCLUDEPATH += "sourceCode/dynamics/physx"
}

HEADERS += ../../coppeliaSimLib/sourceCode/interfaces/dummyClasses.h \
    ../include/simLib/simLib.h \
    ../include/simMath/3Vector.h \
    ../include/simMath/4Vector.h \
    ../include/simMath/7Vector.h \
    ../include/simMath/3X3Matrix.h \
    ../include/simMath/4X4Matrix.h \
    ../include/simMath/mXnMatrix.h \
    ../include/simMath/mathFuncs.h \
    
SOURCES += ../include/simLib/simLib.cpp \
    ../include/simMath/3Vector.cpp \
    ../include/simMath/4Vector.cpp \
    ../include/simMath/7Vector.cpp \
    ../include/simMath/3X3Matrix.cpp \
    ../include/simMath/4X4Matrix.cpp \
    ../include/simMath/mXnMatrix.cpp \
    ../include/simMath/mathFuncs.cpp \

HEADERS += ../include/simStack/stackBool.h \
    ../include/simStack/stackNull.h \
    ../include/simStack/stackNumber.h \
    ../include/simStack/stackString.h \
    ../include/simStack/stackArray.h \
    ../include/simStack/stackMap.h \
    ../include/simStack/stackObject.h \

SOURCES += ../include/simStack/stackBool.cpp \
    ../include/simStack/stackNull.cpp \
    ../include/simStack/stackNumber.cpp \
    ../include/simStack/stackString.cpp \
    ../include/simStack/stackArray.cpp \
    ../include/simStack/stackMap.cpp \
    ../include/simStack/stackObject.cpp \

HEADERS += sourceCode/dynamics/CollShapeDyn_base.h \
    sourceCode/dynamics/ConstraintDyn_base.h \
    sourceCode/dynamics/ParticleObjectContainer_base.h \
    sourceCode/dynamics/ParticleObject_base.h \
    sourceCode/dynamics/ParticleDyn_base.h \
    sourceCode/dynamics/RigidBodyDyn_base.h \
    sourceCode/dynamics/RigidBodyContainerDyn_base.h \
    sourceCode/simExtDyn.h \

SOURCES += sourceCode/dynamics/CollShapeDyn_base.cpp \
    sourceCode/dynamics/ConstraintDyn_base.cpp \
    sourceCode/dynamics/ParticleObjectContainer_base.cpp \
    sourceCode/dynamics/ParticleObject_base.cpp \
    sourceCode/dynamics/ParticleDyn_base.cpp \
    sourceCode/dynamics/RigidBodyDyn_base.cpp \
    sourceCode/dynamics/RigidBodyContainerDyn_base.cpp \
    sourceCode/simExtDyn.cpp \

BULLET_2_78_ENGINE {
    HEADERS +=sourceCode/dynamics/bullet_2_78/CollShapeDyn.h \
        sourceCode/dynamics/bullet_2_78/RigidBodyDyn.h \
        sourceCode/dynamics/bullet_2_78/ConstraintDyn.h \
        sourceCode/dynamics/bullet_2_78/RigidBodyContainerDyn.h \
        sourceCode/dynamics/bullet_2_78/ParticleDyn.h
    SOURCES +=sourceCode/dynamics/bullet_2_78/CollShapeDyn.cpp \
        sourceCode/dynamics/bullet_2_78/RigidBodyDyn.cpp \
        sourceCode/dynamics/bullet_2_78/ConstraintDyn.cpp \
        sourceCode/dynamics/bullet_2_78/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/bullet_2_78/ParticleDyn.cpp
}

BULLET_2_83_ENGINE {
    HEADERS += sourceCode/dynamics/bullet_2_83/CollShapeDyn.h \
        sourceCode/dynamics/bullet_2_83/RigidBodyDyn.h \
        sourceCode/dynamics/bullet_2_83/ConstraintDyn.h \
        sourceCode/dynamics/bullet_2_83/RigidBodyContainerDyn.h \
        sourceCode/dynamics/bullet_2_83/ParticleDyn.h
    SOURCES += sourceCode/dynamics/bullet_2_83/CollShapeDyn.cpp \
        sourceCode/dynamics/bullet_2_83/RigidBodyDyn.cpp \
        sourceCode/dynamics/bullet_2_83/ConstraintDyn.cpp \
        sourceCode/dynamics/bullet_2_83/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/bullet_2_83/ParticleDyn.cpp
}

ODE_ENGINE {
    HEADERS +=sourceCode/dynamics/ode/CollShapeDyn.h \
        sourceCode/dynamics/ode/RigidBodyDyn.h \
        sourceCode/dynamics/ode/ConstraintDyn.h \
        sourceCode/dynamics/ode/RigidBodyContainerDyn.h \
        sourceCode/dynamics/ode/ParticleDyn.h
    SOURCES +=sourceCode/dynamics/ode/CollShapeDyn.cpp \
        sourceCode/dynamics/ode/RigidBodyDyn.cpp \
        sourceCode/dynamics/ode/ConstraintDyn.cpp \
        sourceCode/dynamics/ode/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/ode/ParticleDyn.cpp
}

NEWTON_ENGINE {
    HEADERS +=sourceCode/dynamics/newton/CollShapeDyn.h \
        sourceCode/dynamics/newton/RigidBodyDyn.h \
        sourceCode/dynamics/newton/ConstraintDyn.h \
        sourceCode/dynamics/newton/RigidBodyContainerDyn.h \
        sourceCode/dynamics/newton/ParticleDyn.h
    SOURCES +=sourceCode/dynamics/newton/CollShapeDyn.cpp \
        sourceCode/dynamics/newton/RigidBodyDyn.cpp \
        sourceCode/dynamics/newton/ConstraintDyn.cpp \
        sourceCode/dynamics/newton/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/newton/ParticleDyn.cpp
}

VORTEX_ENGINE {
    HEADERS +=sourceCode/dynamics/vortex/CollShapeDyn.h \
        sourceCode/dynamics/vortex/RigidBodyDyn.h \
        sourceCode/dynamics/vortex/ConstraintDyn.h \
        sourceCode/dynamics/vortex/RigidBodyContainerDyn.h \
        sourceCode/dynamics/vortex/ParticleDyn.h
    SOURCES +=sourceCode/dynamics/vortex/CollShapeDyn.cpp \
        sourceCode/dynamics/vortex/RigidBodyDyn.cpp \
        sourceCode/dynamics/vortex/ConstraintDyn.cpp \
        sourceCode/dynamics/vortex/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/vortex/ParticleDyn.cpp
}

MUJOCO_ENGINE {
    HEADERS +=sourceCode/dynamics/mujoco/CollShapeDyn.h \
        sourceCode/dynamics/mujoco/RigidBodyDyn.h \
        sourceCode/dynamics/mujoco/ConstraintDyn.h \
        sourceCode/dynamics/mujoco/RigidBodyContainerDyn.h \
        sourceCode/dynamics/mujoco/ParticleDyn.h \
        sourceCode/dynamics/mujoco/tinyxml2.h \
        sourceCode/dynamics/mujoco/xmlser.h
    SOURCES +=sourceCode/dynamics/mujoco/CollShapeDyn.cpp \
        sourceCode/dynamics/mujoco/RigidBodyDyn.cpp \
        sourceCode/dynamics/mujoco/ConstraintDyn.cpp \
        sourceCode/dynamics/mujoco/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/mujoco/ParticleDyn.cpp \
        sourceCode/dynamics/mujoco/tinyxml2.cpp \
        sourceCode/dynamics/mujoco/xmlser.cpp
}

PHYSX_ENGINE {
    HEADERS +=sourceCode/dynamics/physx/CollShapeDyn.h \
        sourceCode/dynamics/physx/RigidBodyDyn.h \
        sourceCode/dynamics/physx/ConstraintDyn.h \
        sourceCode/dynamics/physx/RigidBodyContainerDyn.h \
        sourceCode/dynamics/physx/ParticleDyn.h 
    SOURCES +=sourceCode/dynamics/physx/CollShapeDyn.cpp \
        sourceCode/dynamics/physx/RigidBodyDyn.cpp \
        sourceCode/dynamics/physx/ConstraintDyn.cpp \
        sourceCode/dynamics/physx/RigidBodyContainerDyn.cpp \
        sourceCode/dynamics/physx/ParticleDyn.cpp
}

BULLET_2_78_ENGINE {
    HEADERS +=sourceCode/dynamics/bullet_2_78/bullet_2_78/btBulletDynamicsCommon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/btBulletCollisionCommon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btSimpleBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btQuantizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDispatcher.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvt.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btAxisSweep3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/SphereTriangleDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btUnionFind.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSimulationIslandManager.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btManifoldResult.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btGhostObject.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionObject.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionDispatcher.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionCreateFunc.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionConfiguration.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btUniformScalingShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMesh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleInfoMap.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleBuffer.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTetrahedronShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStridingMeshInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStaticPlaneShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btSphereShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btShapeHull.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btOptimizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultiSphereShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMinkowskiSumShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMaterial.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btEmptyShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCylinderShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexPointCloudShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexInternalShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexHullShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvex2dShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConeShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConcaveShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCompoundShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionMargin.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCapsuleShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBoxShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBox2dShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_tri_collision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_radixsort.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_memory.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_math.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_linear_math.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_hash_table.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_geometry.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_geom_types.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_contact.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_clip_polygon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_set.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_collision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_bitset.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_basic_geometry_operations.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_array.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btTriangleShapeEx.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btQuantization.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactQuantizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactMassUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGeometryOperations.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGenericPoolAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btContactProcessing.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btClipPolygon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btBoxCollision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPointCollector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btUniversalConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btTypedConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolverConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolverBody.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSliderConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btJacobianEntry.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHingeConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHinge2Constraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactSolverInfo.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConstraintSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btSimpleDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btRigidBody.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btContinuousDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btActionInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btVector3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btTransformUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btTransform.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btStackAlloc.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btSerializer.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btScalar.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btRandom.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuickprof.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuaternion.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuadWord.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btPoolAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMotionState.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMinMax.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMatrix3x3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btList.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btIDebugDraw.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btHashMap.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btGeometryUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btDefaultMotionState.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btConvexHull.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedObjectArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAabbUtil2.h \
}


ODE_ENGINE {
    HEADERS +=sourceCode/dynamics/ode/ode/rotation.h \
    sourceCode/dynamics/ode/ode/odemath.h \
    sourceCode/dynamics/ode/ode/odeinit.h \
    sourceCode/dynamics/ode/ode/odecpp_collision.h \
    sourceCode/dynamics/ode/ode/odecpp.h \
    sourceCode/dynamics/ode/ode/odeconfig.h \
    sourceCode/dynamics/ode/ode/ode.h \
    sourceCode/dynamics/ode/ode/objects.h \
    sourceCode/dynamics/ode/ode/misc.h \
    sourceCode/dynamics/ode/ode/memory.h \
    sourceCode/dynamics/ode/ode/matrix.h \
    sourceCode/dynamics/ode/ode/mass.h \
    sourceCode/dynamics/ode/ode/export-dif.h \
    sourceCode/dynamics/ode/ode/error.h \
    sourceCode/dynamics/ode/ode/contact.h \
    sourceCode/dynamics/ode/ode/compatibility.h \
    sourceCode/dynamics/ode/ode/common.h \
    sourceCode/dynamics/ode/ode/collision_trimesh.h \
    sourceCode/dynamics/ode/ode/collision_space.h \
    sourceCode/dynamics/ode/ode/collision.h \
    sourceCode/dynamics/ode/ode/src/util.h \
    sourceCode/dynamics/ode/ode/src/step.h \
    sourceCode/dynamics/ode/ode/src/quickstep.h \
    sourceCode/dynamics/ode/ode/src/odetls.h \
    sourceCode/dynamics/ode/ode/src/odeou.h \
    sourceCode/dynamics/ode/ode/src/obstack.h \
    sourceCode/dynamics/ode/ode/src/objects.h \
    sourceCode/dynamics/ode/ode/src/mat.h \
    sourceCode/dynamics/ode/ode/src/lcp.h \
    sourceCode/dynamics/ode/ode/src/heightfield.h \
    sourceCode/dynamics/ode/ode/src/config.h \
    sourceCode/dynamics/ode/ode/src/collision_util.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_colliders.h \
    sourceCode/dynamics/ode/ode/src/collision_transform.h \
    sourceCode/dynamics/ode/ode/src/collision_std.h \
    sourceCode/dynamics/ode/ode/src/collision_space_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_kernel.h \
    sourceCode/dynamics/ode/ode/src/array.h \
    sourceCode/dynamics/ode/ode/src/joints/universal.h \
    sourceCode/dynamics/ode/ode/src/joints/slider.h \
    sourceCode/dynamics/ode/ode/src/joints/pu.h \
    sourceCode/dynamics/ode/ode/src/joints/pr.h \
    sourceCode/dynamics/ode/ode/src/joints/plane2d.h \
    sourceCode/dynamics/ode/ode/src/joints/piston.h \
    sourceCode/dynamics/ode/ode/src/joints/null.h \
    sourceCode/dynamics/ode/ode/src/joints/lmotor.h \
    sourceCode/dynamics/ode/ode/src/joints/joints.h \
    sourceCode/dynamics/ode/ode/src/joints/joint_internal.h \
    sourceCode/dynamics/ode/ode/src/joints/joint.h \
    sourceCode/dynamics/ode/ode/src/joints/hinge2.h \
    sourceCode/dynamics/ode/ode/src/joints/hinge.h \
    sourceCode/dynamics/ode/ode/src/joints/fixed.h \
    sourceCode/dynamics/ode/ode/src/joints/contact.h \
    sourceCode/dynamics/ode/ode/src/joints/ball.h \
    sourceCode/dynamics/ode/ode/src/joints/amotor.h \
    sourceCode/dynamics/ode/ode/OPCODE/Opcode.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_VolumeCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TriTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TriBoxOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeBuilders.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Settings.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Picking.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OptimizedTree.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OBBCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Model.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_MeshInterface.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_IceHook.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_HybridModel.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Common.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Collider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BoxBoxOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BaseModel.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBTree.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceUtils.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTypes.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriList.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriangle.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceSegment.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRevisitedRadix.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRay.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRandom.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePreprocessor.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePoint.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePlane.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePairs.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceOBB.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMemoryMacros.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix4x4.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix3x3.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceLSS.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceIndexedTriangle.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceHPoint.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceFPU.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceContainer.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceBoundingSphere.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAxes.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAABB.h \
    sourceCode/dynamics/ode/ode/src/collision_libccd.h \
    sourceCode/dynamics/ode/ode/src/collision_space_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_std.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_colliders.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_internal.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/alloc.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/ccd.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/compiler.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/config.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/dbg.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/list.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/polytope.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/quat.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/simplex.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/support.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/vec3.h \
}


NEWTON_ENGINE {
    HEADERS +=sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/Newton.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonClass.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dg.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAABBPolygonSoup.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgArray.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAsyncThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull3d.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull4d.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgCRC.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDebug.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDelaunayTetrahedralization.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgFastQueue.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGoogol.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGraph.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgHeap.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgIntersections.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMemory.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMutexThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgNode.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgObb.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPathFinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPlane.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupBuilder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupDatabase.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedra.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedraMassProperties.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgQuaternion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRandom.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRef.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRefCounter.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRtti.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSmallDeterminant.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSPDMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgStack.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgStdafx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadHive.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadProfiler.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTree.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTypes.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBallConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBilateralConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBodyMasterList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhase.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseAggregate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseDefault.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhasePersistent.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollision.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBox.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBVH.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCapsule.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionChamferCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompound.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompoundFractured.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCone.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvex.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexHull.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexPolygon.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableClothPatch.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableSolidMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionHeightField.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionInstance.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionNull.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionScene.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionSphere.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCapsule.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionUserMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCorkscrewConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBodiesUpdate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDynamicBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgHingeConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgKinematicBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgPhysics.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgPhysicsStdafx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSkeletonContainer.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSlidingConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUniversalConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUpVectorConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUserConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorld.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicUpdate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dLinearAlgebra.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMathDefines.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dQuaternion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dStdAfxMath.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/Custom6DOF.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomAlloc.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomArcticulatedTransformManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomBallAndSocket.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomControllerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomCorkScrew.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomDryRollingFriction.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomGear.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHinge.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHingeActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomInputManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJoint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJointLibraryStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomKinematicController.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPathFollow.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPulley.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomRackAndPinion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlider.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSliderActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlidingContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomTriggerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversal.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversalActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUpVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUserBlank.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dBaseHierarchy.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dClassInfo.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersAlloc.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dCRC.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dHeap.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRefCounter.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRtti.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dString.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dTree.h \
        sourceCode/dynamics/newton/NewtonConvertUtil.h
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPlayerControllerManager.h \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerBodyState.h \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerComponent.h \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerJoint.h \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerManager.h \
    win32 {
        HEADERS += sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2/pthread.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAMP.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpAllocator.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpInstance.h
    }
}


BULLET_2_78_ENGINE {
    SOURCES +=sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDispatcher.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvt.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btUnionFind.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btManifoldResult.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btGhostObject.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionObject.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btUniformScalingShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMesh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleCallback.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleBuffer.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTetrahedronShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btSphereShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btShapeHull.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btOptimizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultiSphereShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btEmptyShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCylinderShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexInternalShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexHullShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvex2dShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConeShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConcaveShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCompoundShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCapsuleShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBoxShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBox2dShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_tri_collision.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_memory.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_contact.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_set.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btTriangleShapeEx.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGenericPoolAllocator.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btContactProcessing.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btRigidBody.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btSerializer.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuickprof.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btGeometryUtil.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btConvexHull.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedAllocator.cpp \
}

ODE_ENGINE {
    SOURCES +=sourceCode/dynamics/ode/ode/src/util.cpp \
    sourceCode/dynamics/ode/ode/src/timer.cpp \
    sourceCode/dynamics/ode/ode/src/step.cpp \
    sourceCode/dynamics/ode/ode/src/sphere.cpp \
    sourceCode/dynamics/ode/ode/src/rotation.cpp \
    sourceCode/dynamics/ode/ode/src/ray.cpp \
    sourceCode/dynamics/ode/ode/src/quickstep.cpp \
    sourceCode/dynamics/ode/ode/src/plane.cpp \
    sourceCode/dynamics/ode/ode/src/odetls.cpp \
    sourceCode/dynamics/ode/ode/src/odeou.cpp \
    sourceCode/dynamics/ode/ode/src/odemath.cpp \
    sourceCode/dynamics/ode/ode/src/odeinit.cpp \
    sourceCode/dynamics/ode/ode/src/ode.cpp \
    sourceCode/dynamics/ode/ode/src/obstack.cpp \
    sourceCode/dynamics/ode/ode/src/misc.cpp \
    sourceCode/dynamics/ode/ode/src/memory.cpp \
    sourceCode/dynamics/ode/ode/src/matrix.cpp \
    sourceCode/dynamics/ode/ode/src/mat.cpp \
    sourceCode/dynamics/ode/ode/src/mass.cpp \
    sourceCode/dynamics/ode/ode/src/lcp.cpp \
    sourceCode/dynamics/ode/ode/src/heightfield.cpp \
    sourceCode/dynamics/ode/ode/src/fastltsolve.c \
    sourceCode/dynamics/ode/ode/src/fastlsolve.c \
    sourceCode/dynamics/ode/ode/src/fastldlt.c \
    sourceCode/dynamics/ode/ode/src/fastdot.c \
    sourceCode/dynamics/ode/ode/src/export-dif.cpp \
    sourceCode/dynamics/ode/ode/src/error.cpp \
    sourceCode/dynamics/ode/ode/src/cylinder.cpp \
    sourceCode/dynamics/ode/ode/src/convex.cpp \
    sourceCode/dynamics/ode/ode/src/collision_util.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh_new.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_sphere.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ray.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_plane.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_opcode.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_gimpact.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_distance.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_disabled.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ccylinder.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_box.cpp \
    sourceCode/dynamics/ode/ode/src/collision_transform.cpp \
    sourceCode/dynamics/ode/ode/src/collision_space.cpp \
    sourceCode/dynamics/ode/ode/src/collision_sapspace.cpp \
    sourceCode/dynamics/ode/ode/src/collision_quadtreespace.cpp \
    sourceCode/dynamics/ode/ode/src/collision_kernel.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_trimesh.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_sphere.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_plane.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_box.cpp \
    sourceCode/dynamics/ode/ode/src/capsule.cpp \
    sourceCode/dynamics/ode/ode/src/box.cpp \
    sourceCode/dynamics/ode/ode/src/array.cpp \
    sourceCode/dynamics/ode/ode/src/joints/universal.cpp \
    sourceCode/dynamics/ode/ode/src/joints/slider.cpp \
    sourceCode/dynamics/ode/ode/src/joints/pu.cpp \
    sourceCode/dynamics/ode/ode/src/joints/pr.cpp \
    sourceCode/dynamics/ode/ode/src/joints/plane2d.cpp \
    sourceCode/dynamics/ode/ode/src/joints/piston.cpp \
    sourceCode/dynamics/ode/ode/src/joints/null.cpp \
    sourceCode/dynamics/ode/ode/src/joints/lmotor.cpp \
    sourceCode/dynamics/ode/ode/src/joints/joint.cpp \
    sourceCode/dynamics/ode/ode/src/joints/hinge2.cpp \
    sourceCode/dynamics/ode/ode/src/joints/hinge.cpp \
    sourceCode/dynamics/ode/ode/src/joints/fixed.cpp \
    sourceCode/dynamics/ode/ode/src/joints/contact.cpp \
    sourceCode/dynamics/ode/ode/src/joints/ball.cpp \
    sourceCode/dynamics/ode/ode/src/joints/amotor.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Opcode.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_VolumeCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeBuilders.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Picking.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OptimizedTree.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OBBCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Model.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_MeshInterface.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_HybridModel.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Common.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Collider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BaseModel.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBTree.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceUtils.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriangle.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceSegment.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRevisitedRadix.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRay.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRandom.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePoint.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePlane.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceOBB.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix4x4.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix3x3.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceIndexedTriangle.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceHPoint.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceContainer.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAABB.cpp \
    sourceCode/dynamics/ode/ode/src/collision_libccd.cpp \
    sourceCode/dynamics/ode/ode/src/nextafterf.c \
    sourceCode/dynamics/ode/ode/libccd/src/alloc.c \
    sourceCode/dynamics/ode/ode/libccd/src/ccd.c \
    sourceCode/dynamics/ode/ode/libccd/src/mpr.c \
    sourceCode/dynamics/ode/ode/libccd/src/polytope.c \
    sourceCode/dynamics/ode/ode/libccd/src/support.c \
    sourceCode/dynamics/ode/ode/libccd/src/vec3.c \
}
    
NEWTON_ENGINE {
    SOURCES +=sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/Newton.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonClass.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dg.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAABBPolygonSoup.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAsyncThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull3d.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull4d.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgCRC.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDebug.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDelaunayTetrahedralization.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGoogol.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgIntersections.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMemory.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMutexThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgNode.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgObb.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupBuilder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedra.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedraMassProperties.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgQuaternion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRandom.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRef.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRefCounter.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSmallDeterminant.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSPDMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadHive.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTree.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTypes.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBallConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBilateralConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBodyMasterList.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhase.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseAggregate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseDefault.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhasePersistent.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollision.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBox.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBVH.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCapsule.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionChamferCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompound.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompoundFractured.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCone.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvex.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexHull.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexPolygon.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableClothPatch.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableSolidMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionHeightField.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionInstance.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionNull.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionScene.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionSphere.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCapsule.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionUserMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCorkscrewConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBodiesUpdate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDynamicBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgHingeConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgKinematicBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgNarrowPhaseCollision.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSkeletonContainer.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSlidingConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUniversalConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUpVectorConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUserConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorld.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsParallelSolver.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsSimpleSolver.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicUpdate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect1.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect2.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect3.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect4.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect5.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect6.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dLinearAlgebra.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMathDefines.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dQuaternion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dStdAfxMath.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/Custom6DOF.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomAlloc.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomArcticulatedTransformManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomBallAndSocket.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomControllerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomCorkScrew.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomDryRollingFriction.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomGear.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHinge.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHingeActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomInputManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJoint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJointLibraryStdAfx.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomKinematicController.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPathFollow.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPulley.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomRackAndPinion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlider.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSliderActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlidingContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomTriggerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversal.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversalActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUpVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUserBlank.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dBaseHierarchy.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dClassInfo.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersAlloc.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersStdAfx.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dCRC.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRefCounter.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dString.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dTree.cpp
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPlayerControllerManager.cpp \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerBodyState.cpp \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerComponent.cpp \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerJoint.cpp \
#        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerManager.cpp \


    win32 {
        SOURCES += sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2/pthread.c \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAMP.cpp \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpSolver.cpp \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpInstance.cpp
    }
}


unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
