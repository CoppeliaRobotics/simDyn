# location of boost headers
#    BOOST_INCLUDEPATH = "d:/coppeliaRobotics/programming/vcpkg/installed/x64-windows/include" # (e.g. Windows)
    
# Boost libraries to link:
#    BOOST_LIB_PATH = "D:/coppeliaRobotics/programming/vcpkg/installed/x64-windows/lib" # (e.g. Windows)


# location of bullet files
#    BULLET_DIR = "e:/bullet3-2.83.7" # (e.g. Windows)
#    BULLET_BUILD_DIR = $${BULLET_DIR}/build/Release # (e.g. Windows)
#    BULLET_INCLUDEPATH = $${BULLET_DIR}/src # (e.g. Windows)

# location of vortex files
#    VORTEX_DIR = "c:/CM Labs/Vortex Studio 2017a" # (e.g. Windows)
#    VORTEX_INCLUDEPATH = $${VORTEX_DIR}/include # (e.g. Windows)
#    VORTEX_LIBPATH = $${VORTEX_DIR}/lib # (e.g. Windows)

# location of MuJoCo files
#    MUJOCO_PATH = "e:/Mujoco-3.2.6" # (e.g. Windows)
#    MUJOCO_LIBFILE = $${MUJOCO_PATH}/lib/mujoco.lib # (e.g. Windows)
#    MUJOCO_LIBPATH = $${MUJOCO_PATH}/lib # (e.g. Linux)
#    MUJOCO_INCLUDEPATH = $${MUJOCO_PATH}/include

exists(../config.pri) { include(../config.pri) }

