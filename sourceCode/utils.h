#pragma once

#include <simLib/simLib.h>

class CXSceneObject
{
public:
};

class CXShape : public CXSceneObject
{
public:
};

class CXJoint : public CXSceneObject
{
public:
};

class CXForceSensor : public CXSceneObject
{
public:
};

class CXDummy : public CXSceneObject
{
public:
};

class CXGeomProxy
{
public:
};

class CXGeomWrap
{
public:
};

class CXGeometric
{
public:
};

static bool isJointInDynamicMode(CXSceneObject* joint)
{
    int m=_simGetJointMode(joint);
    while (m==sim_jointmode_dependent)
    {
        int masterJ;
        sReal off,mult;
        simGetJointDependency(_simGetObjectID(joint),&masterJ,&off,&mult);
        if (masterJ==-1)
            break;
        joint=(CXSceneObject*)_simGetObject(masterJ);
        m=_simGetJointMode(joint);
    }
    return(m==sim_jointmode_dynamic);
}

static CXSceneObject* getJointOrFsensorChild(CXSceneObject* object, int* objType = nullptr, int* objHandle = nullptr)
{
    CXSceneObject* retVal = nullptr;
    bool isJoint = (_simGetObjectType(object) == sim_object_joint_type);
    int rcnt = 0;
    int childrenCount = 0;
    CXSceneObject** childrenPointer=(CXSceneObject**)_simGetObjectChildren(object,&childrenCount);
    for (int i = 0; i < childrenCount; i++)
    {
        CXSceneObject* child = childrenPointer[i];
        int t = _simGetObjectType(child);
        int inc = rcnt;
        if (t == sim_object_shape_type)
        {
            if (_simIsShapeDynamicallyStatic(child) == 0)
                rcnt++;
        }
        #ifdef INCLUDE_MUJOCO_CODE
            else if ( (t == sim_object_joint_type) && isJoint ) // consecutive joints
            {
                if (isJointInDynamicMode(child))
                    rcnt++;
            }
        #endif
        else if (t == sim_object_dummy_type)
        {
            int linkedDummyHandle=-1;
            int linkType=_simGetDummyLinkType(child, &linkedDummyHandle);
            #ifdef INCLUDE_MUJOCO_CODE
                if ( (linkedDummyHandle != -1) && ( (linkType == sim_dummylink_dynloopclosure)||(linkType == sim_dummylink_dyntendon) ) )
            #else
                if ( (linkedDummyHandle != -1) && (linkType == sim_dummylink_dynloopclosure) )
            #endif
                    rcnt++;
        }
        if (inc != rcnt)
        {
            retVal = child;
            if (objType != nullptr)
                objType[0] = t;
            if (objHandle != nullptr)
                objHandle[0] = _simGetObjectID(child);
        }
    }
    if (rcnt > 1)
        retVal = nullptr;
    return retVal;
}

