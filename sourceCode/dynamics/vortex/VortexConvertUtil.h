#pragma once

#ifdef INCLUDE_VORTEX_CODE

#include "7Vector.h"
#include "Vx/VxTransform.h"


inline Vx::VxVector3 C3Vector2VxVector3(const C3Vector& inVector) { return Vx::VxVector3(Vx::VxReal(inVector(0)), Vx::VxReal(inVector(1)), Vx::VxReal(inVector(2))); }
inline C3Vector VxVector32C3Vector(const Vx::VxVector3& inVector) { return C3Vector((float)inVector[0], (float)inVector[1], (float)inVector[2]); }

inline Vx::VxReal getVortexUnsignedDouble(float v)
{
	if (v>=0.0f)
		return(double(v));
	return(Vx::VX_INFINITY);
}

inline bool areEquals(Vx::VxReal a, Vx::VxReal b, Vx::VxReal epsilon = Vx::VX_SMALL_EPSILON)
{
    return VxFabs(a-b) < epsilon;
}
#endif // INCLUDE_VORTEX_CODE
