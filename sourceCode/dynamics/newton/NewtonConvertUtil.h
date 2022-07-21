#pragma once

#include "3X3Matrix.h"
#include "7Vector.h"
#include "dMatrix.h"

inline dMatrix GetDMatrixFromCoppeliaSimTransformation (const C7Vector& tr)
{
    C3X3Matrix m(tr.Q.getMatrix());
    dMatrix matrix(dVector(m.axis[0](0),m.axis[0](1),m.axis[0](2),0.0f),
                  dVector(m.axis[1](0),m.axis[1](1),m.axis[1](2),0.0f),
                  dVector(m.axis[2](0),m.axis[2](1),m.axis[2](2),0.0f),
                  dVector(tr.X(0),tr.X(1),tr.X(2),1.0f));
    return(matrix);
}
