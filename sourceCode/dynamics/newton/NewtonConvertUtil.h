#pragma once

#include <simMath/3X3Matrix.h>
#include <simMath/4X4Matrix.h>
#include <simMath/7Vector.h>
#include <dMatrix.h>

inline dMatrix GetDMatrixFromCoppeliaSimTransformation (const C7Vector& tr)
{
    C3X3Matrix m(tr.Q.getMatrix());
    dMatrix matrix(dVector(m.axis[0](0),m.axis[0](1),m.axis[0](2),0.0),
                  dVector(m.axis[1](0),m.axis[1](1),m.axis[1](2),0.0),
                  dVector(m.axis[2](0),m.axis[2](1),m.axis[2](2),0.0),
                  dVector(tr.X(0),tr.X(1),tr.X(2),1.0));
    return(matrix);
}

inline C7Vector GetCoppeliaSimTransformationFromDMatrix (const dMatrix& matrix)
{
    C4X4Matrix m;
    for (size_t i=0;i<3;i++)
    {
        m.M.axis[0](i)=matrix.m_front[int(i)];
        m.M.axis[1](i)=matrix.m_up[int(i)];
        m.M.axis[2](i)=matrix.m_right[int(i)];
        m.X(i)=matrix.m_posit[int(i)];
    }
    return(m.getTransformation());
}
