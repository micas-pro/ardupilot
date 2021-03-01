#pragma GCC optimize("O2")

#include "matrixNxM.h"

template<typename T, uint8_t N, uint8_t M>
MatrixNxM<T, N, M>& MatrixNxM<T, N, M>::operator -=(const MatrixNxM<T, N, M> &B) 
{
    for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] -= B[i][j];
    
    return *this;
}

template<typename T, uint8_t N, uint8_t M>
MatrixNxM<T, N, M>& MatrixNxM<T, N, M>::operator +=(const MatrixNxM<T, N, M> &B) 
{
    for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] += B[i][j];
    
    return *this;
}
