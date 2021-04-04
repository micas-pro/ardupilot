#pragma once
//#pragma GCC optimize("O2")

#include "math.h"
#include <stdint.h>
#include <string.h>


template<typename T, uint8_t N, uint8_t M>
class MatrixNxM 
{
public:
    // constructor from zeros
    MatrixNxM<T, N, M>() 
    {
        memset(v, 0, sizeof(v));        
    }

    // constructor from zeros
    MatrixNxM<T, N, M>(const MatrixNxM<T, N, M> &x) 
    {
        memcpy(v, x.v, sizeof(v)); 
    }

    // constructor from array
    MatrixNxM<T, N, M>(const T **x) 
    {
        for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] = x[i][j];
    }

    // constructor from const
    MatrixNxM<T, N, M>(const T &a) 
    {
        for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] = a;
    }

    virtual ~MatrixNxM<T, N, M>() {}

    // subtract B from the matrix
    MatrixNxM<T, N, M> &operator -=(const MatrixNxM<T, N, M> &B)
    {
        for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] -= B[i][j];
            
        return *this;        
    }

    // add B to the matrix
    MatrixNxM<T, N, M> &operator +=(const MatrixNxM<T, N, M> &B)
    {
        for (int i=0;i<N;i++)
            for (int j=0;j<M;j++)
                v[i][j] += B[i][j];
        
        return *this;
    }

    T* operator[](size_t idx) { return v[idx]; }
    const T* operator[](size_t idx) const { return v[idx]; }

private:
    T v[N][M];
};

template<typename T, uint8_t N, uint8_t M, uint8_t K>
MatrixNxM<T, N, K> operator*(const MatrixNxM<T, N, M> &A, const MatrixNxM<T, M, K> &B)
{
    MatrixNxM<T, N, K> result;

    for(uint8_t i = 0; i < N; ++i)
        for(uint8_t j = 0; j < K; ++j)
            for(uint8_t k = 0; k < M; ++k)
            {
                result[i][j] += A[i][k] * B[k][j];
            }

    return result;
}

template<typename T, uint8_t N, uint8_t M>
MatrixNxM<T, N, M> operator-(const MatrixNxM<T, N, M> &A, const MatrixNxM<T, N, M> &B)
{
    MatrixNxM<T, N, M> result;

    for (int i=0;i<N;i++)
        for (int j=0;j<M;j++)
            result[i][j] = A[i][j] - B[i][j];

    return result;
}

template<typename T, uint8_t N, uint8_t M>
MatrixNxM<T, N, M> operator+(const MatrixNxM<T, N, M> &A, const MatrixNxM<T, N, M> &B)
{
    MatrixNxM<T, N, M> result;

    for (int i=0;i<N;i++)
        for (int j=0;j<M;j++)
            result[i][j] = A[i][j] + B[i][j];

    return result;
}

