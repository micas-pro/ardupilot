#include "AC_GPC_LinearModels.h"

float normalize_dy(const float dy, const uint8_t n) 
{
    return normalize(dy, defines::gpc::normalization_dy[n][0], defines::gpc::normalization_dy[n][1]);
}

float normalize_y(const float y) 
{
    return normalize_dy(y, 0);
}

float normalize_u(const float u) 
{
    return normalize(u, defines::gpc::normalization_u[0], defines::gpc::normalization_u[1]);
}

float normalize_d1u(const float d1u) 
{
    return normalize(d1u, defines::gpc::normalization_d1u[0], defines::gpc::normalization_d1u[1]);
}

float denormalize_d1y(const float d1y) 
{
    return denormalize(d1y, defines::gpc::normalization_dy[1][0], defines::gpc::normalization_dy[1][1]);
}

void calculate_diffs(float *diffs, const size_t &n) 
{
    for (size_t i=0; i<n; i++) {
        diffs[i] = diffs[i+1] - diffs[i];
    }
}

