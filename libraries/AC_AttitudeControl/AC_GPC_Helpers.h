#pragma once

#include "AC_debug.h"
#ifndef GPC_DEBUG
#include <GCS_MAVLink/GCS.h>
#endif

#ifdef GPC_DEBUG

#define FLT_EPSILON 1.1920928955078125e-7F

#include <type_traits>
#include <math.h>

/*
 * Type-safe sign (signum) function
 */
template <typename T> 
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
                  "Template parameter not of type float");
    return (fabsf(static_cast<float>(fVal1)) < FLT_EPSILON);
}

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

float constrain_float(const float amt, const float low, const float high);

#endif

float normalize(const float x, const float x_shift, const float x_d);
float denormalize(const float x, const float x_shift, const float x_d);

class DebugLogger
{
public:
#ifdef GPC_DEBUG
    DebugLogger() {}
#else
    DebugLogger(GCS &gcs);
#endif
    virtual ~DebugLogger() {}

    void debug_msg(const char *fmt, ...);

private:
#ifndef GPC_DEBUG
    GCS &_gcs;
#endif
};

