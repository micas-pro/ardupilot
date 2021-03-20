#include "AC_GPC_Helpers.h"
#ifndef GPC_DEBUG
#include <AP_Math/AP_Math.h>
#else
#include <math.h>
#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>
#include <stdarg.h>
#include <stdio.h>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std::chrono;
#endif

#ifdef GPC_DEBUG

void DebugLogger::debug_msg(const char *fmt, ...)
{
    auto t_now = high_resolution_clock::now();
    auto t_now_us = time_point_cast<std::chrono::microseconds>(t_now);
    va_list arg_list;
    va_start(arg_list, fmt);
    char buff[1000];
    vsnprintf(buff, 1000, fmt, arg_list);
    va_end(arg_list);
    printf("[DEBUG][%li us] %s\n", t_now_us.time_since_epoch().count(), buff);
}

float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

#else

DebugLogger::DebugLogger(GCS &gcs) :
    _gcs(gcs)
{
}

void DebugLogger::debug_msg(const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    _gcs.send_textv(MAV_SEVERITY_DEBUG, fmt, arg_list);
    va_end(arg_list);
}

#endif

float normalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : (x - x_shift) / x_d;
}

float denormalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : x * x_d + x_shift;
}

