#include "AC_GPC_Helpers.h"
#include <AP_Math/AP_Math.h>

float normalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : (x - x_shift) / x_d;
}

float denormalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : x * x_d + x_shift;
}

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

