#pragma once

#include "AC_debug.h"
#include <GCS_MAVLink/GCS.h>

float normalize(const float x, const float x_shift, const float x_d);
float denormalize(const float x, const float x_shift, const float x_d);

class DebugLogger
{
public:
    DebugLogger(GCS &gcs);
    virtual ~DebugLogger() {}

    void debug_msg(const char *fmt, ...);

private:
    GCS &_gcs;
};

