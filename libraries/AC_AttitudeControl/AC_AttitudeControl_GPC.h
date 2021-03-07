#pragma once

#include "AC_AttitudeControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixNxM.h>
#include <AP_Math/circular_buffer.h>
#include "AC_AttitudeControl_GPC_defines.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#define GPC_N                               25
#define GPC_Nu                              1

#define _GPC_DEBUG_LOG_1HZ                  (c % 400 == 0)
#define _GPC_DEBUG_LOG_2HZ                  (c % 200 == 0)
#define _GPC_DEBUG_LOG_5HZ                  (c % 80 == 0)

#define GPC_DEBUG_LOG_INIT                  static uint32_t c = 0; c++;
#define GPC_DEBUG_LOG(__fmt__, ...)         _gcs.send_text(MAV_SEVERITY_DEBUG, __fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_1HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_1HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_2HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_2HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)
#define GPC_DEBUG_LOG_5HZ(__fmt__, ...)     if (_GPC_DEBUG_LOG_5HZ) GPC_DEBUG_LOG(__fmt__, __VA_ARGS__)

float normalize(const float x, const float x_shift, const float x_d);
float normalize_dy(const float dy, const uint8_t n);
float normalize_y(const float y);
float normalize_u(const float u);
void calculate_diffs(float *diffs, const size_t &n);

float denormalize(const float x, const float x_shift, const float x_d);
float denormalize_dy1(const float dy1);

template <typename T = float>
struct GPC_Params
{
    T lambda;
    T min_u;
    T max_u;
};

template <typename T>
class LinearModel;

template <typename T>
class LinearModel
{
public:
    LinearModel(const uint8_t y_steps, const uint8_t u_steps):
        _y_steps(y_steps),
        _u_steps(u_steps)
    {
        _y = new CircularBuffer<T>(y_steps);
        _u = new CircularBuffer<T>(u_steps);
    }

    virtual ~LinearModel() 
    {
        delete _y;
        delete _u;
    }

    void load_weights(const T w[]);
    T predict_one_step(const T &u, const T &dk);
    void measured_y(const T &y);
    void reset_to(const LinearModel &model);
    bool ready();

private:    

    T predict_one_step();

    MatrixNxM<T, GPC_LINEAR_MODEL_WEIGHTS, 1> _weights;
    MatrixNxM<T, 1, GPC_LINEAR_MODEL_WEIGHTS> _state;
    uint8_t _y_steps, _u_steps;
    CircularBuffer<T> *_y;
    CircularBuffer<T> *_u;
};

template <typename T, uint8_t N, uint8_t Nu>
class GPC_Controller
{
public: 
    GPC_Controller(const GPC_Params<T> gpc_params, LinearModel<T> *model, LinearModel<T> *y0_model, GCS &gcs)
        : _gpc_params(gpc_params),
        _model(model),
        _y0_model(y0_model),
        _gcs(gcs)
    {
        _current_u = T();
    }

    virtual ~GPC_Controller() 
    {
        delete _model;
        delete _y0_model;
    }

    void initialize();
    void set_lambda(const T &lambda);
    const T run_step(const T &y, const T &target_y);
    const T get_current_u() { return _current_u; }

private:
    GCS &_gcs;
    GPC_Params<T> _gpc_params;
    LinearModel<T> *_model, *_y0_model;
    T _current_u;
    MatrixNxM<T, Nu, N> K;
};

class AC_AttitudeControl_GPC 
{
public:
    AC_AttitudeControl_GPC(GCS &gcs);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_GPC();

    float get_pitch();
    void set_lambda(const float lambda);
    void rate_controller_run(const float roll, const float target_roll, const float pitch, const float target_pitch, const float yaw, const float target_yaw);

    void debug_msg(const char *fmt, ...);

private:    
    GCS &_gcs;
    GPC_Controller<float, GPC_N, GPC_Nu> *_gpc_pitch_controller;
};

