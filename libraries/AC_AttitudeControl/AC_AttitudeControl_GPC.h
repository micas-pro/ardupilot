#pragma once

#ifndef GPC_DEBUG
#include "AC_AttitudeControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/system.h>
#endif

#include <AP_Math/matrixNxM.h>
#include <AP_Math/circular_buffer.h>
#include "AC_AttitudeControl_GPC_defines.h"
#include "AC_GPC_Helpers.h"
#include "AC_GPC_LinearModels.h"

#define DEBUG_DATA_COUNT        5
static const float debug_data_u[DEBUG_DATA_COUNT] = {};
static const float debug_data_y[DEBUG_DATA_COUNT] = {};


template <typename T = float>
struct GPC_Params
{
    T lambda;
    T min_u;
    T max_u;
};

template <typename T, uint8_t N, uint8_t Nu>
class GPC_Controller
{
public: 
    GPC_Controller(const GPC_Params<T> gpc_params, LinearModelBase<T> *model, LinearModelBase<T> *y0_model, DebugLogger *logger)
        : _gpc_params(gpc_params),
        _model(model),
        _y0_model(y0_model),
        _logger(logger),
        _ty(CircularBuffer<T>(3))
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
    const T run_step(const T &y, const T &target_y, const T &throttle);
    const T get_current_u() { return _current_u; }

private:
    T transform_u(const T &u, const T &throttle);
    void guess_target_y();

    DebugLogger *_logger;
    GPC_Params<T> _gpc_params;
    LinearModelBase<T> *_model, *_y0_model;
    T _current_u;
    MatrixNxM<T, Nu, N> K;
    MatrixNxM<T, N, 1> _target_y;
    CircularBuffer<T> _ty;
    T _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW];
};

class SwitchingController
{
public:
    SwitchingController(DebugLogger *logger);
    virtual ~SwitchingController() {}

    void initialize();
    float run_step(const float &y, const float &target_y, const float &throttle);
    float get_current_u() { return _current_u; }

private:

    void read_and_smooth(const CircularBuffer<float> *c, float buff[], const size_t n, const float gaussian_weights[]);

    float _current_u;
    DebugLogger *_logger;
    CircularBuffer<float> _y;
};

#ifndef GPC_DEBUG

class AC_AttitudeControl_GPC : public DebugLogger
{
public:
    AC_AttitudeControl_GPC(GCS &gcs);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_GPC();

    float get_pitch();
    void set_lambda(const float lambda);
    void rate_controller_run(const float roll, const float target_roll, const float pitch, const float target_pitch, const float yaw, const float target_yaw, const float throttle);

private:    
    GPC_Controller<float, GPC_N, GPC_Nu> *_gpc_pitch_controller;
    SwitchingController *_switching_controller;
};

#endif

// templates implementation ----------------------------------------------------


template<typename T, uint8_t N, uint8_t Nu>
void GPC_Controller<T, N, Nu>::initialize() 
{
    _model->load_weights(defines::gpc::linear_model_w);
    _y0_model->load_weights(defines::gpc::linear_model_w);

    for (uint8_t i=0;i<GPC_Nu;i++)
        for (uint8_t j=0;j<GPC_N;j++)
            K[i][j] = defines::gpc::gpc_K[i][j];

    const T n = (GPC_LOWPASS_SMOOTHING_WINDOW-1)*0.9f + 1.1f;
    for (size_t i=0;i<GPC_LOWPASS_SMOOTHING_WINDOW-1;i++) {
        _low_pass_smoothing_weights[i] = 0.9f / n;
    }

    _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW-1] = 1.1f / n;
}

template<typename T, uint8_t N, uint8_t Nu>
void GPC_Controller<T, N, Nu>::set_lambda(const T &lambda) 
{
    _gpc_params.lambda = lambda;
}

template<typename T, uint8_t N, uint8_t Nu>
T GPC_Controller<T, N, Nu>::transform_u(const T &u, const T &throttle)
{
    //return (0.001168222604331f + 0.000575672724742 * throttle) * u / 0.001168222604331f;
    return u;
}

template<typename T, uint8_t N, uint8_t Nu>
const T GPC_Controller<T, N, Nu>::run_step(const T &y, const T &target_y, const T &throttle) 
{    
    GPC_DEBUG_LOG_INIT;

    const bool log05Hz = _GPC_DEBUG_LOG_05HZ;
#ifndef GPC_DEBUG    
    uint64_t start_time = 0;
    if (log05Hz) start_time = AP_HAL::micros64();
#endif    
    _ty.add(target_y);

    // prediction
    const T predicted_y = _model->predict_one_step(transform_u(_current_u, throttle), T());    
    //GPC_DEBUG_LOG_1HZ("GPC py=%.2f", predicted_y);

    // adjust model to current measurement
    const T smoothed_y = _model->measured_y(y);

    // skip if there is not enough past data for the model
    if (!_model->ready()) {
        return T();
    }

    //if (c % 10 != 0) return _current_u;

    // prediction error 
    const T dk = smoothed_y - predicted_y;

    // free trajectory prediction
    MatrixNxM<T, N, 1> y0k;    
    _y0_model->reset_to(*_model);
    for (uint8_t i=0;i<N;i++) {
        y0k[i][0] = _y0_model->predict_one_step(transform_u(_current_u, throttle), 0.0f) + dk;
    }

    // GPC_DEBUG_LOG_05HZ("----Y0-----", NULL);
    // for (uint8_t i=0;i<N;i++) {
        
    //     GPC_DEBUG_LOG_05HZ("%2u: %.3f", i, y0k[i][0]);
    // }
    // GPC_DEBUG_LOG_05HZ("-----------", NULL);

    // target y trajectory is smooth, so it can be ~guessed 
    // non-constant target trajectory gives better u prediction
    guess_target_y();

    // calculate GPC
    MatrixNxM<T, N, 1> y_diff = _target_y - y0k;
    MatrixNxM<T, Nu, 1> duk = K * y_diff;

    // constraints
    T next_u = _current_u + constrain_float(duk[0][0], -GPC_MAX_duk, GPC_MAX_duk);
#ifdef GPC_DEBUG
    //GPC_DEBUG_LOG("%u GPC y=%.2f ty=%.2f dk=%.2f u=%.2f", c, y, target_y, dk, next_u);
#else
    if (log05Hz) GPC_DEBUG_LOG("GPC y=%.2f ty=%.2f dk=%.2f u=%.2f", y, target_y, dk, next_u);
#endif
    if (next_u > _gpc_params.max_u) next_u = _gpc_params.max_u;
    if (next_u < _gpc_params.min_u) next_u = _gpc_params.min_u;

    _current_u = next_u;

#ifndef GPC_DEBUG
    if (log05Hz) {
        start_time = AP_HAL::micros64() - start_time;
        GPC_DEBUG_LOG("GPC time: %llu us", start_time);
    }
#endif

    return _current_u;
}

template<typename T, uint8_t N, uint8_t Nu>
void GPC_Controller<T, N, Nu>::guess_target_y() 
{
    if (!_ty.ready()) {
        const T last_ty = _ty.get_last_item();
        for (uint8_t i=0;i<N;i++) {
            _target_y[i][0] = last_ty;
        }

        return;
    }

    T tys[3];
    _ty.get_last_n_items(tys, 3);
    
    const T last_ty = tys[2];
    const T dty1 = tys[1] - tys[0];
    const T dty2 = tys[2] - tys[1];
    const T ddty = dty2 - dty1;

    T a_limit1 = T();
    T a_limit2 = T();
    if (sgn(dty2) == sgn(ddty)) {
        a_limit1 = dty2;
        a_limit2 = 1.5f * dty2;
    } else {
        a_limit1 = dty2;
        a_limit2 = -1.5f * dty2;
    }

    const T a_min = std::min(a_limit1, a_limit2);
    const T a_max = std::max(a_limit1, a_limit2);

    _target_y[0][0] = last_ty;
    T a = dty2;
    T aa = ddty;
    for (uint8_t i=1;i<N;i++) {
        _target_y[i][0] = _target_y[i-1][0] + a;
        a = constrain_float(a + aa, a_min, a_max);
    }
}

