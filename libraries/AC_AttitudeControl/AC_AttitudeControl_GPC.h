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
        _logger(logger)
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
    DebugLogger *_logger;
    GPC_Params<T> _gpc_params;
    LinearModelBase<T> *_model, *_y0_model;
    T _current_u;
    MatrixNxM<T, Nu, N> K;
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
    void rate_controller_run(const float roll, const float target_roll, const float pitch, const float target_pitch, const float yaw, const float target_yaw);

private:    
    GPC_Controller<float, GPC_N, GPC_Nu> *_gpc_pitch_controller;
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
}

template<typename T, uint8_t N, uint8_t Nu>
void GPC_Controller<T, N, Nu>::set_lambda(const T &lambda) 
{
    _gpc_params.lambda = lambda;
}

template<typename T, uint8_t N, uint8_t Nu>
const T GPC_Controller<T, N, Nu>::run_step(const T &y, const T &target_y) 
{    
    GPC_DEBUG_LOG_INIT;

    const bool log05Hz = _GPC_DEBUG_LOG_05HZ;
#ifndef GPC_DEBUG    
    uint64_t start_time = 0;
    if (log05Hz) start_time = AP_HAL::micros64();
#endif

    // prediction
    const T predicted_y = _model->predict_one_step(_current_u, T());    
    //GPC_DEBUG_LOG_1HZ("GPC py=%.2f", predicted_y);

    // adjust model to current measurement
    _model->measured_y(y);

    // skip if there is not enough past data for the model
    if (!_model->ready()) {
        return T();
    }

    //if (c % 10 != 0) return _current_u;

    // prediction error
    const T dk = y - predicted_y;

    // free trajectory prediction
    MatrixNxM<T, N, 1> y0k;
    MatrixNxM<T, N, 1> y_target(target_y);
    _y0_model->reset_to(*_model);
    for (uint8_t i=0;i<N;i++) {
        y0k[i][0] = _y0_model->predict_one_step(_current_u, dk);
    }

    // GPC_DEBUG_LOG_05HZ("----Y0-----", NULL);
    // for (uint8_t i=0;i<N;i++) {
        
    //     GPC_DEBUG_LOG_05HZ("%2u: %.3f", i, y0k[i][0]);
    // }
    // GPC_DEBUG_LOG_05HZ("-----------", NULL);

    // calculate GPC
    y_target -= y0k;
    MatrixNxM<T, Nu, 1> duk = K * y_target;

    // constraints
    T next_u = _current_u + constrain_float(duk[0][0], -GPC_MAX_duk, GPC_MAX_duk);
#ifdef GPC_DEBUG
    GPC_DEBUG_LOG("%u GPC y=%.2f ty=%.2f dk=%.2f u=%.2f", c, y, target_y, dk, next_u);
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

