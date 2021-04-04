#include "AC_AttitudeControl_GPC.h"

#ifndef GPC_DEBUG

AC_AttitudeControl_GPC::AC_AttitudeControl_GPC(GCS &gcs):   
    DebugLogger(gcs)
{
    GPC_Params<float> params;
    params.lambda = GPC_lambda;
    params.min_u = -0.41f;
    params.max_u = 0.41f;

    _gpc_pitch_controller = new GPC_Controller<float, GPC_N, GPC_Nu>(
        params, 
        new DifferenceEquationModel<float>(GPC_LINEAR_MODEL_AN, GPC_LINEAR_MODEL_BN, GPC_LINEAR_MODEL_UDELAY, this), 
        new DifferenceEquationModel<float>(GPC_LINEAR_MODEL_AN, GPC_LINEAR_MODEL_BN, GPC_LINEAR_MODEL_UDELAY, this), 
        this
    );

    _gpc_pitch_controller->initialize();

    _switching_controller = new SwitchingController(this);
    _switching_controller->initialize();
}

AC_AttitudeControl_GPC::~AC_AttitudeControl_GPC() 
{
    delete _gpc_pitch_controller;
}

float AC_AttitudeControl_GPC::get_pitch() 
{
    return _gpc_pitch_controller->get_current_u();
    //return _switching_controller->get_current_u();
}

void AC_AttitudeControl_GPC::set_lambda(const float lambda) 
{
    _gpc_pitch_controller->set_lambda(lambda);
}

void AC_AttitudeControl_GPC::rate_controller_run(const float roll, const float target_roll, const float pitch, const float target_pitch, const float yaw, const float target_yaw, const float throttle) 
{
    GPC_DEBUG_LOG_INIT;

    // if (c % 2 == 0) {
    //    _gpc_pitch_controller->run_step(pitch, target_pitch, throttle);
    // }

    _gpc_pitch_controller->run_step(pitch, target_pitch, throttle);
    // _switching_controller->run_step(pitch, target_pitch, throttle);
}

#endif

SwitchingController::SwitchingController(DebugLogger *logger) :
    _logger(logger),
    _current_u(0.0f),
    _y(3 + GPC_GAUSSIAN_SMOOTHING_WINDOW)
{
    
}

void SwitchingController::initialize() 
{
    
}

float SwitchingController::run_step(const float &y, const float &target_y, const float &throttle) 
{
#define SWITCHING_TILT_ANGLE            8.0f
#define SWITCHING_MAX_SPEED             0.15f
#define SWITCHING_SIGNAL                0.5f

    _y.add(y);

    if (!_y.ready()) {
        _current_u = 0.0f;
        return _current_u;
    }

    float cy[2];
    read_and_smooth(&_y, cy, 2, defines::gpc::gpc_gaussian_smoothing_weights);
    const float dy = cy[1] - cy[0];
    const float last_y = cy[1];

    if (last_y > SWITCHING_TILT_ANGLE || dy > SWITCHING_MAX_SPEED) {
        _current_u = -SWITCHING_SIGNAL;
    } else if (last_y < -SWITCHING_TILT_ANGLE || dy < -SWITCHING_MAX_SPEED) {
        _current_u = SWITCHING_SIGNAL;
    } else {
        _current_u = 0.0f;
    }

    return _current_u;
}

void SwitchingController::read_and_smooth(const CircularBuffer<float> *c, float buff[], const size_t n, const float gaussian_weights[]) 
{
    for (size_t i=0; i<n; i++) {
        buff[i] = 0.0f;
    }

    float raw_data[n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1];
    c->get_last_n_items(raw_data, n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1);
    for (size_t k=0, i=GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i<n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i++, k++) {
        for (size_t j=0; j<GPC_GAUSSIAN_SMOOTHING_WINDOW; j++) {
            buff[k] += gaussian_weights[j] * raw_data[i-j];
        }
    }
}


