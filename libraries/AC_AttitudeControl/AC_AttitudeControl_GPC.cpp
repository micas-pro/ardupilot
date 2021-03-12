#include "AC_AttitudeControl_GPC.h"


AC_AttitudeControl_GPC::AC_AttitudeControl_GPC(GCS &gcs):   
    DebugLogger(gcs)
{
    GPC_Params<float> params;
    params.lambda = GPC_lambda;
    params.min_u = -0.7f;
    params.max_u = 0.7f;

    _gpc_pitch_controller = new GPC_Controller<float, GPC_N, GPC_Nu>(
        params, 
        new LinearModelNoYuDiff<float>(GPC_LINEAR_MODEL_DY, GPC_LINEAR_MODEL_U, GPC_GAUSSIAN_SMOOTHING_WINDOW, this), 
        new LinearModelNoYuDiff<float>(GPC_LINEAR_MODEL_DY, GPC_LINEAR_MODEL_U, GPC_GAUSSIAN_SMOOTHING_WINDOW, this),
        this
    );

    _gpc_pitch_controller->initialize();
}

AC_AttitudeControl_GPC::~AC_AttitudeControl_GPC() 
{
    delete _gpc_pitch_controller;
}

float AC_AttitudeControl_GPC::get_pitch() 
{
    return _gpc_pitch_controller->get_current_u();
}

void AC_AttitudeControl_GPC::set_lambda(const float lambda) 
{
    _gpc_pitch_controller->set_lambda(lambda);
}

void AC_AttitudeControl_GPC::rate_controller_run(const float roll, const float target_roll, const float pitch, const float target_pitch, const float yaw, const float target_yaw) 
{
    _gpc_pitch_controller->run_step(pitch, target_pitch);
}


