#include "AC_AttitudeControl_GPC.h"


AC_AttitudeControl_GPC::AC_AttitudeControl_GPC(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt):   
    _dt(dt),
    _ahrs(ahrs),
    _aparm(aparm),
    _motors(motors)
{
    GPC_Params<float> params;
    params.lambda = 0.5f;
    params.min_u = -0.6f;
    params.max_u = 0.6f;

    LinearModel<float> model(40, 10);
    LinearModel<float> y0_model(40, 10);

    gpc_pitch = new GPC_Controller<float, GPC_N, GPC_Nu>(params, model, y0_model);
}

AC_AttitudeControl_GPC::~AC_AttitudeControl_GPC() 
{
    delete gpc_pitch;
}

float AC_AttitudeControl_GPC::get_pitch() 
{
    return 0.0f;
}

void AC_AttitudeControl_GPC::set_lambda(const float lambda) 
{
    //_gpc_params.lambda = lambda;
}

void AC_AttitudeControl_GPC::rate_controller_run() 
{
    
}

