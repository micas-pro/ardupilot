#include "AC_AttitudeControl_GPC.h"


AC_AttitudeControl_GPC::AC_AttitudeControl_GPC(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) 
    : AC_AttitudeControl(ahrs, aparm, motors, dt)
{
    _gpc_params.lambda = 0.5f;
    _gpc_params.n = 30;
    _gpc_params.nu = 10;

    nullpid = new AC_PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

float AC_AttitudeControl_GPC::get_pitch() 
{
    return 0.0f;
}

void AC_AttitudeControl_GPC::set_lambda(const float lambda) 
{
    _gpc_params.lambda = lambda;
}

AC_PID& AC_AttitudeControl_GPC::get_rate_roll_pid() 
{
    return *nullpid;
}

AC_PID& AC_AttitudeControl_GPC::get_rate_pitch_pid() 
{
    return *nullpid;
}

AC_PID& AC_AttitudeControl_GPC::get_rate_yaw_pid() 
{
    return *nullpid;
}

void AC_AttitudeControl_GPC::rate_controller_run() 
{
    
}

void AC_AttitudeControl_GPC::update_althold_lean_angle_max(float throttle_in) 
{
    
}

void AC_AttitudeControl_GPC::set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) 
{
    
}
