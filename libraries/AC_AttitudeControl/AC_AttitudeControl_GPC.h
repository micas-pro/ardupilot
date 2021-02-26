#pragma once

#include "AC_AttitudeControl.h"

class AC_AttitudeControl_GPC : virtual public AC_AttitudeControl
{
public:
    AC_AttitudeControl_GPC(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_GPC() {}

    float get_pitch();

    virtual AC_PID& get_rate_roll_pid() override;
    virtual AC_PID& get_rate_pitch_pid() override;
    virtual AC_PID& get_rate_yaw_pid() override;
    virtual void rate_controller_run() override;
    virtual void update_althold_lean_angle_max(float throttle_in) override;
    virtual void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

private:
    AC_PID *nullpid;

    struct GPC_Params
    {
        float lambda;
        uint8_t n;
        uint8_t nu;
    } _gpc_params;
};

