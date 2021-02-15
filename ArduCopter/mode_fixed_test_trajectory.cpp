#include "Copter.h"

#if MODE_FIXEDTESTTRAJECTORY_ENABLED == ENABLED

const int16_t ModeFixedTestTrajectory::fixed_trajectory[] = {
    #include "mode_fixed_test_trajectory_data.txt"
};

size_t ModeFixedTestTrajectory::get_trajectory_length()
{
    return (sizeof(fixed_trajectory)/sizeof(*(fixed_trajectory)));
}

bool ModeFixedTestTrajectory::init(bool ignore_checks)
{
    reset();

    debug_msg("ModeFixedTestTrajectory init: %u steps", (uint32_t)get_trajectory_length());
    
    return true;
}

void ModeFixedTestTrajectory::debug_msg(const char *fmt, ...)
{
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), "%s", fmt);
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(MAV_SEVERITY_DEBUG, taggedfmt, arg_list);
    va_end(arg_list);
}

float ModeFixedTestTrajectory::get_current_throttle()
{
    if (_current_time <= 0.0f) {
        return 0.0f;
    } else if (_current_time <= MODE_FIXED_TRAJECTORY_THROTTLE_IDLE_TIME_SECONDS + MODE_FIXED_TRAJECTORY_THROTTLE_START_TIME_SECONDS) {
        return constrain_float(linear_interpolate(
            0.0f, 
            throttle_hover(), 
            _current_time, 
            MODE_FIXED_TRAJECTORY_THROTTLE_IDLE_TIME_SECONDS, 
            MODE_FIXED_TRAJECTORY_THROTTLE_IDLE_TIME_SECONDS + MODE_FIXED_TRAJECTORY_THROTTLE_START_TIME_SECONDS), 
        0.0f, 1.0f);
    } else {
        return throttle_hover();
    }
}

float ModeFixedTestTrajectory::get_current_trajectory_value()
{
    return (float)fixed_trajectory[_current_step];
}

void ModeFixedTestTrajectory::reset()
{
    _current_step = 0;
    _current_time = 0.0f;
    _started = false;
}

bool ModeFixedTestTrajectory::next_step()
{
    if(_current_step >= get_trajectory_length() - 1) {
        _current_step = get_trajectory_length() - 1;
        return false;
    } else {
        _current_step++;
        _current_time += MODE_FIXED_TRAJECTORY_TIME_STEP_SECONDS;
        return true;
    }
}

bool ModeFixedTestTrajectory::check_started()
{
    // TODO add arm/disarm check!
    if (_started) {
        if (copter.ap.throttle_zero) {
            reset();
        }
    } else {
        if (!copter.ap.throttle_zero) {
            reset();
            _started = true;
        }
    }

    return _started;
}

void ModeFixedTestTrajectory::run()
{
    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
            // Motors Stopped
            attitude_control->set_yaw_target_to_current_heading();
            attitude_control->reset_rate_controller_I_terms();
            break;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // Landed
            attitude_control->set_yaw_target_to_current_heading();
            attitude_control->reset_rate_controller_I_terms();

            attitude_control->set_throttle_out(0.0f,
                                        true,
                                        g.throttle_filt);
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            // clear landing flag above zero throttle
            if (!motors->limit.throttle_lower) {
                set_land_complete(false);
            }
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // do nothing
            break;
    }

    float throttle = 0.0f;
    float target_pitch = 0.0f;
    if (check_started() && next_step()) {
        throttle = get_current_throttle();
        target_pitch = get_current_trajectory_value();
    } 

    static uint32_t cnt = 0;
    if (cnt++ % 200 == 0) {
        debug_msg("throttle: %.2f, pitch: %.2f", throttle, target_pitch);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, target_pitch, 0.0f);

    // output pilot's throttle
    attitude_control->set_throttle_out(throttle,
                                       true,
                                       g.throttle_filt);
}
#endif
