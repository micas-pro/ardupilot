#pragma once

#include "AC_AttitudeControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixNxM.h>
#include <deque>

#include "AC_AttitudeControl_GPC_defines.h"

using namespace std;

#define GPC_N                   40
#define GPC_Nu                  20

template <typename T = float>
struct GPC_Params
{
    T lambda;
    T min_u;
    T max_u;
};

template <typename T>
class LinearModel
{
public:
    LinearModel(const uint8_t y_steps, const uint8_t u_steps):
        _y_steps(y_steps),
        _u_steps(u_steps)
    {
        _y.resize(y_steps);
        _u.resize(u_steps);
    }

    virtual ~LinearModel() {}

    virtual T predict_one_step(const T &u, const T &dk) {
        return T();
    }

private:
    const T normalize_u(const T &u) {

    }

    const T normalize_y(const T &y) {

    }

    const T denormalize_y(const T &y) {
        
    }

    uint8_t _y_steps, _u_steps;
    deque<T> _y;
    deque<T> _u;
};

template <typename T, uint8_t N, uint8_t Nu>
class GPC_Controller
{
public: 
    GPC_Controller(const GPC_Params<T> gpc_params, LinearModel<T> &model, LinearModel<T> &y0_model)
        : _gpc_params(gpc_params),
        _model(model),
        _y0_model(y0_model)        
    {
        _current_u = T();
    }

    virtual ~GPC_Controller() {}

    void initialize() 
    {

    }

    const T run_step(const T &y, const T &target_y) 
    {
        // prediction
        const T predicted_y = _model.predict_one_step(_current_u);

        // prediction error
        const T dk = y - predicted_y;

        // free trajectory prediction
        MatrixNxM<T, N, 1> y0k;
        MatrixNxM<T, N, 1> y_target(target_y);
        for (uint8_t i=0;i<N;i++) {
            y0k[i] = _y0_model.predict_one_step(_current_u, dk);
        }

        // calculate GPC
        y_target -= y0k;
        MatrixNxM<T, Nu, 1> duk = K * y_target;

        // constraints
        T next_u = _current_u + duk[0, 0];
        if (next_u > _gpc_params.max_u) next_u = _gpc_params.max_u;
        if (next_u < _gpc_params.min_u) next_u = _gpc_params.min_u;

        _current_u = next_u;

        return _current_u;
    }

private:
    GPC_Params<T> _gpc_params;
    LinearModel<T> &_model, &_y0_model;
    T _current_u;
    MatrixNxM<float, 2, 3> K;
};

class AC_AttitudeControl_GPC
{
public:
    AC_AttitudeControl_GPC(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_GPC();

    float get_pitch();
    void set_lambda(const float lambda);
    void set_rate_target_ang_vel(const Vector3f &x) { _rate_target_ang_vel = x; };
    void rate_controller_run();

private:
    // Intersampling period in seconds
    float               _dt;

    // This represents a 321-intrinsic rotation in NED frame to the target (setpoint)
    // attitude used in the attitude controller, in radians.
    Vector3f            _attitude_target_euler_angle;

    // This represents the angular velocity in radians per second in the body frame, used in the angular
    // velocity controller.
    Vector3f            _rate_target_ang_vel;

    // References to external libraries
    const AP_AHRS_View&  _ahrs;
    const AP_Vehicle::MultiCopter &_aparm;
    AP_Motors&          _motors;

    GPC_Controller<float, GPC_N, GPC_Nu> *gpc_pitch;
};

