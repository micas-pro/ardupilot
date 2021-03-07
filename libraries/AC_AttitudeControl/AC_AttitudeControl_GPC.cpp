#include "AC_AttitudeControl_GPC.h"


AC_AttitudeControl_GPC::AC_AttitudeControl_GPC(GCS &gcs):   
    _gcs(gcs)
{
    GPC_Params<float> params;
    params.lambda = 0.5f;
    params.min_u = -0.6f;
    params.max_u = 0.6f;

    _gpc_pitch_controller = new GPC_Controller<float, GPC_N, GPC_Nu>(
        params, 
        new LinearModel<float>(GPC_LINEAR_MODEL_A, GPC_LINEAR_MODEL_B), 
        new LinearModel<float>(GPC_LINEAR_MODEL_A, GPC_LINEAR_MODEL_B),
        gcs
    );

    _gpc_pitch_controller->initialize();
}

AC_AttitudeControl_GPC::~AC_AttitudeControl_GPC() 
{
    delete _gpc_pitch_controller;
}

void AC_AttitudeControl_GPC::debug_msg(const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    _gcs.send_textv(MAV_SEVERITY_DEBUG, fmt, arg_list);
    va_end(arg_list);
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

float normalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : (x - x_shift) / x_d;
}

float normalize_dy(const float dy, const uint8_t n) 
{
    return normalize(dy, defines::gpc::normalization_dy[n][0], defines::gpc::normalization_dy[n][1]);
}

float normalize_y(const float y) 
{
    return normalize_dy(y, 0);
}

float normalize_u(const float u) 
{
    return normalize(u, defines::gpc::normalization_u[0], defines::gpc::normalization_u[1]);
}

float denormalize(const float x, const float x_shift, const float x_d) 
{
    return is_zero(x_d) ? x : x * x_d + x_shift;
}

float denormalize_dy1(const float dy1) 
{
    return denormalize(dy1, defines::gpc::normalization_dy[1][0], defines::gpc::normalization_dy[1][1]);
}

void calculate_diffs(float *diffs, const size_t &n) 
{
    for (size_t i=0; i<n; i++) {
        diffs[i] = diffs[i+1] - diffs[i];
    }
}

template<typename T>
void LinearModel<T>::load_weights(const T w[]) 
{
    for (int i=0;i<GPC_LINEAR_MODEL_WEIGHTS;i++) {
        _weights[i][0] = w[i];
    }
}

template<typename T>
T LinearModel<T>::predict_one_step(const T &u, const T &dk) 
{
    _u->add(u);

    if (!ready()) {
        _y->add(T());
        return T() + dk;
    }

    T py = predict_one_step() + dk;
    _y->add(py);

    return py;
}

template<typename T>
void LinearModel<T>::measured_y(const T &y) 
{
    _y->replace_last(y);
}

template<typename T>
void LinearModel<T>::reset_to(const LinearModel &model) 
{
    _u->clear();
    _y->clear();

    T u_other[_u_steps];
    model._u->write_to(u_other);
    for (uint8_t i=0;i<_u_steps;i++) {
        _u->add(u_other[i]);
    }

    T y_other[_y_steps];
    model._y->write_to(y_other);
    for (uint8_t i=0;i<_y_steps;i++) {
        _y->add(y_other[i]);
    }
}

template<typename T>
bool LinearModel<T>::ready() 
{
    return _y->ready() && _u->ready();
}

template<typename T>
T LinearModel<T>::predict_one_step() 
{    
    if (!ready()) {
        return T();
    }

    uint8_t i = 0;

    // write normalized u
    T* u = new T[_u_steps];
    _u->write_to(u);
    for (;i<_u_steps;i++) {
        _state[0][i] = normalize_u(u[i]);
    }

    delete [] u;

    // get past y values
    T* y = new T[_y_steps];
    T* diffs = new T[_y_steps];
    _y->write_to(y);
    for (uint8_t j=0;j<_y_steps;j++) {
        diffs[j] = y[j];
    }    

    // write normalized y
    _state[0][i++] = normalize_y(y[_y_steps - 1]);

    // write normalized dy
    for (uint8_t j=1; j<=_y_steps-1; j++) {
        calculate_diffs(diffs, _y_steps - j);
        _state[0][i++] = normalize_dy(diffs[_y_steps - 1 - j], j);
    }

    delete [] diffs;

    // A[1xN] * W[Nx1] = Y[1x1]  <-- model predicts dy, not y!
    MatrixNxM<T, 1, 1> pdy = _state * _weights;

    // denormalize dy and calculate y
    T last_y = y[_y_steps - 1];
    T denormalized_dy = denormalize_dy1(pdy[0][0]);
    delete [] y;    
    T py = last_y + denormalized_dy;
    _y->add(py);

    return last_y + denormalized_dy;
}

template<typename T, uint8_t N, uint8_t Nu>
void GPC_Controller<T, N, Nu>::initialize() 
{
    _model->load_weights(defines::gpc::linear_model_w);
    _y0_model->load_weights(defines::gpc::linear_model_w);
    K = MatrixNxM<T, Nu, N>((const float**)defines::gpc::gpc_K);
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

    // prediction
    const T predicted_y = _model->predict_one_step(_current_u, T());    
    GPC_DEBUG_LOG_1HZ("GPC py=%.2f", predicted_y);

    // adjust model to current measurement
    _model->measured_y(y);

    // skip if there is not enough past data for the model
    if (!_model->ready()) {
        return T();
    }

    // prediction error
    const T dk = y - predicted_y;

    // free trajectory prediction
    MatrixNxM<T, N, 1> y0k;
    MatrixNxM<T, N, 1> y_target(target_y);
    _y0_model->reset_to(*_model);
    for (uint8_t i=0;i<N;i++) {
        y0k[i][0] = _y0_model->predict_one_step(_current_u, dk);
    }

    // calculate GPC
    y_target -= y0k;
    MatrixNxM<T, Nu, 1> duk = K * y_target;

    // constraints
    T next_u = _current_u + duk[0][0];
    GPC_DEBUG_LOG_1HZ("GPC y=%.2f ty=%.2f dk=%.2f u=%.2f", y, target_y, dk, next_u);
    if (next_u > _gpc_params.max_u) next_u = _gpc_params.max_u;
    if (next_u < _gpc_params.min_u) next_u = _gpc_params.min_u;

    _current_u = next_u;

    return _current_u;
}
