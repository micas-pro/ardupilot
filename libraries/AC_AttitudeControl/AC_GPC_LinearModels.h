#pragma once

#include "AC_GPC_Helpers.h"
//#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixNxM.h>
#include <AP_Math/circular_buffer.h>

#define GPC_LINEAR_MODEL_MAX_DY_ABS             0.55f
#define GPC_LINEAR_MODEL_Y                      0
#define GPC_LINEAR_MODEL_DY                     4
#define GPC_LINEAR_MODEL_U                      0
#define GPC_LINEAR_MODEL_D1U                    0
#define GPC_LINEAR_MODEL_U_SUM_WINDOW           30
#if GPC_LINEAR_MODEL_U_SUM_WINDOW > 0
    #define GPC_LINEAR_MODEL_U_SUM_USED 1
#else
    #define GPC_LINEAR_MODEL_U_SUM_USED 0
#endif
//#define GPC_LINEAR_MODEL_WEIGHTS                (GPC_LINEAR_MODEL_Y + GPC_LINEAR_MODEL_DY + GPC_LINEAR_MODEL_U + GPC_LINEAR_MODEL_D1U)
#define GPC_LINEAR_MODEL_WEIGHTS                (GPC_LINEAR_MODEL_Y + GPC_LINEAR_MODEL_DY + GPC_LINEAR_MODEL_U + GPC_LINEAR_MODEL_D1U + GPC_LINEAR_MODEL_U_SUM_WINDOW + GPC_LINEAR_MODEL_U_SUM_USED)
#define GPC_LINEAR_MODEL_AN                     1
#define GPC_LINEAR_MODEL_BN                     1
#define GPC_Nu                                  5
#define GPC_N                                   40
#define GPC_lambda                              1.000f
#define GPC_MAX_duk                             0.4f
#define GPC_GAUSSIAN_SMOOTHING_WINDOW           10

namespace defines {

namespace gpc {

const float normalization_dy[6][2] = {
    { 0.0f, 0.0f },
    { 0.0f, 0.0f },
    { 0.0f, 0.0f },
    { 0.0f, 0.0f },
    { 0.0f, 0.0f },
    { 0.0f, 0.0f },
};

const float normalization_u[2] = { 0.0f, 0.0f };
const float normalization_d1u[2] = { 0.0f, 0.0f };

// const float normalization_dy[6][2] = {
//     { 0.000000000000f, 80.000000000000f },
//     { 0.000000000000f, 0.030000000000f },
//     { 0.000000000000f, 0.000600000000f },
//     { 0.000000000000f, 0.000600000000f },
//     { 0.000000000000f, 0.000600000000f },
//     { 0.000000000000f, 0.000600000000f }
// };

// const float normalization_u[2] = { 0.000000000000f, 1.200000000000f };
// const float normalization_d1u[2] = { 0.000000000000f, 0.056000000000f };

// const float linear_model_w[GPC_LINEAR_MODEL_WEIGHTS] = { 
//     2.555961847305f, 
//     -3.606574058533f, 
//     0.749199748039f, 
//     -0.836554765701f, 
//     -0.057546745986f, 
//     -3.632860422134f, 
//     0.867749750614f, 
//     2.809866189957f, 
//     -3.319396018982f, 
//     -3.169429779053f, 
//     0.895687818527f, 
//     4.529780864716f, 
//     -3.645090103149f, 
//     -7.897997379303f, 
//     6.779769897461f, 
//     2.711076736450f, 
//     -3.404711484909f, 
//     -4.076398849487f, 
//     2.539579153061f, 
//     5.552387237549f, 
//     -13.000236511230f, 
//     18.222537994385f, 
//     -11.215386390686f, 
//     -4.081554412842f, 
//     -0.938470363617f, 
//     15.093239784241f, 
//     -8.436522483826f, 
//     -17.637632369995f, 
//     27.168569564819f, 
//     -13.987618446350f, 
//     0.525150835514f, 
//     -0.597688198090f, 
//     -0.120367549360f, 
//     0.502316474915f, 
//     -0.083815030754f
// };

const float linear_model_w[GPC_LINEAR_MODEL_AN + GPC_LINEAR_MODEL_BN] = { 0.999587799711154f, 0.021126315934115f };

// N = 40, Nu = 5, lambda = 1
const float gpc_K[GPC_Nu][GPC_N] = {
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.010872044177f, 0.025613507386f, 0.040053435694f, 0.052428714013f, 0.062971670720f, 0.071683061099f, 0.078563640121f, 0.083614162448f, 0.086835382429f, 0.088228054103f, 0.087792931199f, 0.085530767134f, 0.081442315015f, 0.075528327638f, 0.067789557490f, 0.058226756746f, 0.046840677272f, 0.033632070625f, 0.018601688050f, 0.001750280485f, -0.016921401444f, -0.037412607419f, -0.059722587432f, -0.083850591784f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.006998143684f, -0.005416947824f, 0.000490530480f, 0.007627770004f, 0.013904415309f, 0.019320821132f, 0.023877342066f, 0.027574332554f, 0.030412146896f, 0.032391139243f, 0.033511663603f, 0.033774073836f, 0.033178723657f, 0.031725966636f, 0.029416156196f, 0.026249645614f, 0.022226788022f, 0.017347936407f, 0.011613443610f, 0.005023662326f, -0.002421054894f, -0.010720355646f, -0.019873887669f, -0.029881298851f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.004169358959f, -0.016755878961f, -0.020886975099f, -0.020760363078f, -0.020445860455f, -0.019943544677f, -0.019253493161f, -0.018375783292f, -0.017310492423f, -0.016057697874f, -0.014617476935f, -0.012989906863f, -0.011175064883f, -0.009173028189f, -0.006983873942f, -0.004607679273f, -0.002044521279f, 0.000705522972f, 0.003642376447f, 0.006765962143f, 0.010076203089f, 0.013573022345f, 0.017256343004f, 0.021126088189f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.001763239379f, -0.008383729259f, -0.024053697856f, -0.032705507381f, -0.040044970543f, -0.046072628291f, -0.050789021351f, -0.054194690228f, -0.056290175202f, -0.057076016332f, -0.056552753452f, -0.054720926177f, -0.051581073896f, -0.047133735778f, -0.041379450770f, -0.034318757594f, -0.025952194753f, -0.016280300526f, -0.005303612972f, 0.006977330074f, 0.020561990996f, 0.035449832402f, 0.051640317120f, 0.069132908202f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000231477327f, -0.001395511965f, -0.008948264341f, -0.028127257022f, -0.044796283658f, -0.058956378859f, -0.070608576805f, -0.079753911255f, -0.086393415537f, -0.090528122555f, -0.092159064787f, -0.091287274287f, -0.087913782680f, -0.082039621170f, -0.073665820532f, -0.062793411120f, -0.049423422859f, -0.033556885254f, -0.015194827381f, 0.005661722104f, 0.029011734971f, 0.054854183415f, 0.083188040051f, 0.114012277923f }
};

const float gpc_gaussian_smoothing_weights[GPC_GAUSSIAN_SMOOTHING_WINDOW] = {
    0.182291054022883f,   0.192704930829177f,   0.182291054022883f,   0.154306045841418f,   0.116881448825698f,   0.079223365505212f, 0.048051400144531f,   0.026079776394858f,   0.012666211560552f,   0.005504712852788f,
};

} // namespace gpc

}  // namespace defines

// utils --------------------------------------------------------------------------------

float normalize_dy(const float dy, const uint8_t n);
float normalize_y(const float y);
float normalize_u(const float u);
float normalize_d1u(const float d1u);

// calculate in-place n diffs, so the array must actually contain n+1 elements
void calculate_diffs(float *diffs, const size_t &n);

float denormalize_d1y(const float d1y);

// classes ------------------------------------------------------------------------------

template <typename T>
class LinearModelBase;

template <typename T>
class LinearModelBase
{
public:
    LinearModelBase(const uint8_t _y_steps, const uint8_t _u_steps, DebugLogger *logger):
        y_steps(_y_steps),
        u_steps(_u_steps),
        _logger(logger)
    {
        y = new CircularBuffer<T>(_y_steps);
        u = new CircularBuffer<T>(_u_steps);
    }

    virtual ~LinearModelBase() 
    {
        delete y;
        delete u;
    }

    virtual void load_weights(const T w[]) = 0;
    T predict_one_step(const T &_u, const T &dk);
    void measured_y(const T &_y);
    void reset_to(const LinearModelBase &model);
    bool ready();

protected:

    virtual void read_u(T t_array[], const size_t n); // take last n of u elements
    virtual void read_y(T t_array[], const size_t n); // take last n of y elements 
    T check_and_denormalize_dy(const T &dy);   

    virtual T predict_next_dy() = 0;

    DebugLogger *_logger;    
    uint8_t y_steps, u_steps;
    CircularBuffer<T> *y;
    CircularBuffer<T> *u;
};

template <typename T>
class DifferenceEquationModel : virtual public LinearModelBase<T>
{
public:
    DifferenceEquationModel(const uint8_t _an, const uint8_t _bn, DebugLogger *logger): 
        LinearModelBase<T>(_an + GPC_GAUSSIAN_SMOOTHING_WINDOW, _bn, logger),
        an(_an),
        bn(_bn)
    {
        a = new T[_an];
        b = new T[_bn];
    }

    virtual ~DifferenceEquationModel() 
    {
        delete [] a;
        delete [] b;
    }

    virtual void load_weights(const T w[]) override;

protected:

    virtual void read_y(T t_array[], const size_t n) override; // take last n of y elements, gaussian smoothed
    virtual T predict_next_dy() override;
    void read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]);

    uint8_t an, bn;

    // y[k+1] = a[0]*y(k) + a[1]*y(k-1) + ... + a[an-1]*y(k-an+1) + b[0]*u(k) + b[1]*u(k-1) + ... + b[bn-1]*u(k-bn+1)
    T *a; 
    T *b;    
};

template <typename T>
class NeuralLinearModelBase : virtual public LinearModelBase<T>
{
public:
    NeuralLinearModelBase(const uint8_t _y_steps, const uint8_t _u_steps, DebugLogger *logger): LinearModelBase<T>(_y_steps, _u_steps, logger) 
    {}

    virtual ~NeuralLinearModelBase() {}

    virtual void load_weights(const T w[]) override;

protected:

    void fill_u(const size_t pos_in_state, const uint8_t n);
    T fill_u_and_get_u_sum(const size_t pos_in_state, const uint8_t n);
    void fill_y(const size_t pos_in_state);
    void fill_dy_chain(const size_t pos_in_state, const uint8_t n);
    void fill_d1u(const size_t pos_in_state, const uint8_t n);
    virtual T predict_next_dy() override;

    virtual void fill_current_state() = 0;

    MatrixNxM<T, GPC_LINEAR_MODEL_WEIGHTS, 1> weights;
    MatrixNxM<T, 1, GPC_LINEAR_MODEL_WEIGHTS> state;
};

// ----------------------------------------------------------------------------------

template<typename T>
class LinearModel : virtual public NeuralLinearModelBase<T>
{
public:
    LinearModel(const uint8_t _y_steps, const uint8_t _u_steps, DebugLogger *logger): NeuralLinearModelBase<T>(_y_steps, _u_steps, logger)
    {}

    virtual ~LinearModel() {}

protected:
    void fill_current_state() override;
};

// ----------------------------------------------------------------------------------

template<typename T>
class LinearModelNoYuDiff : virtual public NeuralLinearModelBase<T>
{
public:
    LinearModelNoYuDiff(const uint8_t _y_steps, const uint8_t _u_steps, const uint8_t _gaussian_smoothing_window, DebugLogger *logger): 
        NeuralLinearModelBase<T>(_y_steps + _gaussian_smoothing_window + 1, _u_steps + _gaussian_smoothing_window, logger),
        LinearModelBase<T>(_y_steps, _u_steps, logger),
        gaussian_smoothing_window(_gaussian_smoothing_window),
        real_y_steps(_y_steps),
        real_u_steps(_u_steps)
    {        
    }

    virtual ~LinearModelNoYuDiff() {}

protected:
    void read_u(T t_array[], const size_t n) override; // take last n of u elements, gaussian smoothed
    void read_y(T t_array[], const size_t n) override; // take last n of y elements, gaussian smoothed
    void fill_current_state() override;
    void read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]);

    uint8_t gaussian_smoothing_window;
    uint8_t real_y_steps;
    uint8_t real_u_steps;
};

// ----------------------------------------------------------------------------------

template<typename T>
class LinearModelNoYwideD1MultiAttention : virtual public NeuralLinearModelBase<T>
{
public:
    LinearModelNoYwideD1MultiAttention(const uint8_t _y_diffs, const uint8_t _u_window, DebugLogger *logger): 
        NeuralLinearModelBase<T>((1 << _y_diffs)+1, _u_window, logger),
        LinearModelBase<T>((1 << _y_diffs)+1, _u_window, logger),
        y_diffs(_y_diffs),
        u_window(_u_window)
    {        
    }

    virtual ~LinearModelNoYwideD1MultiAttention() {}

protected:
    void fill_current_state() override;

    uint8_t y_diffs;
    uint8_t u_window;
};

// implementation ------------------------------------------------------------------

template<typename T>
void LinearModelBase<T>::read_u(T t_array[], const size_t n) 
{
    this->u->get_last_n_items(t_array, n);
}

template<typename T>
void LinearModelBase<T>::read_y(T t_array[], const size_t n) 
{
    this->y->get_last_n_items(t_array, n);
}

template<typename T>
void NeuralLinearModelBase<T>::fill_u(const size_t pos_in_state, const uint8_t n) 
{
    T buff[n];
    this->read_u(buff, n);
    for (size_t i=0;i<n;i++) {
        this->state[0][pos_in_state + i] = normalize_u(buff[i]);
    }
}

template<typename T>
T NeuralLinearModelBase<T>::fill_u_and_get_u_sum(const size_t pos_in_state, const uint8_t n) 
{
    T buff[n];
    this->read_u(buff, n);
    T sum = T();
    for (size_t i=0;i<n;i++) {
        sum += buff[i];
        this->state[0][pos_in_state + i] = normalize_u(buff[i]);
    }

    return sum;
}

template<typename T>
void NeuralLinearModelBase<T>::fill_y(const size_t pos_in_state) 
{
    this->state[0][pos_in_state] = normalize_y(this->y->get_last_item());
}

template<typename T>
void NeuralLinearModelBase<T>::fill_dy_chain(const size_t pos_in_state, const uint8_t n) 
{
    size_t i = pos_in_state;
    T diffs[n+1];
    this->read_y(diffs, n+1);
    for (uint8_t j=1; j<=n; j++) {
        calculate_diffs(diffs, n + 1 - j);
        this->state[0][i++] = normalize_dy(diffs[n - j], j);
    }
}

template<typename T>
void NeuralLinearModelBase<T>::fill_d1u(const size_t pos_in_state, const uint8_t n) 
{
    T diffs[n+1];
    this->read_u(diffs, n+1);
    calculate_diffs(diffs, n);
    for (size_t j=0; j<n; j++) {
        
        this->state[0][pos_in_state + j] = normalize_d1u(diffs[j]);
    }
}

template<typename T>
void NeuralLinearModelBase<T>::load_weights(const T w[]) 
{
    for (int i=0;i<GPC_LINEAR_MODEL_WEIGHTS;i++) {
        weights[i][0] = w[i];
    }
}

template<typename T>
T LinearModelBase<T>::predict_one_step(const T &_u, const T &dk) 
{
    this->u->add(_u);

    if (!this->ready()) {
        this->y->add(T() + dk);
        return T() + dk;
    }

    // get next denormalized dy - can be overriden in derived classes
    const T denormalized_dy = this->predict_next_dy();    
    
    T last_y = this->y->get_last_item();
    T py = last_y + denormalized_dy + dk;
    this->y->add(py);

    return py;
}

template<typename T>
T LinearModelBase<T>::check_and_denormalize_dy(const T &dy) 
{
    T denormalized_dy = T();
    if (!std::isnan(dy) && !std::isinf(dy)) {
        denormalized_dy = constrain_float(denormalize_d1y(dy), -GPC_LINEAR_MODEL_MAX_DY_ABS, GPC_LINEAR_MODEL_MAX_DY_ABS);
    }

    return denormalized_dy;
}

template<typename T>
T NeuralLinearModelBase<T>::predict_next_dy()
{
    // can be overriden in derived classes
    this->fill_current_state();

    // A[1xN] * W[Nx1] = Y[1x1]  <-- model predicts dy, not y!
    MatrixNxM<T, 1, 1> pdy = this->state * this->weights;

    // check for NaN and Inf and denormalize dy 
    return this->check_and_denormalize_dy(pdy[0][0]);
}

template<typename T>
void LinearModelBase<T>::measured_y(const T &_y) 
{
    y->replace_last(_y);
}

template<typename T>
void LinearModelBase<T>::reset_to(const LinearModelBase &model) 
{
    u->clear();
    y->clear();

    T u_other[u_steps];
    model.u->write_to(u_other);
    for (uint8_t i=0;i<u_steps;i++) {
        u->add(u_other[i]);
    }

    T y_other[y_steps];
    model.y->write_to(y_other);
    for (uint8_t i=0;i<y_steps;i++) {
        y->add(y_other[i]);
    }
}

template<typename T>
bool LinearModelBase<T>::ready() 
{
    return y->ready() && u->ready();
}

// ------------------------------------

template<typename T>
void LinearModel<T>::fill_current_state() 
{
    GPC_DEBUG_LOG_INIT;

    if (!this->ready()) {
        return;
    }

    this->fill_u(0, GPC_LINEAR_MODEL_U);
    this->fill_y(GPC_LINEAR_MODEL_U);
    this->fill_dy_chain(GPC_LINEAR_MODEL_U + 1, GPC_LINEAR_MODEL_DY);
}
 // ------------------------------------------------------------

template<typename T>
void LinearModelNoYuDiff<T>::fill_current_state() 
{
    GPC_DEBUG_LOG_INIT;

    if (!this->ready()) {
        return;
    }

    this->fill_u(0, GPC_LINEAR_MODEL_U);
    this->fill_d1u(GPC_LINEAR_MODEL_U, GPC_LINEAR_MODEL_D1U);
    this->fill_dy_chain(GPC_LINEAR_MODEL_U + GPC_LINEAR_MODEL_D1U, GPC_LINEAR_MODEL_DY);
}

template<typename T>
void LinearModelNoYuDiff<T>::read_u(T t_array[], const size_t n) 
{
    // use gaussian smoothing for u
    read_and_smooth(this->u, t_array, n, defines::gpc::gpc_gaussian_smoothing_weights);
}

template<typename T>
void LinearModelNoYuDiff<T>::read_y(T t_array[], const size_t n) 
{
    // use gaussian smoothing for y
    read_and_smooth(this->y, t_array, n, defines::gpc::gpc_gaussian_smoothing_weights);
}

template<typename T>
void LinearModelNoYuDiff<T>::read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]) 
{
    for (size_t i=0; i<n; i++) {
        buff[i] = T();
    }

    T raw_data[n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1];
    c->get_last_n_items(raw_data, n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1);
    for (size_t k=0, i=GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i<n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i++, k++) {
        for (size_t j=0; j<GPC_GAUSSIAN_SMOOTHING_WINDOW; j++) {
            buff[k] += gaussian_weights[j] * raw_data[i-j];
        }
    }
}

template<typename T>
void LinearModelNoYwideD1MultiAttention<T>::fill_current_state() 
{
    GPC_DEBUG_LOG_INIT;

    if (!this->ready()) {
        return;
    }

    // fill u
    T u_sum = this->fill_u_and_get_u_sum(0, this->u_window);

    // fill sum of u 
    this->state[0][this->u_window] = normalize_u(u_sum);

    // fill y diffs in increasing steps
    T yy[this->y_steps];
    this->read_y(yy, this->y_steps);
    const T last_y = yy[this->y_steps - 1];
    for (size_t i=0; i<this->y_diffs; i++) {
        this->state[0][this->u_window + 1 + i] = normalize_dy(last_y - yy[this->y_steps - 1 - (1 << (i+1))], 1);
    }
}


template<typename T>
void DifferenceEquationModel<T>::load_weights(const T w[]) 
{
    for (size_t i=0;i<this->an;i++) {
        a[i] = w[i];
    }

    for (size_t i=0;i<this->bn;i++) {
        b[i] = w[this->an + i];
    }
}

template<typename T>
T DifferenceEquationModel<T>::predict_next_dy() 
{
    T cu[this->bn];
    this->read_u(cu, this->bn);

    T cy[this->an + 1];
    this->read_y(cy, this->an + 1);
    calculate_diffs(cy, this->an);

    T dy = T();

    for (size_t i=0;i<this->an;i++) {
        dy += a[i]*cy[this->an-i-1];
    }

    for (size_t i=0;i<this->bn;i++) {
        dy += b[i]*cu[this->bn-i-1];
    }

    // check for NaN and Inf and denormalize dy 
    return this->check_and_denormalize_dy(dy);
}

template<typename T>
void DifferenceEquationModel<T>::read_y(T t_array[], const size_t n) 
{
    // use gaussian smoothing for y
    read_and_smooth(this->y, t_array, n, defines::gpc::gpc_gaussian_smoothing_weights);
}

template<typename T>
void DifferenceEquationModel<T>::read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]) 
{
    for (size_t i=0; i<n; i++) {
        buff[i] = T();
    }

    T raw_data[n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1];
    c->get_last_n_items(raw_data, n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1);
    for (size_t k=0, i=GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i<n + GPC_GAUSSIAN_SMOOTHING_WINDOW - 1; i++, k++) {
        for (size_t j=0; j<GPC_GAUSSIAN_SMOOTHING_WINDOW; j++) {
            buff[k] += gaussian_weights[j] * raw_data[i-j];
        }
    }
}

