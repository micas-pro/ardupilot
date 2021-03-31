#pragma once

#include "AC_GPC_Helpers.h"
//#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixNxM.h>
#include <AP_Math/circular_buffer.h>

#define GPC_LINEAR_MODEL_MAX_DY_ABS             0.45f
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
#define GPC_LINEAR_MODEL_AN                     2
#define GPC_LINEAR_MODEL_BN                     2
#define GPC_LINEAR_MODEL_UDELAY                 15

#define GPC_N                                   30
#define GPC_Nu                                  10
#define GPC_lambda                              1.000f

#define GPC_MAX_duk                             0.4f
#define GPC_GAUSSIAN_SMOOTHING_WINDOW           10
#define GPC_LOWPASS_SMOOTHING_WINDOW            7

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

const float linear_model_w[GPC_LINEAR_MODEL_AN + GPC_LINEAR_MODEL_BN] = { 1.393158126378344f,  -0.393346437291313f,   0.000978736065689f,   0.012937070114930f };

// N = 30, Nu = 10, lambda = 1, udelay = 15
const float gpc_K[GPC_Nu][GPC_N] = {
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000581088329f, 0.009351513757f, 0.025102215983f, 0.044294746544f, 0.063401743379f, 0.079627326636f, 0.091007593208f, 0.096289995752f, 0.094752535212f, 0.086028377807f, 0.069961165894f, 0.046490779530f, 0.015594964847f, -0.022733633345f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000301117582f, -0.004263991355f, -0.003643073836f, 0.002180514217f, 0.011467714723f, 0.022090206116f, 0.032203318220f, 0.040414808703f, 0.045764079851f, 0.047638323335f, 0.045681145715f, 0.039753082398f, 0.029800023248f, 0.015801442182f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000219981413f, -0.003841915222f, -0.013776732597f, -0.020436617713f, -0.021849408264f, -0.018663467586f, -0.012220048770f, -0.003838523648f, 0.005416769921f, 0.014778094972f, 0.023748614741f, 0.032132973850f, 0.039854440338f, 0.046882956791f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000153809379f, -0.002697034757f, -0.010514748069f, -0.025571747364f, -0.037291123793f, -0.042862161871f, -0.042280932883f, -0.036402275202f, -0.026189482629f, -0.012446936414f, 0.004258279826f, 0.023702547869f, 0.045797392489f, 0.070507513140f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000101803961f, -0.001794821516f, -0.007137443987f, -0.018380067758f, -0.036785517695f, -0.051138115923f, -0.058082974221f, -0.057162887032f, -0.048857227928f, -0.033819329974f, -0.012589306901f, 0.014619223436f, 0.047721109394f, 0.086681747096f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000062761676f, -0.001115110534f, -0.004558308457f, -0.012050180779f, -0.025356501897f, -0.045254535894f, -0.060069948549f, -0.066024544808f, -0.062276531699f, -0.048963151105f, -0.026431206947f, 0.005181150733f, 0.045817871166f, 0.095455207595f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000035103717f, -0.000631292409f, -0.002689006525f, -0.007375311810f, -0.016020526158f, -0.030018996310f, -0.049757448921f, -0.063168294356f, -0.066091688319f, -0.057330201154f, -0.036693864747f, -0.004110191980f, 0.040447079883f, 0.096986026435f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000016959645f, -0.000311693393f, -0.001422227910f, -0.004124801842f, -0.009367028595f, -0.018191536389f, -0.031658744918f, -0.049795782217f, -0.060170189827f, -0.058272291418f, -0.042578271303f, -0.012491339973f, 0.032220537260f, 0.091645876376f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000006287355f, -0.000121593597f, -0.000638312821f, -0.002036256781f, -0.004943177216f, -0.010081401683f, -0.018210603155f, -0.030072057725f, -0.045359894675f, -0.051316951669f, -0.043128727642f, -0.018904119462f, 0.022097711722f, 0.080165012902f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000001015136f, -0.000025638692f, -0.000213494670f, -0.000833217925f, -0.002261735831f, -0.004951035508f, -0.009390781076f, -0.016070829034f, -0.025446440555f, -0.036930281128f, -0.037500679504f, -0.022037662812f, 0.011469672771f, 0.063808911156f }
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

template<typename T>
T get_lowpass_smoothed(const CircularBuffer<T> *smoothed, const T &x, const size_t window_size, const float weights[]) 
{
    T data[window_size + 1];
    smoothed->get_last_n_items(data, window_size);
    const T last_smoothed_y = data[window_size - 1];
    data[window_size] = x;

    calculate_diffs(data, window_size);

    T u = T();
    for (size_t i=0; i<window_size; i++) {
        u += weights[i] * data[i];
    }

    return last_smoothed_y + u;
}

template<typename T>
void read_and_smooth_gaussian(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]) 
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

    virtual void read_u(T t_array[], const size_t n); // take last n of u elements
    virtual void read_y(T t_array[], const size_t n); // take last n of y elements

protected:

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
    DifferenceEquationModel(const uint8_t _an, const uint8_t _bn, const uint8_t _udelay, DebugLogger *logger): 
        LinearModelBase<T>(_an + GPC_GAUSSIAN_SMOOTHING_WINDOW, _bn + _udelay, logger),
        an(_an),
        bn(_bn),
        udelay(_udelay)
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

    virtual T predict_next_dy() override;

    uint8_t an, bn, udelay;

    // y[k+1] = a[0]*y(k) + a[1]*y(k-1) + ... + a[an-1]*y(k-an+1) + 
    //        + b[0]*u(k-udelay) + b[1]*u(k-udelay-1) + ... + b[bn-1]*u(k-udelay-bn+1)
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

    void read_u(T t_array[], const size_t n) override; // take last n of u elements, gaussian smoothed
    void read_y(T t_array[], const size_t n) override; // take last n of y elements, gaussian smoothed

protected:
    
    void fill_current_state() override;

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
    const size_t u_elements = this->bn + this->udelay;
    T cu[u_elements];
    this->read_u(cu, u_elements);

    T cy[this->an + 1];
    this->read_y(cy, this->an + 1);
    calculate_diffs(cy, this->an);

    T dy = T();

    for (size_t i=0;i<this->an;i++) {
        dy += a[i]*cy[this->an-i-1];
    }

    for (size_t i=0;i<this->bn;i++) {
        dy += b[i]*cu[u_elements - 1 - i - this->udelay];
    }

    // check for NaN and Inf and denormalize dy 
    return this->check_and_denormalize_dy(dy);
}


