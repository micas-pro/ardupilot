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
#define GPC_LINEAR_MODEL_AN                     1
#define GPC_LINEAR_MODEL_BN                     1
#define GPC_LINEAR_MODEL_UDELAY                 18
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

const float linear_model_w[GPC_LINEAR_MODEL_AN + GPC_LINEAR_MODEL_BN] = { 0.999695386209530f,   0.003797495125387f };

// N = 40, Nu = 5, lambda = 1, udelay = 18
const float gpc_K[GPC_Nu][GPC_N] = {
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.003055366239f, 0.008029785244f, 0.013941983708f, 0.019952288154f, 0.025349337231f, 0.030133317747f, 0.034304416450f, 0.037862820032f, 0.040808715129f, 0.043142288319f, 0.044863726124f, 0.045973215008f, 0.046470941381f, 0.046357091593f, 0.045631851939f, 0.044295408656f, 0.042347947926f, 0.039789655873f, 0.036620718564f, 0.032841322011f, 0.028451652168f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.001135382766f, -0.000050158270f, 0.002374948657f, 0.005373715396f, 0.008285419375f, 0.011110087113f, 0.013847745124f, 0.016498419912f, 0.019062137973f, 0.021538925795f, 0.023928809859f, 0.026231816636f, 0.028447972591f, 0.030577304179f, 0.032619837849f, 0.034575600039f, 0.036444617183f, 0.038226915704f, 0.039922522017f, 0.041531462530f, 0.043053763643f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000980688738f, -0.003821815886f, -0.004948598675f, -0.005056145255f, -0.004754624402f, -0.004044160721f, -0.002924878783f, -0.001396903120f, 0.000539641775f, 0.002884631446f, 0.005637941474f, 0.008799447481f, 0.012369025124f, 0.016346550098f, 0.020731898137f, 0.025524945013f, 0.030725566536f, 0.036333638551f, 0.042349036944f, 0.048771637638f, 0.055601316594f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000839387815f, -0.003283722229f, -0.008027100291f, -0.011335650453f, -0.013769086214f, -0.015327674146f, -0.016011680740f, -0.015821372405f, -0.014757015469f, -0.012818876180f, -0.010007220704f, -0.006322315126f, -0.001764425450f, 0.003666182401f, 0.009969242584f, 0.017144489340f, 0.025191656988f, 0.034110479928f, 0.043900692643f, 0.054562029695f, 0.066094225728f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000711331466f, -0.002794325545f, -0.006858422709f, -0.013462295918f, -0.018755136771f, -0.022737344627f, -0.025409318723f, -0.026771458174f, -0.026824161974f, -0.025567828995f, -0.023002857987f, -0.019129647579f, -0.013948596280f, -0.007460102476f, 0.000335435568f, 0.009437619708f, 0.019846051923f, 0.031560334310f, 0.044580069091f, 0.058904858605f, 0.074534305316f }
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

    virtual void read_y(T t_array[], const size_t n) override; // take last n of y elements, gaussian smoothed
    virtual T predict_next_dy() override;
    void read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]);

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
    T cu[this->bn + this->udelay];
    this->read_u(cu, this->bn + this->udelay);

    T cy[this->an + 1];
    this->read_y(cy, this->an + 1);
    calculate_diffs(cy, this->an);

    T dy = T();

    for (size_t i=0;i<this->an;i++) {
        dy += a[i]*cy[this->an-i-1];
    }

    for (size_t i=0;i<this->bn;i++) {
        dy += b[i]*cu[this->bn-i-1-this->udelay];
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

