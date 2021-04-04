#pragma once

// #ifdef GPC_DEBUG
// #include <stdio.h>
// #endif

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

#define GPC_N                                   70
#define GPC_Nu                                  2
#define GPC_lambda                              1.000f

#define GPC_MAX_duk                             0.9f
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

// N = 70, Nu = 2, lambda = 50000/Nu, udelay = 15
const float gpc_K[GPC_Nu][GPC_N] = {
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000029626f, 0.000000483042f, 0.000001395140f, 0.000002779561f, 0.000004641579f, 0.000006983179f, 0.000009805053f, 0.000013107382f, 0.000016890146f, 0.000021153247f, 0.000025896557f, 0.000031119933f, 0.000036823231f, 0.000043006302f, 0.000049668999f, 0.000056811171f, 0.000064432671f, 0.000072533350f, 0.000081113058f, 0.000090171648f, 0.000099708970f, 0.000109724876f, 0.000120219217f, 0.000131191845f, 0.000142642611f, 0.000154571368f, 0.000166977965f, 0.000179862256f, 0.000193224091f, 0.000207063322f, 0.000221379802f, 0.000236173382f, 0.000251443914f, 0.000267191250f, 0.000283415242f, 0.000300115741f, 0.000317292601f, 0.000334945672f, 0.000353074808f, 0.000371679860f, 0.000390760680f, 0.000410317121f, 0.000430349036f, 0.000450856276f, 0.000471838694f, 0.000493296143f, 0.000515228475f, 0.000537635542f, 0.000560517197f, 0.000583873294f, 0.000607703683f, 0.000632008219f, 0.000656786755f, 0.000682039142f }, 
    { 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, 0.000000000000f, -0.000000009084f, -0.000000120410f, 0.000000032323f, 0.000000593148f, 0.000001618643f, 0.000003130975f, 0.000005138774f, 0.000007645338f, 0.000010651872f, 0.000014158756f, 0.000018166045f, 0.000022673666f, 0.000027681496f, 0.000033189393f, 0.000039197207f, 0.000045704785f, 0.000052711972f, 0.000060218612f, 0.000068224553f, 0.000076729637f, 0.000085733712f, 0.000095236620f, 0.000105238209f, 0.000115738322f, 0.000126736805f, 0.000138233504f, 0.000150228263f, 0.000162720928f, 0.000175711345f, 0.000189199358f, 0.000203184814f, 0.000217667558f, 0.000232647436f, 0.000248124293f, 0.000264097974f, 0.000280568326f, 0.000297535195f, 0.000314998426f, 0.000332957866f, 0.000351413359f, 0.000370364753f, 0.000389811892f, 0.000409754624f, 0.000430192795f, 0.000451126250f, 0.000472554836f, 0.000494478398f, 0.000516896785f, 0.000539809841f, 0.000563217413f, 0.000587119348f, 0.000611515492f, 0.000636405692f, 0.000661789794f }
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

    // for(int i=0;i<window_size;i++) printf("%.3f ", data[i]);
    // printf("\n");

    const T last_smoothed_y = data[window_size - 1];
    data[window_size] = x;

    // for(int i=0;i<window_size+1;i++) printf("%.3f ", data[i]);
    // printf("\n");

    calculate_diffs(data, window_size);

    // for(int i=0;i<window_size;i++) printf("%.3f ", data[i]);
    // printf("\n");

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
    void reset_to(const LinearModelBase &model);
    bool ready();

    virtual T measured_y(const T &_y);
    void measured_y_no_smooth(const T &_y);
    virtual void read_u(T t_array[], const size_t n); // take last n of u elements
    virtual void read_y(T t_array[], const size_t n); // take last n of y elements

protected:

    T check_and_denormalize_dy(const T &dy);   

    virtual T predict_next_dy(const T &dk) = 0;

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
        LinearModelBase<T>(_an + GPC_GAUSSIAN_SMOOTHING_WINDOW + 1, _bn + _udelay, logger),
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

    virtual T measured_y(const T &_y) override;
    virtual void load_weights(const T w[]) override;

protected:

    virtual T predict_next_dy(const T &dk) override;

    uint8_t an, bn, udelay;

    // y[k+1] = a[0]*y(k) + a[1]*y(k-1) + ... + a[an-1]*y(k-an+1) + 
    //        + b[0]*u(k-udelay) + b[1]*u(k-udelay-1) + ... + b[bn-1]*u(k-udelay-bn+1)
    T *a; 
    T *b;    

private:

    T _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW];
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
    virtual T predict_next_dy(const T &dk) override;

    virtual void fill_current_state(const T &dk) = 0;

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
    void fill_current_state(const T &dk) override;
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
    
    void fill_current_state(const T &dk) override;

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
    void fill_current_state(const T &dk) override;

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
    const T denormalized_dy = this->predict_next_dy(dk);    
    
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
T NeuralLinearModelBase<T>::predict_next_dy(const T &dk)
{
    // can be overriden in derived classes
    this->fill_current_state(dk);

    // A[1xN] * W[Nx1] = Y[1x1]  <-- model predicts dy, not y!
    MatrixNxM<T, 1, 1> pdy = this->state * this->weights;

    // check for NaN and Inf and denormalize dy 
    return this->check_and_denormalize_dy(pdy[0][0]);
}

template<typename T>
T LinearModelBase<T>::measured_y(const T &_y) 
{
    y->replace_last(_y);
    return _y;
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
void LinearModel<T>::fill_current_state(const T &dk) 
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
void LinearModelNoYuDiff<T>::fill_current_state(const T &dk) 
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
void LinearModelNoYwideD1MultiAttention<T>::fill_current_state(const T &dk) 
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

    const T n = (GPC_LOWPASS_SMOOTHING_WINDOW-1)*1.1f + 0.9f;
    for (size_t i=0;i<GPC_LOWPASS_SMOOTHING_WINDOW-1;i++) {
        _low_pass_smoothing_weights[i] = 1.1f / n;
    }

    _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW-1] = 0.9f / n;
}

template<typename T>
T DifferenceEquationModel<T>::measured_y(const T &_y) 
{    
    if (this->y->ready()) {
        this->y->remove_last();
        T smoothed_y = get_lowpass_smoothed(this->y, _y, GPC_LOWPASS_SMOOTHING_WINDOW, _low_pass_smoothing_weights);
        this->y->add(smoothed_y);
        return smoothed_y;
    } else {
        this->y->replace_last(_y);
        return _y;
    }
}

template<typename T>
T DifferenceEquationModel<T>::predict_next_dy(const T &dk) 
{
    const size_t u_elements = this->bn + this->udelay;
    T cu[u_elements];
    this->read_u(cu, u_elements);

    T cy[this->an + 1];
    this->read_y(cy, this->an + 1);
    // for (size_t i=0;i<this->an + 1;i++) {
    //     cy[i] += dk;
    // }

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

template<typename T>
void LinearModelBase<T>::measured_y_no_smooth(const T &_y) 
{
    this->y->replace_last(_y);
}   
