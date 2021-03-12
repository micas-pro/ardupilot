#pragma once

#include "AC_GPC_Helpers.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/matrixNxM.h>
#include <AP_Math/circular_buffer.h>

#define GPC_LINEAR_MODEL_Y                      0
#define GPC_LINEAR_MODEL_DY                     3
#define GPC_LINEAR_MODEL_U                      30
#define GPC_LINEAR_MODEL_D1U                    29
#define GPC_LINEAR_MODEL_WEIGHTS                (GPC_LINEAR_MODEL_Y + GPC_LINEAR_MODEL_DY + GPC_LINEAR_MODEL_U + GPC_LINEAR_MODEL_D1U)
#define GPC_Nu                                  1
#define GPC_N                                   70
#define GPC_lambda                              1.000f
#define GPC_GAUSSIAN_SMOOTHING_WINDOW           10

namespace defines {

namespace gpc {

const float normalization_dy[6][2] = {
    { 0.000000000000f, 80.000000000000f },
    { 0.000000000000f, 0.030000000000f },
    { 0.000000000000f, 0.000600000000f },
    { 0.000000000000f, 0.000600000000f },
    { 0.000000000000f, 0.000600000000f },
    { 0.000000000000f, 0.000600000000f }
};

const float normalization_u[2] = { 0.000000000000f, 1.200000000000f };
const float normalization_d1u[2] = { 0.000000000000f, 0.056000000000f };

const float linear_model_w[GPC_LINEAR_MODEL_WEIGHTS] = { 
    0.268710881472f, 
    0.102351501584f, 
    0.207284823060f, 
    0.285076498985f, 
    0.140161767602f, 
    -0.063153266907f, 
    -0.097613446414f, 
    -0.003727850737f, 
    -0.324658513069f, 
    0.107500173151f, 
    -0.121275201440f, 
    -0.302733749151f, 
    -0.403476685286f, 
    0.089980080724f, 
    0.172666028142f, 
    0.030766684562f, 
    -0.152927637100f, 
    0.268138051033f, 
    -0.014861730859f, 
    -0.403797298670f, 
    0.145706281066f, 
    0.308621972799f, 
    0.188603907824f, 
    -0.094689063728f, 
    1.424415469170f, 
    0.221995905042f, 
    0.698885023594f, 
    2.738082885742f, 
    0.014734034427f, 
    0.104031533003f, 
    -1.362865686417f, 
    1.366166234016f, 
    -0.572940349579f, 
    0.424482464790f, 
    -1.532802700996f, 
    2.383923530579f, 
    -1.037249445915f, 
    -1.288316726685f, 
    0.282602548599f, 
    2.319040298462f, 
    -1.182139992714f, 
    -2.655364274979f, 
    3.355310440063f, 
    -0.709246933460f, 
    -0.950512290001f, 
    -0.885428011417f, 
    3.081715583801f, 
    -2.573507785797f, 
    -2.195150375366f, 
    4.605765342712f, 
    -1.831322073936f, 
    -2.636324644089f, 
    -2.516123056412f, 
    10.121372222900f, 
    -9.447655677795f, 
    -1.320758342743f, 
    10.517411231995f, 
    -11.177677154541f, 
    4.285815715790f, 
    1.008337616920f, 
    -0.051476579159f, 
    -0.018991274759f
};

// N = 70, too long, CPU stuck?
const float gpc_K[GPC_Nu][GPC_N] = { 
    { 0.000000000000f, 0.000023619179f, -0.000011530310f, 0.000011079507f, 0.000015966199f, -0.000011954100f, 0.000007794231f, 0.000008185336f, 0.000009807995f, -0.000002833195f, 0.000007232447f, 0.000005924256f, -0.000000044955f, 0.000007504309f, 0.000009924050f, 0.000012648918f, 0.000011640830f, 0.000029471138f, 0.000035167128f, 0.000039966273f, 0.000053558072f, 0.000070238915f, 0.000085269202f, 0.000096103771f, 0.000118410978f, 0.000133270017f, 0.000155302781f, 0.000173784664f, 0.000201851326f, 0.000223509383f, 0.000248621803f, 0.000273152189f, 0.000299817700f, 0.000328291794f, 0.000359002711f, 0.000391477347f, 0.000425620717f, 0.000461493291f, 0.000499201662f, 0.000538793571f, 0.000580204624f, 0.000623423968f, 0.000668444402f, 0.000715293180f, 0.000763976738f, 0.000814495431f, 0.000866844538f, 0.000921026117f, 0.000977044827f, 0.001034905150f, 0.001094609373f, 0.001156159101f, 0.001219556505f, 0.001284804411f, 0.001351905746f, 0.001420863134f, 0.001491679034f, 0.001564355936f, 0.001638896455f, 0.001715303242f, 0.001793578921f, 0.001873726082f, 0.001955747317f, 0.002039645237f, 0.002125422466f, 0.002213081629f, 0.002302625351f, 0.002394056254f, 0.002487376971f, 0.002582590136f }
};

// N = 50
// const float gpc_K[GPC_Nu][GPC_N] = { 
//     { 0.000000000000f, 0.000216233590f, -0.000105559987f, 0.000101432891f, 0.000146170560f, -0.000109439785f, 0.000071356189f, 0.000074936756f, 0.000089792196f, -0.000025937900f, 0.000066213057f, 0.000054236561f, -0.000000411566f, 0.000068701948f, 0.000090854685f, 0.000115800850f, 0.000106571800f, 0.000269808274f, 0.000321955067f, 0.000365891239f, 0.000490324162f, 0.000643037289f, 0.000780639567f, 0.000879830049f, 0.001084052534f, 0.001220087046f, 0.001421797001f, 0.001590998650f, 0.001847948944f, 0.002046228465f, 0.002276132669f, 0.002500708361f, 0.002744831127f, 0.003005511463f, 0.003286669913f, 0.003583975215f, 0.003896557778f, 0.004224971199f, 0.004570191350f, 0.004932655290f, 0.005311773489f, 0.005707446590f, 0.006119608679f, 0.006548509253f, 0.006994207238f, 0.007456705890f, 0.007935961976f, 0.008431994345f, 0.008944845657f, 0.009474556923f }
// };

// N = 40
// const float gpc_K[GPC_Nu][GPC_N] = { 
//     { 0.000000000000f, 0.001020603924f, -0.000498234051f, 0.000478754510f, 0.000689912455f, -0.000516546361f, 0.000336795070f, 0.000353695035f, 0.000423811434f, -0.000122424653f, 0.000312519928f, 0.000255991900f, -0.000001942555f, 0.000324267280f, 0.000428826288f, 0.000546570040f, 0.000503009719f, 0.001273471822f, 0.001519600196f, 0.001726975142f, 0.002314287826f, 0.003035080634f, 0.003684551536f, 0.004152722071f, 0.005116634614f, 0.005758705797f, 0.006710759416f, 0.007509376630f, 0.008722159893f, 0.009658022158f, 0.010743150205f, 0.011803128134f, 0.012955366571f, 0.014185755309f, 0.015512798985f, 0.016916054410f, 0.018391417189f, 0.019941500261f, 0.021570909644f, 0.023281708231f }
// };

const float gpc_gaussian_smoothing_weights[GPC_GAUSSIAN_SMOOTHING_WINDOW] = {
    0.134926224710375f,   0.196316498560442f,   0.222455736655261f,   0.196316498560442f,   0.134926224710375f,   0.072220803783150f, 0.030106110127849f,   0.009774022935568f,   0.002471260008415f,   0.000486619948122f
};

} // namespace gpc

}  // namespace defines

// utils --------------------------------------------------------------------------------

float normalize_dy(const float dy, const uint8_t n);
float normalize_y(const float y);
float normalize_u(const float u);
float normalize_d1u(const float d1u);
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

    void load_weights(const T w[]);
    T predict_one_step(const T &_u, const T &dk);
    void measured_y(const T &_y);
    void reset_to(const LinearModelBase &model);
    bool ready();

protected:

    void fill_u(const size_t pos_in_state, const uint8_t n);
    void fill_y(const size_t pos_in_state);
    void fill_dy_chain(const size_t pos_in_state, const uint8_t n);
    void fill_d1u(const size_t pos_in_state, const uint8_t n);

    virtual void read_u(T t_array[], const size_t n); // take last n of u elements
    virtual void read_y(T t_array[], const size_t n); // take last n of y elements
    virtual void fill_current_state() = 0;

    DebugLogger *_logger;
    MatrixNxM<T, GPC_LINEAR_MODEL_WEIGHTS, 1> weights;
    MatrixNxM<T, 1, GPC_LINEAR_MODEL_WEIGHTS> state;
    uint8_t y_steps, u_steps;
    CircularBuffer<T> *y;
    CircularBuffer<T> *u;
};

// ----------------------------------------------------------------------------------

template<typename T>
class LinearModel : virtual public LinearModelBase<T>
{
public:
    LinearModel(const uint8_t _y_steps, const uint8_t _u_steps, DebugLogger *logger): LinearModelBase<T>(_y_steps, _u_steps, logger)
    {        
    }

    virtual ~LinearModel() {}

protected:
    void fill_current_state() override;
};

// ----------------------------------------------------------------------------------

template<typename T>
class LinearModelNoYuDiff : virtual public LinearModelBase<T>
{
public:
    LinearModelNoYuDiff(const uint8_t _y_steps, const uint8_t _u_steps, const uint8_t _gaussian_smoothing_window, DebugLogger *logger): 
        LinearModelBase<T>(_y_steps + _gaussian_smoothing_window + 1, _u_steps + _gaussian_smoothing_window, logger),
        gaussian_smoothing_window(_gaussian_smoothing_window),
        real_y_steps(_y_steps),
        real_u_steps(_u_steps)
    {        
    }

    virtual ~LinearModelNoYuDiff() {}

protected:
    //void read_u(T t_array[], const size_t n) override; // take last n of u elements, gaussian smoothed
    //void read_y(T t_array[], const size_t n) override; // take last n of y elements, gaussian smoothed
    void fill_current_state() override;
    void read_and_smooth(const CircularBuffer<T> *c, T buff[], const size_t n, const float gaussian_weights[]);

    uint8_t gaussian_smoothing_window;
    uint8_t real_y_steps;
    uint8_t real_u_steps;
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
void LinearModelBase<T>::fill_u(const size_t pos_in_state, const uint8_t n) 
{
    T buff[n];
    read_u(buff, n);
    for (size_t i=0;i<n;i++) {
        this->state[0][pos_in_state + i] = normalize_u(buff[i]);
    }
}

template<typename T>
void LinearModelBase<T>::fill_y(const size_t pos_in_state) 
{
    this->state[0][pos_in_state] = normalize_y(this->y->get_last_item());
}

template<typename T>
void LinearModelBase<T>::fill_dy_chain(const size_t pos_in_state, const uint8_t n) 
{
    size_t i = pos_in_state;
    T diffs[n+1];
    read_y(diffs, n+1);
    for (uint8_t j=1; j<=n; j++) {
        calculate_diffs(diffs, n + 1 - j);
        this->state[0][i++] = normalize_dy(diffs[n - j], j);
    }
}

template<typename T>
void LinearModelBase<T>::fill_d1u(const size_t pos_in_state, const uint8_t n) 
{
    T diffs[n+1];
    read_u(diffs, n+1);
    calculate_diffs(diffs, n);
    for (size_t j=0; j<n; j++) {
        
        this->state[0][pos_in_state + j] = normalize_d1u(diffs[j]);
    }
}

template<typename T>
void LinearModelBase<T>::load_weights(const T w[]) 
{
    for (int i=0;i<GPC_LINEAR_MODEL_WEIGHTS;i++) {
        weights[i][0] = w[i];
    }
}

template<typename T>
T LinearModelBase<T>::predict_one_step(const T &_u, const T &dk) 
{
    u->add(_u);

    if (!ready()) {
        y->add(T() + dk);
        return T() + dk;
    }

    // can be overriden in derived classes
    fill_current_state();

    // A[1xN] * W[Nx1] = Y[1x1]  <-- model predicts dy, not y!
    MatrixNxM<T, 1, 1> pdy = this->state * this->weights;    

    // denormalize dy and calculate y
    T denormalized_dy = T();
    T last_y = y->get_last_item();
    if (!std::isnan(pdy[0][0]) && !std::isinf(pdy[0][0])) {
        denormalized_dy = denormalize_d1y(pdy[0][0]);
    }
    
    T py = last_y + denormalized_dy + dk;
    y->add(py);

    return py;
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

// template<typename T>
// void LinearModelNoYuDiff<T>::read_u(T t_array[], const size_t n) 
// {
//     // use gaussian smoothing for u
//     read_and_smooth(this->u, t_array, n, defines::gpc::gpc_gaussian_smoothing_weights);
// }

// template<typename T>
// void LinearModelNoYuDiff<T>::read_y(T t_array[], const size_t n) 
// {
//     // use gaussian smoothing for y
//     read_and_smooth(this->y, t_array, n, defines::gpc::gpc_gaussian_smoothing_weights);
// }

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

