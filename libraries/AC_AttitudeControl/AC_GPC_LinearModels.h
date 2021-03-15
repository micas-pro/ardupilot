#pragma once

#include "AC_GPC_Helpers.h"
#include <AP_Math/AP_Math.h>
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
#define GPC_Nu                                  5
#define GPC_N                                   40
#define GPC_lambda                              1.000f
#define GPC_MAX_duk                             0.4f
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
    2.555961847305f, 
    -3.606574058533f, 
    0.749199748039f, 
    -0.836554765701f, 
    -0.057546745986f, 
    -3.632860422134f, 
    0.867749750614f, 
    2.809866189957f, 
    -3.319396018982f, 
    -3.169429779053f, 
    0.895687818527f, 
    4.529780864716f, 
    -3.645090103149f, 
    -7.897997379303f, 
    6.779769897461f, 
    2.711076736450f, 
    -3.404711484909f, 
    -4.076398849487f, 
    2.539579153061f, 
    5.552387237549f, 
    -13.000236511230f, 
    18.222537994385f, 
    -11.215386390686f, 
    -4.081554412842f, 
    -0.938470363617f, 
    15.093239784241f, 
    -8.436522483826f, 
    -17.637632369995f, 
    27.168569564819f, 
    -13.987618446350f, 
    0.525150835514f, 
    -0.597688198090f, 
    -0.120367549360f, 
    0.502316474915f, 
    -0.083815030754f
};

// N = 40, Nu = 5, lambda = 1
const float gpc_K[GPC_Nu][GPC_N] = {
    { -0.047746212614f, -0.014063933557f, 0.005344697919f, -0.003654388324f, -0.019528202598f, -0.006010008408f, 0.002409269489f, -0.010388617390f, -0.014154050279f, 0.002666448337f, 0.016058370453f, 0.018723504824f, 0.026201168119f, 0.032160720680f, 0.033082638436f, 0.044455814705f, 0.063008582170f, 0.064969779452f, 0.058753182832f, 0.067433427116f, 0.078877143139f, 0.075731751587f, 0.073696155148f, 0.083419749470f, 0.085348096696f, 0.073373130874f, 0.067307743475f, 0.069384507667f, 0.063301836912f, 0.053737904252f, 0.052951872691f, 0.051074867484f, 0.037993254758f, 0.024889877334f, 0.017323585150f, 0.006166529789f, -0.009968076587f, -0.023269703868f, -0.036545545953f, -0.056768011532f }, 
    { 0.032783186305f, -0.038354729393f, -0.018516364447f, 0.006328977177f, 0.007238973443f, -0.014604099929f, -0.007418624168f, 0.008944357183f, -0.000920384060f, -0.015233381418f, -0.007856309797f, 0.003362022477f, 0.001985125029f, 0.006769583638f, 0.012622423664f, 0.006073019582f, 0.005381919203f, 0.022957376524f, 0.029683865149f, 0.018501662340f, 0.019733730403f, 0.032769286243f, 0.030875987018f, 0.022595082500f, 0.030681256595f, 0.039978243823f, 0.031834855294f, 0.023910667593f, 0.028705114746f, 0.027547161698f, 0.017470954629f, 0.016715626293f, 0.021971267508f, 0.016253313204f, 0.006845591594f, 0.004724992612f, 0.001969408544f, -0.007449007069f, -0.014153447550f, -0.016573667920f }, 
    { 0.018765276215f, 0.037787932004f, -0.039824369598f, -0.013887619294f, 0.020814163771f, 0.008026506833f, -0.015838919746f, -0.000960754972f, 0.016381213337f, -0.002640531351f, -0.021226595875f, -0.013034408790f, -0.006547609506f, -0.013154095606f, -0.009620411879f, -0.008784656160f, -0.023987236839f, -0.025803056814f, -0.006641206273f, -0.006078963955f, -0.023304518352f, -0.019735402777f, -0.006013011080f, -0.013741700428f, -0.023015165661f, -0.009118697993f, 0.001860600082f, -0.008055490244f, -0.011987062401f, -0.001377863249f, -0.001947433769f, -0.010666411165f, -0.004201708378f, 0.007413490848f, 0.005045374370f, 0.001957510230f, 0.009165616866f, 0.013679984666f, 0.011464208502f, 0.015776182735f }, 
    { 0.006852385850f, 0.020024665686f, 0.038845439328f, -0.032074180798f, 0.003720955002f, 0.018075523031f, 0.006940009239f, -0.009414673927f, 0.004780785077f, 0.014122569667f, -0.004764930159f, -0.019967623500f, -0.017100410896f, -0.018020131230f, -0.026853752837f, -0.026253632368f, -0.031181906462f, -0.047631960395f, -0.050567151873f, -0.038617441962f, -0.042924988184f, -0.057172922262f, -0.053403398366f, -0.045197143861f, -0.053383222399f, -0.058170830513f, -0.044441279988f, -0.035191926912f, -0.040037793531f, -0.037399783124f, -0.025288597409f, -0.023393935086f, -0.024825108025f, -0.012872434144f, 0.001767238414f, 0.006442996455f, 0.013462015680f, 0.028388694181f, 0.040594470489f, 0.049514389483f }, 
    { -0.003555675044f, 0.004868070097f, 0.023359819508f, 0.049445467507f, -0.011544751224f, -0.002163246216f, 0.017097332626f, 0.013375462034f, -0.005143217691f, 0.001985165150f, 0.015328610658f, 0.002088313547f, -0.019049025267f, -0.025603091972f, -0.029646750893f, -0.039621842081f, -0.042316151901f, -0.048631667153f, -0.068606276531f, -0.079744648597f, -0.071667720995f, -0.072391712916f, -0.086849707984f, -0.088344634618f, -0.080097017468f, -0.084881120825f, -0.091415118248f, -0.079330850624f, -0.063942687149f, -0.061418115380f, -0.056382395762f, -0.040725089912f, -0.031322596011f, -0.027881065990f, -0.013049025290f, 0.009463396341f, 0.025158821736f, 0.040503045601f, 0.063772797170f, 0.087484161954f }
};

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
class LinearModelNoYwideD1MultiAttention : virtual public LinearModelBase<T>
{
public:
    LinearModelNoYwideD1MultiAttention(const uint8_t _y_diffs, const uint8_t _u_window, DebugLogger *logger): 
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
        denormalized_dy = constrain_float(denormalize_d1y(pdy[0][0]), -GPC_LINEAR_MODEL_MAX_DY_ABS, GPC_LINEAR_MODEL_MAX_DY_ABS);
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

    this->fill_u(0, this->u_window);

    // fill sum of u 
    T u_sum = T();
    for (size_t i=0; i<this->u_window; i++) {
        u_sum += this->state[0][i];
    }

    this->state[0][this->u_window] = normalize_u(u_sum);

    // fill y diffs in increasing steps
    T yy[this->y_steps];
    this->read_y(yy, this->y_steps);
    const T last_y = yy[this->y_steps - 1];
    for (size_t i=0; i<this->y_diffs; i++) {
        this->state[0][this->u_window + 1 + i] = normalize_dy(last_y - yy[this->y_steps - 1 - (1 << (i+1))], 1);
    }
}

