#include <iostream>
#include "../libraries/AP_Math/circular_buffer.h"
#include "../libraries/AC_AttitudeControl/AC_AttitudeControl_GPC.h"
#include "../libraries/AC_AttitudeControl/AC_GPC_Helpers.h"
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <iomanip> 

using namespace std;

DebugLogger logger;

struct UY {
    float u;
    float y;
    float y_target;
};

ostream& operator<<(ostream& os, const UY& rightOp)
{
    os << fixed << showpoint << setprecision(3);
    os << rightOp.u << "," << rightOp.y << "," << rightOp.y_target << endl;
}

vector<UY> read_csv_data(const string &filename)
{
    ifstream file(filename);
    if (!file.good()) {
        throw invalid_argument("Can't open csv file!");
    }

    vector<UY> result;
    string line;
    while (getline(file, line)) {
        stringstream lineStream(line);
        string cell;
        std::vector<std::string> parsedRow;
        UY uy{0.0, 0.0, 0.0};
        int i = 0;
        while(std::getline(lineStream, cell, ','))
        {
            if (i == 0) {
                uy.u = stof(cell);
            }

            if (i == 1) {
                uy.y = stof(cell);
            }

            if (i == 2) {
                uy.y_target = stof(cell);
            }

            i++;
        }

        result.push_back(uy);
    }

    return result;
}

void test(const string &filename, GPC_Controller<float, GPC_N, GPC_Nu> *gpc, LinearModelBase<float> *gpc_linear_model)
{
    logger.debug_msg("Loading data from: %s", filename.c_str());
    auto data = read_csv_data(filename);
    logger.debug_msg("Loaded %u samples", data.size());

    vector<float> gpc_u;
    gpc_u.resize(data.size());

    const int sim_start = 5000-8;
    const int sim_end = sim_start + 2000;

    // simulated model, preloaded with real y and u = 0
    DifferenceEquationModel<float> model(GPC_LINEAR_MODEL_AN, GPC_LINEAR_MODEL_BN, GPC_LINEAR_MODEL_UDELAY, &logger);
    model.load_weights(defines::gpc::linear_model_w);
    CircularBuffer<float> smoothed_y(GPC_LOWPASS_SMOOTHING_WINDOW + 5);
    
    float _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW];
    const float n = (GPC_LOWPASS_SMOOTHING_WINDOW-1)*0.9f + 1.1f;
    for (size_t i=0;i<GPC_LOWPASS_SMOOTHING_WINDOW-1;i++) {
        _low_pass_smoothing_weights[i] = 0.9f / n;
    }

    _low_pass_smoothing_weights[GPC_LOWPASS_SMOOTHING_WINDOW-1] = 1.1f / n;

    for (int i=50;i>=1;i--) {
        if (!smoothed_y.ready()) {
            smoothed_y.add(data[sim_start-i].y);
        } else {
            smoothed_y.add(get_lowpass_smoothed(&smoothed_y, data[sim_start-i].y, GPC_LOWPASS_SMOOTHING_WINDOW, _low_pass_smoothing_weights));
        }

        model.predict_one_step(0.0f, 0.0f);
        model.measured_y(smoothed_y.get_last_item());

        gpc_linear_model->predict_one_step(0.0f, 0.0f);
        gpc_linear_model->measured_y(smoothed_y.get_last_item());
    }

    for (int i=sim_start;i<sim_end;i++) {
        const float my = model.predict_one_step(gpc->get_current_u(), 0.0f);
        const float cu = gpc->get_current_u();        
        gpc_u[i] = gpc->run_step(my, data[i].y_target, 0.5f);
        if (fabs(gpc_u[i]) < 0.0001f) {
            gpc_u[i] = data[i].u;
        } else {
            logger.debug_msg("%u: y=%.3f ty=%.3f cu=%.3f next_u=%.3f", i, my, data[i].y_target, cu, gpc_u[i]);
        }
    }
}

int main(int argc, char* argv[]) {

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <path_to_file.csv>" << endl;
        return 1;
    }

    cout << "Started GPC testing..." << endl;

    logger.debug_msg("Logger test %u %.3f", 1, 0.123f);
    logger.debug_msg("GPC ctor");

    GPC_Params<float> params;
    params.lambda = GPC_lambda;
    params.min_u = -0.41f;
    params.max_u = 0.41f;

    auto linear_model = new DifferenceEquationModel<float>(GPC_LINEAR_MODEL_AN, GPC_LINEAR_MODEL_BN, GPC_LINEAR_MODEL_UDELAY, &logger);
    auto y0_linear_model = new DifferenceEquationModel<float>(GPC_LINEAR_MODEL_AN, GPC_LINEAR_MODEL_BN, GPC_LINEAR_MODEL_UDELAY, &logger);
    GPC_Controller<float, GPC_N, GPC_Nu> *gpc_pitch_controller = new GPC_Controller<float, GPC_N, GPC_Nu>(
        params, 
        linear_model, 
        y0_linear_model,
        &logger
    );

    logger.debug_msg("GPC initialize");
    gpc_pitch_controller->initialize();

    string data_file(argv[1]);
    test(data_file, gpc_pitch_controller, linear_model);

    delete gpc_pitch_controller;

    cout << "Done." << endl;
}

