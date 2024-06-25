//
// Created by gorana on 11.3.21..
//

#include "noise_options.h"

#include <vector>

using namespace std;


NoiseOptions::NoiseOptions(double start_kernel_, double end_kernel_, double step_, std::vector<double> &angles_) :
        RangeOptions(start_kernel_, end_kernel_, step_, angles_) {}

std::vector<double> NoiseOptions::get_noise_deviations() {
    return get_values();
}
