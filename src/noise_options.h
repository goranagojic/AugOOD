#ifndef PICTUREMODIFIER_NOISE_OPTIONS_H
#define PICTUREMODIFIER_NOISE_OPTIONS_H

#include "range_options.h"

#include <vector>


class NoiseOptions : RangeOptions<double> {
public:
    NoiseOptions(double start_angle_, double end_angle_, double step_, std::vector<double> &angles_);
    std::vector<double> get_noise_deviations();
};


#endif //PICTUREMODIFIER_NOISE_OPTIONS_H
