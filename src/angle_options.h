//
// Created by gorana on 11.3.21..
//

#ifndef PICTUREMODIFIER_ANGLE_OPTIONS_H
#define PICTUREMODIFIER_ANGLE_OPTIONS_H

#include "range_options.h"

#include <vector>


class AngleOptions : RangeOptions<double> {
public:
    AngleOptions(double start_angle_, double end_angle_, double step_, std::vector<double> &angles_);
    std::vector<double> get_angles();
};

#endif //PICTUREMODIFIER_ANGLE_OPTIONS_H
