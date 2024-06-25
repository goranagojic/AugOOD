#include "angle_options.h"

#include <cassert>
#include <iostream>


using namespace std;


AngleOptions::AngleOptions(double start_angle_, double end_angle_, double step_, std::vector<double> &angles_) :
    RangeOptions<double>(start_angle_, end_angle_, step_, angles_) {
}

std::vector<double> AngleOptions::get_angles() {
    return get_values();
}