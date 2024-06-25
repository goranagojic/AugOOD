//
// Created by gorana on 11.3.21..
//

#ifndef PICTUREMODIFIER_KERNEL_OPTIONS_H
#define PICTUREMODIFIER_KERNEL_OPTIONS_H

#include "range_options.h"

#include <vector>


class KernelOptions : RangeOptions<int> {
public:
    KernelOptions(int start_angle_, int end_angle_, int step_, std::vector<int> &angles_);

    std::vector<int> get_kernels();
};


#endif //PICTUREMODIFIER_KERNEL_OPTIONS_H
