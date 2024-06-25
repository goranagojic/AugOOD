#include "kernel_options.h"

#include <vector>

using namespace std;


KernelOptions::KernelOptions(int start_kernel_, int end_kernel_, int step_, std::vector<int> &angles_) :
    RangeOptions(start_kernel_, end_kernel_, step_, angles_) {
}

std::vector<int> KernelOptions::get_kernels() {
    return get_values();
}
