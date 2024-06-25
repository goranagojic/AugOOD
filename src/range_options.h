//
// Created by gorana on 11.3.21..
//

#ifndef PICTUREMODIFIER_RANGE_OPTIONS_H
#define PICTUREMODIFIER_RANGE_OPTIONS_H

#include <vector>
#include <cassert>

/**
 * Template class for all arithmetic, range command line options. Supports storage of range values and utilities to
 * generate range from specified parameters.
 *
 * The class supports two types to specify input ranges:
 *  1) as a vector (e.g. 20, 40, 60, 80) or
 *  2) as range parameters. In this case start, end and step values have to be specified (e.g. start=20, end=80 and
 *     step=20). The parameters specified are used to internally generate a vector in format 1).
 *
 * Specific range options are supposed to inherit this class.
 *
 * @tparam T - some of arithmetic types
 */
template<
        typename T,
        typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type
        >
class RangeOptions {
protected:
    /** begining of the range */
    T start;

    /** end of the range */
    T end;

    /** step used to successively increment start value */
    T step;

    /** range of values */
    std::vector<T> values;

public:

    /**
     * Generates RangeOptions object and populates `values` field.
     *
     * If vector `values_` is not empty, it's content is copied to the `values` field.
     * Otherwise, parameters `start_`, `step_` and `end_` are used to generate values for `values` field by successivly
     * incrementing `start_` value with `step_` until the `_end` value is reached.
     *
     * `_step` can have both negative and positive value. If positive, `start_` must be less or equal to the `end_`. Otherwise
     * `end_` must be less or equal to `start_`.
     *
     **/
    RangeOptions(T start_, T end_, T step_, std::vector<T> values_) {
        if (!values_.empty()) {
            values = std::vector<T>(values_);
        } else {
            if (step_ >= 0) assert((start_ <= end_) && "If step>0, then start_angle must be less or equal to "
                                                                  "the end_angle");
            if (step_ <= 0) assert((start_ >= end_) && "If step<0, then start_angle must be greater or equal to "
                                                                  "the start_angle");

            if (step_ < 0) {
                std::swap(start_, end_);
                step_ = -step_;
            }

            while (start_ <= end_) {
                values.push_back(start_);
                start_ += step_;
            }
        }
    }

    /**
     * Returns range of values.
     *
     * @return vector<T> where T is some of the arithmetic types
     */
    std::vector<T> get_values() {
        return values;
    }
};


#endif //PICTUREMODIFIER_RANGE_OPTIONS_H
