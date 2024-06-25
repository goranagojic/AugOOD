#include "run_options.h"

#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cassert>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>


using namespace boost;
using namespace std;

namespace po = boost::program_options;


/**
 * Create an object with `RunOption::GAUSS_NOISE` as default option value.
 */
RunOption::RunOption() {
    boost_parser_out = RunOption::GAUSS_NOISE;
    input_options.push_back(boost_parser_out);
}


/**
 * Tokenize an input option string to get separate option strings.
 *
 * The method will populate a `input_options` class field. If the
 * field is already populated, the previous content will be lost.
 * Each option string will be trimmed and converted to uppercase.
 * Comma is used as a delimiter to split the input option string
 * into option substrings.
 */
void RunOption::extract_options() {
    assert(!boost_parser_out.empty());
    boost::split(input_options, boost_parser_out, boost::is_any_of(","));
    for (auto &opt : input_options) {
        boost::trim(opt);
        boost::to_upper(opt);
    }
    assert(!input_options.empty());
}

/**
 * Replaces composite options in `input_values` vector with corresponding
 * groups of elementary options.
 *
 * E.g. composite option BLUR will be replaced with elementary options
 * GAUSS_UNI_BLUR, MOT_BLUR_H, MOT_BLUR_V and MOT_BLUR_C.
 *
 * Expansion list
 * --------------
 * Here you can see the list of composite options that are expanded and
 * the groups of the corresponding elementary options.
 *
 * `RunOption::MOT_BLUR` is expanded into:
 *      - `RunOption::MOT_BLUR_C`
 *      - `RunOption::MOT_BLUR_H`
 *      - `RunOption::MOT_BLUR_V`
 *
 *  `RunOption::BLUR` is expanded into:
 *      - `RunOption::GAUSS_UNI_BLUR`
 *      - all elementary options of `RunOption::MOT_BLUR`
 *
 *  `RunOption::NOISE` is expanded into:
 *      - `RunOption::GAUSS_NOISE_MONO`
 *      - `RunOption::GAUSS_NOISE`
 *      - `RunOption::GAUSS_NOISE_R`
 *      - `RunOption::GAUSS_NOISE_G`
 *      - `RunOption::GAUSS_NOISE_B`
 *      - `RunOption::GAUSS_NOISE_YUV`
 *      - `RunOption::GAUSS_NOISE_FREQ`
 *
 *  `RunOption::ALL` is expanded into:
 *      - all elementary options of `RunOption::BLUR`
 *      - all elementary options of `RunOption::NOISE`
 *
 */
void RunOption::expand_options() {

    /* get string option tokens */
    extract_options();

    auto &iopts = input_options;
    vector<string> topts;

    /* add horizontal, vertical and custom mb options */
    auto add_motion_blur_opts = [&topts](){
        topts.push_back(RunOption::MOT_BLUR_C);
        topts.push_back(RunOption::MOT_BLUR_H);
        topts.push_back(RunOption::MOT_BLUR_V);
    };

    /* add gaussian and motion blur options */
    auto add_blur_opts = [&topts, &add_motion_blur_opts](){
        topts.push_back(RunOption::GAUSS_UNI_BLUR);
        add_motion_blur_opts();
    };

    /* add all noise options */
    auto add_noise_opts = [&topts](){
        topts.push_back(RunOption::GAUSS_NOISE);
        topts.push_back(RunOption::GAUSS_NOISE_MONO);
        topts.push_back(RunOption::GAUSS_NOISE_R);
        topts.push_back(RunOption::GAUSS_NOISE_G);
        topts.push_back(RunOption::GAUSS_NOISE_B);
        topts.push_back(RunOption::GAUSS_NOISE_YUV);
        topts.push_back(RunOption::GAUSS_NOISE_FREQ);
    };

    /* add all options */
    auto add_all_opts = [&add_blur_opts, &add_noise_opts](){
        add_blur_opts();
        add_noise_opts();
    };

    auto expand_option_message = [](const string &opt){
        cout << "[RunOption::expand_options] Expanding " << opt << " option..." << endl;
    };

    auto it = input_options.begin();
    while (it != input_options.end()) {
        auto &iopt = *it;
        if (iopt == RunOption::MOT_BLUR) {
            expand_option_message(RunOption::MOT_BLUR);
            add_motion_blur_opts();
            it = input_options.erase(it);
        } else if (iopt == RunOption::BLUR) {
            expand_option_message(RunOption::BLUR);
            add_blur_opts();
            it = input_options.erase(it);
        } else if (iopt == RunOption::NOISE) {
            expand_option_message(RunOption::NOISE);
            add_noise_opts();
            it = input_options.erase(it);
        } else if (iopt == RunOption::ALL) {
            expand_option_message(RunOption::ALL);
            add_all_opts();
            it = input_options.erase(it);
        } else {
            it++;
        }
    }

    /* insert all expanded options into input options */
    input_options.insert(input_options.end(), topts.begin(), topts.end());

    /* remove duplicate options if any */
    sort(input_options.begin(), input_options.end());
    input_options.erase( unique(input_options.begin(), input_options.end()), input_options.end() );

    assert(!input_options.empty());
}

/**
 * Checks if any of options for received through `runOptions` command
 * line argument is invalid.
 *
 * An input option string parsed by boost::program_options module is first tokenized by calling
 * 'RunOption::extract_options' method and composite options are expanded into groups of elementary options with a call
 * to `RunOption::expand_options` method.
 */
void RunOption::validate_value() {
    try {
        expand_options();
        assert(!input_options.empty());

        auto &aopts = RunOption::available_options;
        for (string &opt : input_options) {
            if (find(aopts.begin(), aopts.end(), opt) == aopts.end()) {
                stringstream ss;
                ss << "Vrednost argumenta 'runOptions' ne moze biti " << quoted(opt) << "." << endl;
                throw invalid_argument(ss.str());
            }
        }
    }
    catch(...) {
        throw_with_nested(po::validation_error(po::validation_error::invalid_option_value, "run_options"));
    }
}

/**
 * Checks if a given option is present in the `input_values` vector.
 *
 * @return An indicator if the option is found in `input_values` vector.
 */
bool RunOption::find_option(const string &option) const {
    auto &iopt = input_options;
    return find(iopt.begin(), iopt.end(), option) != iopt.end();
}


std::istream &operator>>(std::istream &is, RunOption &ropt) {
    is >> ropt.boost_parser_out;
    cout << ">>> Ucitavam " << ropt.boost_parser_out << endl;
    ropt.validate_value();
    return is;
}


std::ostream &operator<<(std::ostream &os, RunOption const &ropt) {
    return os << ropt.boost_parser_out;
}