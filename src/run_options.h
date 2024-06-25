#ifndef RUN_OPTIONS_H
#define RUN_OPTIONS_H


#include <istream>
#include <ostream>
#include <string>
#include <vector>


/**
 * The class stores possible values for `runOptions` command line argument alongside utilities necessary for option
 * parsing and validation.
 *
 * Two type of input options are supported: elementary and composite options. Elementary options result in application
 * of specific transformation on each image from an input directory specified with -I option. E.g. elementary option
 * GAUSS_UNI_BLUR results in applying Gaussian blur algorithm on each input image.
 * Composite options are short names for collections of logically related elementary options. E.g. composite option
 * BLUR stands for four elementary options (GAUSS_UNI_BLUR, MOT_BLUR_H, MOT_BLUR_V, MOT_BLUR_C) and in latter parsing
 * stages will be replaced with those elementary options.
 *
 * */
class RunOption {

private:
    /** An option string parsed by Boost library */
    std::string boost_parser_out;

    /** A collection of all run option values parsed from `boost_parser_out` */
    std::vector<std::string> input_options;

public:
    /** Apply Gaussian blur on an input image */
    constexpr static char GAUSS_UNI_BLUR[] = "GAUSS_UNI_BLUR";

    /** Results in application of custom motion blur on the input image. */
    constexpr static char MOT_BLUR_C[] = "MOT_BLUR_C";

    /** Results in application of horizontal motion blur on the input image. */
    constexpr static char MOT_BLUR_H[] = "MOT_BLUR_H";

    /** Results in application of vertical motion blur on the input image. */
    constexpr static char MOT_BLUR_V[] = "MOT_BLUR_V";

    /** Apply all motion blur types on the input image. */
    constexpr static char MOT_BLUR[] = "MOT_BLUR";

    /** Apply all blur transformations on an image */
    constexpr static char BLUR[] = "BLUR";

    /** Results in application of all noise deformations on the input image. */
    constexpr static char NOISE[] = "NOISE";

    /** Apply Gaussian noise on a monochromatic image */
    constexpr static char GAUSS_NOISE_MONO[] = "GAUSS_NOISE_MONO";

    /** Apply Gaussian noise on an unchanged image */
    constexpr static char GAUSS_NOISE[] = "GAUSS_NOISE";

    /** Apply Gaussian noise on an image's red channel */
    constexpr static char GAUSS_NOISE_R[] = "GAUSS_NOISE_R";

    /** Apply Gaussian noise on an image's green channel */
    constexpr static char GAUSS_NOISE_G[] = "GAUSS_NOISE_G";

    /** Apply Gaussian noise on an image's blue channel */
    constexpr static char GAUSS_NOISE_B[] = "GAUSS_NOISE_B";

    /** Apply Gaussian noise on an image transformed to YUV color space */
    constexpr static char GAUSS_NOISE_YUV[] = "GAUSS_NOISE_YUV";

    /** Apply Gaussian noise on an image in a frequency domain. */
    constexpr static char GAUSS_NOISE_FREQ[] = "GAUSS_NOISE_FREQ";

    /** Apply all supported transformation on an input image */
    constexpr static char ALL[] = "ALL";

    /** All supported values for `runOptions` command line parameter */
    inline const static std::vector<std::string> available_options{
            ALL,
            BLUR,
            GAUSS_UNI_BLUR,
            MOT_BLUR,
            MOT_BLUR_C,
            MOT_BLUR_H,
            MOT_BLUR_V,
            NOISE,
            GAUSS_NOISE_MONO,
            GAUSS_NOISE,
            GAUSS_NOISE_R,
            GAUSS_NOISE_G,
            GAUSS_NOISE_B,
            GAUSS_NOISE_YUV,
            GAUSS_NOISE_FREQ
    };

    /**
     * Create an object with `RunOption::GAUSS_NOISE` as default option value.
     */
    RunOption();

    /**
     * Tokenize an input option string to get separate option strings.
     *
     * The method will populate a `input_options` class field. If the
     * field is already populated, the previous content will be lost.
     * Each option string will be trimmed and converted to uppercase.
     */
    void extract_options();

    /**
     * Checks if any of options for received through `runOptions` command
     * line argument is invalid.
     */
    void validate_value();

    /**
     * Replaces composite options in `input_values` vector with corresponding
     * groups of elementary options.
     *
     * E.g. composite option BLUR will be replaced with elementary options
     * GAUSS_UNI_BLUR, MOT_BLUR_H, MOT_BLUR_V and MOT_BLUR_C.
     */
    void expand_options();

    /**
     * Checks if a given option is present in the `input_values` vector.
     *
     * @return An indicator if the option is found in `input_values` vector.
     */
    [[nodiscard]] bool find_option(const std::string &) const;

    friend std::istream &operator>>(std::istream &, RunOption &);

    friend std::ostream &operator<<(std::ostream &, RunOption const &);
};

#endif