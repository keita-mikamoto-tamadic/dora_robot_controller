#include "rad_converter.hpp"
#include <cmath>

namespace rad_converter {

static constexpr double TWO_PI = 2.0 * M_PI;

double rev_to_rad(double rev) {
    return rev * TWO_PI;
}

double rad_to_rev(double rad) {
    return rad / TWO_PI;
}

}  // namespace rad_converter
