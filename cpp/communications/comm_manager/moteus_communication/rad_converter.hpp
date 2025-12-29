#ifndef RAD_CONVERTER_HPP
#define RAD_CONVERTER_HPP

namespace rad_converter {

// Convert revolutions to radians: rad = rev * 2π
double rev_to_rad(double rev);

// Convert radians to revolutions: rev = rad / 2π
double rad_to_rev(double rad);

}  // namespace rad_converter

#endif  // RAD_CONVERTER_HPP
