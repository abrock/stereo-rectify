#ifndef MISC_H
#define MISC_H

#include <opencv2/core.hpp>

#include <fmt/core.h>

#include <iostream>

namespace Misc {

double pos_fmod(double const num, double const mod);

cv::Mat merge_red_cyan(cv::Mat const& a, cv::Mat const& b);

std::string to_lower(std::string const& in);

template <typename... Args>
inline void print(fmt::format_string<Args...> s, Args&&... args) {
    fmt::print(s, std::forward<Args>(args)...);
    std::fflush(stdout);
}

template <typename... Args>
inline void println(fmt::format_string<Args...> s, Args&&... args) {
    std::cout << fmt::format(s, std::forward<Args>(args)...) << std::endl;
}

cv::Point vec2pt(cv::Vec2d const& in);

cv::Vec3d EulerAnglesToAngleAxis(double const x, double const y, double const z);

}

#endif // MISC_H
