#ifndef MISC_H
#define MISC_H

#include <opencv2/core.hpp>

#include <fmt/core.h>

#include <iostream>

namespace Misc {

double pos_fmod(double const num, double const mod);

cv::Mat merge_red_cyan(cv::Mat const& a, cv::Mat const& b);

std::string to_lower(std::string const& in);

template <typename S, typename... Args, typename Char = fmt::char_t<S>>
inline void print(const S& format_str, Args&&... args) {
    fmt::print(format_str, args...);
    std::fflush(stdout);
}

template <typename S, typename... Args, typename Char = fmt::char_t<S>>
inline void println(const S& format_str, Args&&... args) {
    fmt::print(format_str, args...);
    std::cout << std::endl;
}

cv::Point vec2pt(cv::Vec2d const& in);

}

#endif // MISC_H
