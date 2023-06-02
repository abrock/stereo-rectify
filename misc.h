#ifndef MISC_H
#define MISC_H

#include <opencv2/core.hpp>

namespace Misc {

double pos_fmod(double const num, double const mod);

cv::Mat merge_red_cyan(cv::Mat const& a, cv::Mat const& b);

std::string to_lower(std::string const& in);

}

#endif // MISC_H
