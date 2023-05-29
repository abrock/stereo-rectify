#include "misc.h"

#include <cmath>

namespace Misc {

double pos_fmod(const double num, const double mod) {
    return std::fmod(std::fmod(num, mod) + mod, mod);
}

cv::Mat merge_red_cyan(const cv::Mat &a, const cv::Mat &b) {
    cv::Mat result;
    cv::merge(std::vector<cv::Mat>{b, b, a}, result);
    return result;
}


} // namespace Misc
