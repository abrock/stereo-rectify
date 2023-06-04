#include "misc.h"

#include <cmath>

#include <ceres/rotation.h>

namespace Misc {

double pos_fmod(const double num, const double mod) {
    return std::fmod(std::fmod(num, mod) + mod, mod);
}

cv::Mat merge_red_cyan(const cv::Mat &a, const cv::Mat &b) {
    cv::Mat result;
    cv::merge(std::vector<cv::Mat>{b, b, a}, result);
    return result;
}

std::string to_lower(const std::string &in) {
    std::string result(in);
    for (char& c : result) {
        c = tolower(c);
    }
    return result;
}

cv::Point vec2pt(const cv::Vec2d &in) {
    return {int(std::round(in[0])), int(std::round(in[1]))};
}

cv::Vec3d EulerAnglesToAngleAxis(const double x, const double y, const double z) {
    double const euler[3] = {x, y, z};
    double matrix[9];
    constexpr int row_stride = 3;
    ceres::EulerAnglesToRotationMatrix(euler, row_stride, matrix);
    cv::Vec3d result;
    ceres::RotationMatrixToAngleAxis<double>(matrix, result.val);
    return result;
}


} // namespace Misc
