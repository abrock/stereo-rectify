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

void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

void trim(std::string &s) {
    rtrim(s);
    ltrim(s);
}

std::string ltrim_copy(std::string s) {
    ltrim(s);
    return s;
}

std::string rtrim_copy(std::string s) {
    rtrim(s);
    return s;
}

std::string trim_copy(std::string s) {
    trim(s);
    return s;
}


} // namespace Misc
