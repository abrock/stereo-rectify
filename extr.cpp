#include "extr.h"

Extr::Extr()
{

}

cv::Vec3d Extr::world2cam(const cv::Vec3d &in) const {
    cv::Vec3d result;
    world2cam(in.val, loc.val, rot.val, result.val);
    return result;
}

cv::Vec3d Extr::cam2world(const cv::Vec3d &in) const {
    cv::Vec3d result;
    cam2world(in.val, loc.val, rot.val, result.val);
    return result;
}

void Extr::setConstant(ceres::Problem &problem) {
    problem.SetParameterBlockConstant(rot.val);
    problem.SetParameterBlockConstant(loc.val);
}
