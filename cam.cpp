#include "cam.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "extr.h"

Cam::Cam()
{

}

Cam::Projection Cam::str2type(const std::string &str) {
    if ("rectilinear" == str) {
        return Projection::rectilinear;
    }
    if ("equidistant" == str) {
        return Projection::equidistant;
    }
    throw std::runtime_error("Unknown projection type " + str);
}

void Cam::setProjection(const std::string &str) {
    proj = str2type(str);
}

void Cam::setSize(const cv::Size &s) {
    size = s;
    diag = std::sqrt(s.area());
    scale = diag/100;
    c[0] = double(s.width-1)/2;
    c[1] = double(s.height-1)/2;
}

void Cam::setFocal(double f_mm, const double crop_factor) {
    f_mm *= crop_factor; // Do computations as if we had a small frame sensor
    CHECK_GT(diag, 0);
    double const diag_mm = std::sqrt(36*36 + 24*24);
    f = f_mm * diag / diag_mm;
}

void Cam::setImg(const std::shared_ptr<cv::Mat> &_img) {
    setSize(_img->size());
    img = _img;
}

void Cam::computeKeyPoints() {
    int const n_features = 10'000;
    auto detector = cv::ORB::create(n_features);
    auto matcher = cv::xfeatures2d::BEBLID::create(1.0);
    std::vector<cv::KeyPoint> pts;
    detector->detect(*img, key_pts);
    matcher->compute(*img, key_pts, descriptors);
}



cv::Vec2d Cam::project(cv::Vec3d pt) {
    CHECK_GT(f, 0);
    cv::Vec2d result;
    project(pt.val, c.val, &f, result[0], result[1]);
    return result;
}

void Cam::setIntrinsicsConstant(ceres::Problem &problem) {
    for (double * block : {&f, c.val}) {
        problem.SetParameterBlockConstant(block);
    }
}

void Cam::setExtrinsicsConstant(ceres::Problem &problem) {
    extr.setConstant(problem);
}

void Cam::setConstant(ceres::Problem &problem) {
    setIntrinsicsConstant(problem);
    setExtrinsicsConstant(problem);
}

cv::Vec3d Cam::unproject(const cv::Point2d &src_px, double const z) const {
    cv::Vec3d result(0,0,0);
    switch (proj) {
    case Projection::rectilinear:
        result[0] = (src_px.x - c[0]) * z / f;
        result[1] = (src_px.y - c[1]) * z / f;
        break;
    case Projection::equidistant:
        result[0] = (src_px.x - c[0]) / f;
        result[1] = (src_px.y - c[1]) / f;
        double const r = cv::norm(result);
        double const factor = std::tan(r)/r;
        result *= factor * z;
        break;
    }
    result[2] = z;
    return result;
}

cv::Vec2d Cam::getCenteredPoint(const cv::KeyPoint &pt) const {
    cv::Vec2d result(pt.pt.x, pt.pt.y);
    result -= c;
    result *= 2.0/diag;
    return result;
}

cv::Mat Cam::map2target(std::shared_ptr<Cam> target) {
    cv::Mat result;
    cv::Mat2f map(target->size);

    double const fake_loc[3] = {0,0,0};

    for (int yy = 0; yy < map.rows; ++yy) {
        for (int xx = 0; xx < map.cols; ++xx) {
            cv::Point target_px(xx,yy);
            cv::Vec3d target_3d = target->unproject(target_px);
            cv::Vec3d world_3d;
            Extr::cam2world(target_3d.val, fake_loc, target->extr.rot.val, world_3d.val);
            cv::Vec3d src_3d;
            Extr::world2cam(world_3d.val, fake_loc, extr.rot.val, src_3d.val);
            cv::Vec2d src_px = project(src_3d);
            map(target_px) = src_px;
        }
    }

    cv::remap(*img, result, map, cv::noArray(), cv::INTER_LANCZOS4);

    return result;
}
