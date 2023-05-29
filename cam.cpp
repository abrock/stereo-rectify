#include "cam.h"

#include <opencv2/highgui.hpp>

Cam::Cam()
{

}

void Cam::setSize(const cv::Size &s) {
    size = s;
    diag = std::sqrt(s.area());
    scale = diag/100;
    c[0] = double(s.width-1)/2;
    c[1] = double(s.height-1)/2;
}

void Cam::setImg(const std::shared_ptr<cv::Mat> &_img) {
    setSize(_img->size());
    img = _img;
}

void Cam::computeKeyPoints() {
    int const n_features = 1'000;
    auto detector = cv::ORB::create(n_features);
    auto matcher = cv::xfeatures2d::BEBLID::create(1.0);
    std::vector<cv::KeyPoint> pts;
    detector->detect(*img, key_pts);
    matcher->compute(*img, key_pts, descriptors);
}

template<class T>
bool Cam::project(T const * const pt,
                  T const * const c,
                  T const * const f,
                  T & res_x,
                  T & res_y) {
    switch (proj) {
    case Projection::rectilinear:
        res_x = pt[0]/pt[2]*f[0] + c[0];
        res_y = pt[1]/pt[2]*f[0] + c[1];
        return true;
    case Projection::equidistant:
        res_x = pt[0]/pt[2];
        res_y = pt[1]/pt[2];
        T const r = ceres::sqrt(res_x*res_x + res_y*res_y);
        T const factor = ceres::atan(r)/r;
        res_x = res_x*factor*f[0] + c[0];
        res_y = res_y*factor*f[0] + c[1];
        return true;
    }
    return false;
}

cv::Vec2d Cam::project(cv::Vec3d pt) {
    cv::Vec2d result;
    project(pt.val, c.val, &f, result[0], result[1]);
    return result;
}

cv::Vec2d Cam::getCenteredPoint(const cv::KeyPoint &pt) const {
    cv::Vec2d result(pt.pt.x, pt.pt.y);
    result -= c;
    result *= 2.0/diag;
    return result;
}
