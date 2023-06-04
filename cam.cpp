#include "cam.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "extr.h"

#include "misc.h"

Cam::Cam()
{

}

Cam::Projection Cam::str2type(std::string str) {
    str = Misc::to_lower(str);
    if ("rectilinear" == str) {
        return Projection::rectilinear;
    }
    if ("equidistant" == str) {
        return Projection::equidistant;
    }
    throw std::runtime_error("Unknown projection type: \"" + str + "\"");
}

std::string Cam::type2str(const Cam::Projection type) {
    switch (type) {
    case Projection::rectilinear: return "rectilinear";
    case Projection::equidistant: return "equidistant";
    }
    throw std::runtime_error("Projection type conversion to string not implemented for the given type");
}

void Cam::setProjection(const std::string &str) {
    Projection const old_proj = proj;
    proj = str2type(str);
    if (proj != old_proj && manager) {
        manager->optimization_necessary = true;
    }
}

void Cam::setProjection(const QString &str) {
    try {
        setProjection(str.toLower().toStdString());
        std::cout << "Projection type: " << type2str(proj) << ", cam: " << name << std::endl;
        if (manager) {
            manager->autoRun();
        }
    }  catch (std::exception const& e) {
        std::cout << "Unable to set projection: " << std::endl << e.what() << std::endl;
    }
}

void Cam::setSize(const cv::Size &s) {
    size = s;
    diag = std::sqrt(s.area());
    scale = diag/100;
    c[0] = double(s.width-1)/2;
    c[1] = double(s.height-1)/2;
}

void Cam::setFocal(const double _f_mm, const double _crop_factor) {
    CHECK_GT(diag, 0);
    double const old_f = f;
    double const diag_mm = std::sqrt(36*36 + 24*24);
    f_mm = _f_mm;
    crop_factor = _crop_factor;
    f = f_mm * crop_factor * diag / diag_mm;
    if (std::abs(f - old_f) > 1e-4 && manager) {
        manager->optimization_necessary = true;
    }
}

void Cam::setFocal(const QString &str) {
    setFocal(str.toDouble(), crop_factor);
    std::cout << "New focal length: " << f_mm << " for cam " << name << std::endl;
    if (manager) {
        manager->autoRun();
    }
}

void Cam::setImg(const std::shared_ptr<cv::Mat> &_img) {
    setSize(_img->size());
    img = _img;
}

void Cam::computeKeyPoints() {
    int const n_features = 20'000;
    auto detector = cv::ORB::create(n_features);
    auto matcher = cv::xfeatures2d::BEBLID::create(1.0);
    std::vector<cv::KeyPoint> pts;
    detector->detect(*img, key_pts);
    matcher->compute(*img, key_pts, descriptors);
}



cv::Vec2d Cam::project(cv::Vec3d pt) {
    CHECK_GT(f, 0);
    cv::Vec2d result;
    project(pt.val, result[0], result[1]);
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

bool Cam::validSrcPx(const cv::Point2d &src_px) const {
    switch (proj) {
    case Projection::rectilinear:
        return true;
    case Projection::equidistant:
        cv::Vec2d result;
        result[0] = (src_px.x - c[0]) / f;
        result[1] = (src_px.y - c[1]) / f;
        double const r = cv::norm(result);
        return r < M_PI_2;
    }
    return false;
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
        double const r = std::min(cv::norm(result), M_PI_2 - 1e-6);
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

cv::Vec2d rotate(cv::Vec2d const& pt, cv::Vec2d const& center, double const cos, double const sin) {
    cv::Vec2d result = pt -center;
    result = cv::Vec2d(
                cos*result[0] - sin*result[1],
                sin*result[0] + cos*result[1]
                );
    return result+center;
}

cv::Vec3d rotate_z(cv::Vec3d const& pt, double const cos, double const sin) {
    return {
               cos*pt[0] - sin*pt[1],
               sin*pt[0] + cos*pt[1],
               pt[2]
           };
}

cv::Vec2d Cam::mapPointReverse(
        Cam& target,
        cv::Vec2d const& tgt,
        double const rot_c,
        double const rot_s) {
    double const fake_loc[3] = {0,0,0};
    cv::Point2d target_px(tgt[0],tgt[1]);
    cv::Vec3d target_3d = target.unproject(target_px);
    cv::Vec3d world_3d;
    Extr::cam2world(target_3d.val, fake_loc, target.extr.rot.val, world_3d.val);
    cv::Vec3d src_3d;
    Extr::world2cam(world_3d.val, fake_loc, extr.rot.val, src_3d.val);
    src_3d = rotate_z(src_3d, rot_c, rot_s);
    return project(src_3d);
}

cv::Vec2d Cam::mapPointReverse(Cam& target, cv::Vec2d const& tgt, double const added_rot) {
    return mapPointReverse(target, tgt, std::cos(added_rot), std::sin(added_rot));
}

cv::Vec2d Cam::mapPointForward(
        Cam &target,
        const cv::Vec2d &src,
        const double rot_c,
        const double rot_s) {
    double const fake_loc[3] = {0,0,0};
    cv::Point2d const src_px(src[0], src[1]);
    cv::Vec3d const src_3d = rotate_z(unproject(src_px), rot_c, -rot_s);
    cv::Vec3d world_3d;
    Extr::cam2world(src_3d.val, fake_loc, extr.rot.val, world_3d.val);
    cv::Vec3d tgt_3d;
    Extr::world2cam(world_3d.val, fake_loc, target.extr.rot.val, tgt_3d.val);
    return target.project(tgt_3d);
}

cv::Vec2d Cam::mapPointForward(Cam& target, cv::Vec2d const& tgt, double const added_rot) {
    return mapPointForward(target, tgt, std::cos(added_rot), std::sin(added_rot));
}

cv::Mat2f Cam::simCamMap(std::shared_ptr<Cam> target, double const added_rot) {
    cv::Mat2f map(target->size);

    double const cos = std::cos(M_PI*added_rot/180.0);
    double const sin = std::sin(M_PI*added_rot/180.0);

#pragma omp parallel for schedule(dynamic, 100)
    for (int yy = 0; yy < map.rows; ++yy) {
        for (int xx = 0; xx < map.cols; ++xx) {
            map[yy][xx] = mapPointReverse(*target, cv::Vec2d(xx,yy), cos, sin);
        }
    }
    return map;
}

cv::Mat Cam::map2target(std::shared_ptr<Cam> target) {
    cv::Mat result;
    cv::Mat2f map = simCamMap(target);

    cv::remap(*img, result, map, cv::noArray(), cv::INTER_LANCZOS4);
    return result;
}

std::string Cam::print() const {
    std::stringstream out;
    out << "f: " << f << ", size: " << size << ", type: " << type2str(proj) << ", extr: loc " << extr.loc << "(" << cv::norm(extr.loc) << "), rot " << extr.rot;
    return out.str();
}

void Cam::plotResiduals() const {
    cv::Mat img_grey;
    cv::cvtColor(*img, img_grey, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_grey, img_grey, cv::COLOR_GRAY2BGR);

    double const linewidth = diag / 1'000;
    for (auto obs : observations) {
        cv::circle(img_grey, obs->pt.pt, linewidth*2, cv::Scalar(255,0,0), linewidth/2, cv::LINE_AA);
    }
    for (auto obs : observations) {
        if (!obs->pt3d->used) {
            cv::line(img_grey, obs->projectPt(), obs->pt.pt, cv::Scalar(0,0,255), linewidth, cv::LINE_AA);
        }
    }
    for (auto obs : observations) {
        if (obs->pt3d->used) {
            cv::line(img_grey, obs->projectPt(), obs->pt.pt, cv::Scalar(0,255,0), linewidth, cv::LINE_AA);
        }
    }

    cv::imwrite(fn + "-proj.tif", img_grey);
}
