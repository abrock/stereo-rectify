#include "stereomanager.h"

#include "misc.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ParallelTime/paralleltime.h>

#include <fmt/core.h>

#include <filesystem>
namespace fs = std::filesystem;

StereoManager::Method StereoManager::str2method(std::string str) {
    str = Misc::to_lower(str);
    if ("simple" == str) {
        return Method::simple;
    }
    if ("1-cam" == str) {
        return Method::cam1;
    }
    if ("2-cam" == str) {
        return Method::cam2;
    }
    throw std::runtime_error("Optimization method \"" + str + "\" not implemented");
}

void StereoManager::setMethod(const QString &str) {
    try {
        Method const old_method = method;
        method = str2method(str.toStdString());
        std::cout << "Optimization method: " << str.toStdString() << std::endl;
        if (method != old_method) {
            optimization_necessary = true;
            autoRun();
        }
    }  catch (std::exception const& e) {
        std::cout << "Setting optimization method failed: " << std::endl << e.what() << std::endl;
    }
}

void StereoManager::setPreview(const QString &str) {
    std::string const old_preview = preview;
    preview = str.toLower().toStdString();
    if (preview != old_preview) {
        autoRun();
    }
}

void StereoManager::setCLAHE(const bool checked, const QString &_clip_limit, const QString &_grid_size) {
    bool changed = (checked != use_clahe);
    use_clahe = checked;
    double const old_clip_limit = clahe_clip_limit;
    double const old_grid_size = clahe_grid_size;
    clahe_clip_limit = _clip_limit.toDouble();
    clahe_grid_size = _grid_size.toDouble();
    if (clahe_grid_size < 1) {
        std::cerr << "Warning: grid size < 1 makes no sense, setting it to 1" << std::endl;
        clahe_grid_size = 1;
    }
    changed |= (std::abs(clahe_clip_limit - old_clip_limit) > 1e-6);
    changed |= (clahe_grid_size != old_grid_size);
    Misc::print("New CLAHE settings: {}abled, clip limit: {:.3f}, grid size: {}\n", use_clahe ? "en" : "dis", clahe_clip_limit, clahe_grid_size);
    if (changed) {
        autoRun();
    }
}

void StereoManager::optimize() {
    std::pair<double, double> const rot = calib->computeRotationMultiple(calib->cams[0], calib->cams[1]);
    std::cout << "Rotation: " << rot.first << ", rmse: " << rot.second << std::endl;
    cam_r->extr.loc = cv::Vec3d(baseline, 0, 0);
    cam_r->extr.rot = cv::Vec3d(0,0,rot.first);

    if (Method::simple == method) {
        return;
    }

    if (Method::cam1 == method) {
        calib->optimizeStereoDirect(cam_l, cam_r, cam_target_l);
        std::cout << "Cams: " << std::endl << calib->printCams() << "t: " << cam_target_l->print() << std::endl;
        return;
    }
    if (Method::cam2 == method) {
        calib->optimizeStereoDirect2Cams(cam_l, cam_r, cam_target_l, cam_target_r);
        std::cout << "Cams: " << std::endl << calib->printCams()
                  << "tl: " << cam_target_l->print() << std::endl
                  << "tr: " << cam_target_r->print() << std::endl;
        return;
    }

    throw std::runtime_error("Optimization method not implemented in StereoManager::optimize()");
}

void StereoManager::autoRun() {
    if (auto_run) {
        run();
    }
}

cv::Mat StereoManager::getGrey(cv::Mat const& input) {
    cv::Mat result;
    cv::cvtColor(input, result, cv::COLOR_BGR2GRAY);
    if (use_clahe) {
        auto clahe = cv::createCLAHE(clahe_clip_limit, {clahe_grid_size,clahe_grid_size});
        clahe->apply(result, result);
    }
    return result;
}

cv::Mat StereoManager::getRemapped(std::shared_ptr<Cam> cam, std::shared_ptr<Cam> target) {
    cv::Mat src = getGrey(*cam->img);
    cv::Mat result;
    cv::Mat2f map = cam->simCamMap(target, rotation);
    cv::remap(src, result, map, cv::noArray(), cv::INTER_LANCZOS4);
    return result;
}

cv::Size operator *(double const scale, cv::Size const& s) {
    return {int(std::round(scale*s.width)), int(std::round(scale*s.height))};
}

void StereoManager::save() {
    Misc::println("Left filename: {}", cam_l->fn);
    Misc::println("Right filename: {}", cam_r->fn);
    fs::path src_dir = fs::path(cam_l->fn).parent_path();
    std::string fn_l = fs::path(cam_l->fn).filename().string();
    std::string fn_r = fs::path(cam_r->fn).filename().string();
    std::string const tgt_fn = (src_dir / fs::path(fn_l + "-stereo-" + fn_r)).string();
    Misc::println("Target fn: {}", tgt_fn);
    cv::imwrite(tgt_fn, last_preview);
    Misc::println("Saving image done.");
}

void StereoManager::run() {
    ParallelTime total_time;
    std::string const img_name = "preview";

    for (auto c : {cam_target_l, cam_target_r}) {
        c->setSize(scale_result * cam_l->size);
        c->setFocal(target_focal, 1.0);
    }
    if (last_preview.empty()) {
        static cv::Mat empty(cam_target_l->size, CV_8UC1, cv::Scalar(0,0,0));
        cv::imshow(img_name, empty);
    }
    else {
        last_preview *= 0.5;
        cv::imshow(img_name, last_preview);
    }
    cv::waitKey(1);

    ParallelTime t;
    if (optimization_necessary) {
        optimize();
        optimization_necessary = false;
        Misc::println("Optimization: {}", t.print());
    }

    std::cout << "Left cam:  " << cam_l->print() << std::endl;
    std::cout << "Right cam: " << cam_r->print() << std::endl;

    if ("left" == preview || "right" == preview) {
        std::shared_ptr<Cam> cam        = "left" == preview ?        cam_l :        cam_r;
        std::shared_ptr<Cam> target_cam = "left" == preview ? cam_target_l : cam_target_r;
        cv::Mat img = getRemapped(cam, target_cam);
        cv::imshow(img_name, img);
        cv::waitKey(1);
        last_preview = img;
        Misc::println("Total time: {}", total_time.print());
        return;
    }

    t.start();
    cv::Mat img_l = getRemapped(cam_l, cam_target_l);
    cv::Mat img_r = getRemapped(cam_r, cam_target_r);
    Misc::println("Remapping: {}", t.print());

    t.start();
    cv::Mat img;
    if ("red-cyan" == preview) {
        img = Misc::merge_red_cyan(img_l, img_r);
    }
    if ("side-by-side" == preview) {
        cv::hconcat(std::vector<cv::Mat>{img_l, img_r}, img);
        for (std::shared_ptr<Observation> pt_l : cam_l->observations) {
            std::shared_ptr<Observation> pt_r;
            if (!pt_l->pt3d->findByCam(cam_r, pt_r)) {
                continue;
            }
            cv::Vec2d pt2d_l = cam_l->mapPointForward(*cam_target_l, pt_l->get(), rotation);
            cv::Vec2d pt2d_r = cam_r->mapPointForward(*cam_target_r, pt_r->get(), rotation);
            pt2d_r[0] += cam_target_l->size.width;
            cv::line(
                        img,
                        Misc::vec2pt(pt2d_l),
                        Misc::vec2pt(pt2d_r),
                        cv::Scalar(255,255,255),
                        std::max<int>(1.0, cam_target_l->size.height/1000),
                        cv::LINE_AA);
        }
    }
    std::cout << "Merging: " << t.print() << std::endl;

    cv::imshow(img_name, img);
    last_preview = img;
    cv::waitKey(1);
    Misc::println("Total time: {}", total_time.print());
}

void StereoManager::setScaleResult(const QString &str) {
    double const old_scale_result = scale_result;
    scale_result = std::min(5.0, std::max(.1, str.toDouble()));
    if (std::abs(scale_result - old_scale_result) > 1e-6) {
        autoRun();
    }
}

void StereoManager::setTargetFocal(const QString &str) {
    double const old_target_focal = target_focal;
    target_focal = std::max(.1, str.toDouble());
    if (std::abs(target_focal - old_target_focal) > 1e-6) {
        autoRun();
    }
}

void StereoManager::setRotation(const double val) {
    double const old_val = rotation;
    rotation = val;
    if (std::abs(old_val - rotation) > 1e-6) {
        autoRun();
    }
}

void StereoManager::setAutoRun(const bool val) {
    auto_run = val;
    autoRun();
}
