#include "stereomanager.h"

#include "misc.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <ParallelTime/paralleltime.h>

#include <fmt/core.h>

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
        method = str2method(str.toStdString());
        std::cout << "Optimization method: " << str.toStdString() << std::endl;
        autoRun();
    }  catch (std::exception const& e) {
        std::cout << "Setting optimization method failed: " << std::endl << e.what() << std::endl;
    }
}

void StereoManager::setPreview(const QString &str) {
    preview = str.toLower().toStdString();
    autoRun();
}

void StereoManager::setCLAHE(const bool checked, const QString &_clip_limit, const QString &_grid_size) {
    use_clahe = checked;
    clahe_clip_limit = _clip_limit.toDouble();
    clahe_grid_size = _grid_size.toDouble();
    if (clahe_grid_size < 1) {
        std::cerr << "Warning: grid size < 1 makes no sense, setting it to 1" << std::endl;
        clahe_grid_size = 1;
    }
    Misc::print("New CLAHE settings: {}abled, clip limit: {:.3f}, grid size: {}\n", use_clahe ? "en" : "dis", clahe_clip_limit, clahe_grid_size);
    autoRun();
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
    cv::Mat2f map = cam->simCamMap(target);
    cv::remap(src, result, map, cv::noArray(), cv::INTER_LANCZOS4);
    return result;
}

void StereoManager::run() {
    std::string const img_name = "preview";
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
    optimize();
    Misc::println("Optimization: {}", t.print());

    std::cout << "Left cam:  " << cam_l->print() << std::endl;
    std::cout << "Right cam: " << cam_r->print() << std::endl;

    if ("left" == preview || "right" == preview) {
        std::shared_ptr<Cam> cam        = "left" == preview ?        cam_l :        cam_r;
        std::shared_ptr<Cam> target_cam = "left" == preview ? cam_target_l : cam_target_r;
        cv::Mat img = getRemapped(cam, target_cam);
        cv::imshow(img_name, img);
        cv::waitKey(1);
        last_preview = img;
        return;
    }

    t.start();
    cv::Mat img_l = getRemapped(cam_l, cam_target_l);
    cv::Mat img_r = getRemapped(cam_r, cam_target_r);
    Misc::println("Remapping: {}", t.print());

    t.start();
    cv::Mat red_cyan = Misc::merge_red_cyan(img_l, img_r);
    std::cout << "Merging: " << t.print() << std::endl;

    cv::imshow(img_name, red_cyan);
    last_preview = red_cyan;
    cv::waitKey(1);
}

void StereoManager::setAutoRun(const bool val) {
    auto_run = val;
    autoRun();
}
