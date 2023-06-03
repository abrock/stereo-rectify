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
    }  catch (std::exception const& e) {
        std::cout << "Setting optimization method failed: " << std::endl << e.what() << std::endl;
    }
}

void StereoManager::setPreview(const QString &str) {
    preview = str.toLower().toStdString();
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

void StereoManager::run() {
    std::string const img_name = "preview";
    static cv::Mat empty(cam_target_l->size, CV_8UC1, cv::Scalar(0,0,0));
    cv::imshow(img_name, empty);
    cv::waitKey(1);
    optimize();

    if ("left" == preview || "right" == preview) {
        std::shared_ptr<Cam> cam = "left" == preview ? cam_l : cam_r;
        return;
    }

    std::cout << "Left cam:  " << cam_l->print() << std::endl;
    std::cout << "Right cam: " << cam_r->print() << std::endl;

    ParallelTime t;
    std::cout << "Generating maps..." << std::flush;
    cv::Mat2f map_l = cam_l->simCamMap(cam_target_l);
    cv::Mat2f map_r = cam_r->simCamMap(cam_target_r);
    std::cout << "done: " << t.print() << std::endl;

    t.start();
    auto clahe = cv::createCLAHE(4.0, {8,8});

    cv::Mat img_orig_l_grey, img_orig_r_grey;
    cv::cvtColor(*cam_l->img, img_orig_l_grey, cv::COLOR_BGR2GRAY);
    cv::cvtColor(*cam_r->img, img_orig_r_grey, cv::COLOR_BGR2GRAY);

    cv::Mat img_orig_ce_l_grey, img_orig_ce_r_grey;
    clahe->apply(img_orig_l_grey, img_orig_ce_l_grey);
    clahe->apply(img_orig_r_grey, img_orig_ce_r_grey);
    std::cout << "Applying CLAHE: " << t.print() << std::endl;

    t.start();
    cv::Mat img_l, img_r, img_l_grey, img_r_grey, img_l_ce, img_r_ce;
    std::cout << "Remapping..." << std::flush;
    cv::remap(*cam_l->img, img_l, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(*cam_r->img, img_r, map_r, cv::noArray(), cv::INTER_LANCZOS4);

    cv::remap(img_orig_l_grey, img_l_grey, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(img_orig_r_grey, img_r_grey, map_r, cv::noArray(), cv::INTER_LANCZOS4);

    cv::remap(img_orig_ce_l_grey, img_l_ce, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(img_orig_ce_r_grey, img_r_ce, map_r, cv::noArray(), cv::INTER_LANCZOS4);
    std::cout << "done: " << t.print() << std::endl;

    t.start();
    cv::Mat red_cyan = Misc::merge_red_cyan(img_l_grey, img_r_grey);
    cv::Mat red_cyan_ce = Misc::merge_red_cyan(img_l_ce, img_r_ce);
    std::cout << "Merging: " << t.print() << std::endl;

    cv::imshow(img_name, red_cyan);
    cv::waitKey(1);
}
