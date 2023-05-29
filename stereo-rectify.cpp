#include <iostream>

#include <tclap/CmdLine.h>

#include <opencv2/highgui.hpp>

#include "calib.h"
#include "cam.h"

#include <glog/logging.h>

int main(int argc, char ** argv) {
    TCLAP::CmdLine cmd("stereo rectification tool");

    TCLAP::ValueArg<std::string> left_img_arg("l", "left", "left input image", true, "", "left input image", cmd);
    TCLAP::ValueArg<std::string> right_img_arg("r", "right", "right input image", true, "", "right input image", cmd);

    TCLAP::ValueArg<double> focal_arg("f", "focal", "Focal length of the left image", true, 0, "Focal length of the left image", cmd);
    TCLAP::ValueArg<double> focal_right_arg("", "focal-r", "Focal length of the right image. Default is the focal length of the left image",
                                            false, -1, "Focal length", cmd);

    TCLAP::ValueArg<double> baseline_arg("b", "baseline", "Baseline: Distance between the two cameras.", false, 10, "Baseline", cmd);

    TCLAP::ValueArg<std::string> proj_arg("p", "projection", "Projection type, either rectilinear or equidistant", false, "rectilinear",
                                          "Projection type, either rectilinear or equidistant", cmd);

    TCLAP::ValueArg<std::string> proj_right_arg("", "projection-right",
                                                "Projection type of the right camera, either rectilinear or equidistant. Default is the projection type of the left camera",
                                                false, "rectilinear",
                                                "Projection type, either rectilinear or equidistant", cmd);


    cmd.parse(argc, argv);


    std::shared_ptr<cv::Mat> left = std::make_shared<cv::Mat>(cv::imread(left_img_arg.getValue(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH));
    CHECK_GT(left->rows, 0) << "left image " << left_img_arg.getValue() << " empty or unreadable";
    CHECK_GT(left->cols, 0) << "left image " << left_img_arg.getValue() << " empty or unreadable";

    std::shared_ptr<cv::Mat> right = std::make_shared<cv::Mat>(cv::imread(right_img_arg.getValue(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH));
    CHECK_GT(right->rows, 0) << "right image " << right_img_arg.getValue() << " empty or unreadable";
    CHECK_GT(right->cols, 0) << "right image " << right_img_arg.getValue() << " empty or unreadable";

    double const f = focal_arg.getValue();
    CHECK_GT(f, 0);

    double const f_r = focal_right_arg.getValue() > 0 ? focal_right_arg.getValue() : f;
    CHECK_GT(f_r, 0);

    double const b = baseline_arg.getValue();
    CHECK_GT(b, 0);

    std::shared_ptr<Cam> cam_l = std::make_shared<Cam>();
    cam_l->setImg(left);
    cam_l->setFocal(f);
    cam_l->setProjection(proj_arg.getValue());

    std::shared_ptr<Cam> cam_r = std::make_shared<Cam>();
    cam_r->setImg(right);
    cam_r->setFocal(f_r);
    cam_r->setProjection(proj_right_arg.isSet() ? proj_right_arg.getValue() : proj_arg.getValue());

    std::shared_ptr<Calib> calib = std::make_shared<Calib>();

    calib->cams.push_back(cam_l);
    calib->cams.push_back(cam_r);

    calib->computeMatches();

    std::cout << "Number of matches: " << calib->points.size() << std::endl;
    std::cout << "Match stats: " << std::endl
              << calib->matchStats() << std::endl;

    std::pair<double, double> const rot = calib->computeRotationMultiple(calib->cams[0], calib->cams[1]);
    std::cout << "Rotation: " << rot.first << ", rmse: " << rot.second << std::endl;

    calib->cams[1]->extr.loc = cv::Vec3d(b, 0, 0);
    calib->cams[1]->extr.rot = cv::Vec3d(0,0,rot.first);

    std::shared_ptr<Cam> target_cam = std::make_shared<Cam>();
    target_cam->setSize({3000, 2000});
    target_cam->setFocal(4);

    /*
    cv::imwrite(left_img_arg.getValue() + "-simple-rot.tif", calib->cams[0]->map2target(target_cam));
    cv::imwrite(right_img_arg.getValue() + "-simple-rot.tif", calib->cams[1]->map2target(target_cam));
    */

    calib->optimizeSFM();

    cv::imwrite(left_img_arg.getValue() + "-sfm-rot.tif", calib->cams[0]->map2target(target_cam));
    cv::imwrite(right_img_arg.getValue() + "-sfm-rot.tif", calib->cams[1]->map2target(target_cam));

    return EXIT_SUCCESS;
}
