#include <iostream>

#include <tclap/CmdLine.h>

#include <opencv2/highgui.hpp>

#include <QApplication>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QtQuick>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "calib.h"
#include "cam.h"

#include <glog/logging.h>

#include "stereomanager.h"

int main(int argc, char ** argv) {
    TCLAP::CmdLine cmd("stereo rectification tool");

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    TCLAP::ValueArg<std::string> left_img_arg("l", "left", "left input image", true, "", "left input image", cmd);
    TCLAP::ValueArg<std::string> right_img_arg("r", "right", "right input image", true, "", "right input image", cmd);

    TCLAP::ValueArg<double> focal_arg("f", "focal", "Focal length of the left image", true, 0, "Focal length of the left image", cmd);
    TCLAP::ValueArg<double> focal_right_arg("", "focal-r", "Focal length of the right image. Default is the focal length of the left image",
                                            false, -1, "Focal length", cmd);

    TCLAP::ValueArg<double> baseline_arg("b", "baseline", "Baseline: Distance between the two cameras.", false, 10, "Baseline", cmd);

    TCLAP::ValueArg<std::string> proj_arg("p", "projection", "Projection type, either rectilinear or equidistant", false, "rectilinear",
                                          "Projection type, either rectilinear or equidistant", cmd);

    TCLAP::SwitchArg interactive_arg("i", "interactive", "Show GUI for interactive mode", cmd);

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

    std::shared_ptr<StereoManager> manager = std::make_shared<StereoManager>();
    std::shared_ptr<Cam> cam_l = manager->cam_l = std::make_shared<Cam>();
    cam_l->setImg(left);
    cam_l->fn = left_img_arg.getValue();
    cam_l->name = "left";
    cam_l->setFocal(f);
    cam_l->setProjection(proj_arg.getValue());

    std::shared_ptr<Cam> cam_r = manager->cam_r = std::make_shared<Cam>();
    cam_r->setImg(right);
    cam_r->fn = right_img_arg.getValue();
    cam_r->name = "right";
    cam_r->setFocal(f_r);
    cam_r->setProjection(proj_right_arg.isSet() ? proj_right_arg.getValue() : proj_arg.getValue());

    std::shared_ptr<Cam> cam_target_l = manager->cam_target_l = std::make_shared<Cam>();
    std::shared_ptr<Cam> cam_target_r = manager->cam_target_r = std::make_shared<Cam>();
    cam_target_l->setSize(cam_l->size);
    cam_target_r->setSize(cam_r->size);
    cam_target_l->setFocal(cam_l->f_mm, cam_l->crop_factor);
    cam_target_r->setFocal(cam_l->f_mm, cam_l->crop_factor);

    std::shared_ptr<Calib> calib = manager->calib = std::make_shared<Calib>();

    calib->cams.push_back(cam_l);
    calib->cams.push_back(cam_r);

    calib->computeMatches();

    std::cout << "Number of matches: " << calib->points.size() << std::endl;
    std::cout << "Match stats: " << std::endl
              << calib->matchStats() << std::endl;

    std::cout << "Cams: " << std::endl << calib->printCams() << std::endl;

    manager->optimize();

    std::cout << "Cams: " << std::endl << calib->printCams() << std::endl;

    if (!interactive_arg.getValue()) {
        std::shared_ptr<Cam> target_cam = std::make_shared<Cam>();
        target_cam->setSize(cam_l->size);
        target_cam->setFocal(4);
        if (cam_l->proj == Cam::Projection::rectilinear) {
            target_cam->setFocal(f);
        }

        /*
    cv::imwrite(left_img_arg.getValue() + "-simple-rot.tif", calib->cams[0]->map2target(target_cam));
    cv::imwrite(right_img_arg.getValue() + "-simple-rot.tif", calib->cams[1]->map2target(target_cam));
    */

        /*
    calib->refineOrientationSimple(calib->cams[0], calib->cams[1]);
    std::cout << "Cams: " << std::endl << calib->printCams() << std::endl;

    cv::imwrite(left_img_arg.getValue() + "-simple-orientation.tif", calib->cams[0]->map2target(target_cam));
    cv::imwrite(right_img_arg.getValue() + "-simple-orientation.tif", calib->cams[1]->map2target(target_cam));
    */
        /*
    for (size_t ii = 0; ii < 5; ++ii) {
        calib->optimizeSFM(left_img_arg.getValue() + "-it-" + std::to_string(ii));
        std::cout << "Cams: " << std::endl << calib->printCams() << "t: " << target_cam->print() << std::endl;
    }

    calib->plotResiduals();
    cv::imwrite(left_img_arg.getValue() + "-sfm-rot.tif", calib->cams[0]->map2target(target_cam));
    cv::imwrite(right_img_arg.getValue() + "-sfm-rot.tif", calib->cams[1]->map2target(target_cam));
    */

        calib->optimizeStereoDirect(cam_l, cam_r, target_cam);
        std::cout << "Cams: " << std::endl << calib->printCams() << "t: " << target_cam->print() << std::endl;
        Calib::saveStereoImages(cam_l, cam_r, target_cam, target_cam, "single-target");

        calib->optimizeStereoDirect2Cams(cam_l, cam_r, cam_target_l, cam_target_r);
        std::cout << "Cams: " << std::endl << calib->printCams()
                  << "tl: " << cam_target_l->print() << std::endl
                  << "tr: " << cam_target_r->print() << std::endl;
        Calib::saveStereoImages(cam_l, cam_r, cam_target_l, cam_target_r, "two-targets");

        return EXIT_SUCCESS;
    }

    QApplication app(argc, argv);
    app.setOrganizationName("example");
    app.setOrganizationDomain("example.org");
    app.setApplicationName("Stereo Rectifier");

    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.rootContext()->setContextProperty("cam_l", &*cam_l);
    engine.rootContext()->setContextProperty("cam_r", &*cam_r);
    engine.rootContext()->setContextProperty("manager", &*manager);

    engine.load(url);

    return app.exec();
}
