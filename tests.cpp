#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <random>
#include <runningstats/runningstats.h>
namespace rs = runningstats;

#include "calib.h"
#include "cam.h"


TEST(Cam, project_unproject) {
    std::mt19937_64 engine(0xBEEBBEEB);
    int const width = 1200;
    int const height = 800;
    size_t const num_tests = 1'000;
    for (std::string const& type : {"rectilinear", "equidistant"}) {
        Cam cam;
        cam.setSize({width, height});
        cam.setFocal(40);
        cam.setProjection(type);

        std::cout << "Cam: " << cam.print() << std::endl;

        for (size_t test = 0; test < num_tests; ++test) {
            double const z = std::uniform_real_distribution<double>(100, 10'000)(engine);
            cv::Point2f const src_px(
                        std::uniform_int_distribution<int>(0, width)(engine),
                        std::uniform_int_distribution<int>(0, height)(engine)
                        );
            cv::Vec3d const unprojected = cam.unproject(src_px, z);
            ASSERT_NEAR(unprojected[2], z, 1e-6);
            cv::Vec2d const reprojected = cam.project(unprojected);
            ASSERT_NEAR(src_px.x, reprojected[0], 1e-4)
                    << "#test: " << test << " src: " << src_px << ", z " << z << ", unproj: " << unprojected << ", reproj: " << reprojected << ", type: " << type;
            ASSERT_NEAR(src_px.y, reprojected[1], 1e-4)
                    << "#test: " << test << " src: " << src_px << ", z " << z << ", unproj: " << unprojected << ", reproj: " << reprojected << ", type: " << type;
        }
    }
}

TEST(Extr, all) {
    std::mt19937_64 engine(0xBEEBBEEB);
    std::uniform_real_distribution<double> coord(-10'000, 10'000);
    std::uniform_real_distribution<double> rot(-M_PI, M_PI);
    for (size_t ii = 0; ii < 100'000; ++ii) {
        Extr extr;
        extr.loc = cv::Vec3d(
                    coord(engine),
                    coord(engine),
                    coord(engine)
                    );
        extr.rot = cv::Vec3d(
                    rot(engine),
                    rot(engine),
                    rot(engine)
                    );
        cv::Vec3d const orig(coord(engine), coord(engine), coord(engine));
        double const scale = std::max(cv::norm(orig), cv::norm(extr.loc));

        cv::Vec3d const trans1 = extr.cam2world(extr.world2cam(orig));
        cv::Vec3d const trans2 = extr.world2cam(extr.cam2world(orig));

        ASSERT_NEAR(orig[0], trans1[0], scale*1e-6);
        ASSERT_NEAR(orig[1], trans1[1], scale*1e-6);
        ASSERT_NEAR(orig[2], trans1[2], scale*1e-6);

        ASSERT_NEAR(orig[0], trans2[0], scale*1e-6);
        ASSERT_NEAR(orig[1], trans2[1], scale*1e-6);
        ASSERT_NEAR(orig[2], trans2[2], scale*1e-6);
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    std::cout << "RUN_ALL_TESTS return value: " << RUN_ALL_TESTS() << std::endl;
}
