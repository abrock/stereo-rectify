#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <random>
#include <runningstats/runningstats.h>
namespace rs = runningstats;

#include "calib.h"
#include "cam.h"

#include "misc.h"

double uni(double const max, std::mt19937_64& engine) {
    return std::uniform_real_distribution<double>(0, max)(engine);
}

double uni_symmetric(double const max, std::mt19937_64& engine) {
    return std::uniform_real_distribution<double>(-max, max)(engine);
}

cv::Vec3d random_rot(std::mt19937_64& engine) {
    return {
        uni_symmetric(.3, engine),
        uni_symmetric(.3, engine),
        uni_symmetric(M_PI, engine)
    };
}

void randomize_loc(Cam& cam, std::mt19937_64& engine) {
    cam.extr.loc = {
        uni_symmetric(100, engine),
        uni_symmetric(100, engine),
        uni_symmetric(100, engine)
    };
}

TEST(Cam, mapping) {
    std::shared_ptr<Cam> cam1 = std::make_shared<Cam>();
    std::shared_ptr<Cam> cam2 = std::make_shared<Cam>();

    cam1->setSize({1000,2000});
    cam2->setSize({2000,1000});

    cam1->setFocal(40, 1);
    cam2->setFocal(50, 1);

    std::mt19937_64 engine(0xBEEBBEEB);

    // Use random location for the camera to make sure it's ignored
    randomize_loc(*cam1, engine);
    randomize_loc(*cam2, engine);

    size_t const num_tests = 1'000;
    double rotation = 0;
    // Simple test without any rotation
    for (size_t ii = 0; ii < num_tests; ++ii) {
        rotation = 0;
        cv::Vec2d const src_pt(uni(cam1->size.width, engine), uni(cam1->size.height, engine));
        cv::Vec2d const tgt_pt = cam1->mapPointForward(*cam2, src_pt, rotation);
        cv::Vec2d const src_remapped = cam1->mapPointReverse(*cam2, tgt_pt, rotation);
        for (size_t jj = 0; jj < 2; ++jj) {
            ASSERT_NEAR(src_pt[jj], src_remapped[jj], 1e-4) << "#" << ii << ", src: " << src_pt << ", tgt: " << tgt_pt << ", remapped: " << src_remapped;
        }
    }
    // Using the added rotation
    for (size_t ii = 0; ii < num_tests; ++ii) {
        rotation = uni(360, engine);
        cv::Vec2d const src_pt(uni(cam1->size.width, engine), uni(cam1->size.height, engine));
        cv::Vec2d const tgt_pt = cam1->mapPointForward(*cam2, src_pt, rotation);
        cv::Vec2d const src_remapped = cam1->mapPointReverse(*cam2, tgt_pt, rotation);
        for (size_t jj = 0; jj < 2; ++jj) {
            ASSERT_NEAR(src_pt[jj], src_remapped[jj], 1e-4) << "#" << ii << ", src: " << src_pt << ", tgt: " << tgt_pt << ", remapped: " << src_remapped;
        }
    }
    // Only random orientation
    for (size_t ii = 0; ii < num_tests; ++ii) {
        rotation = 0;
        cam1->extr.rot = random_rot(engine);
        cam2->extr.rot = random_rot(engine);
        cv::Vec2d const src_pt(uni(cam1->size.width, engine), uni(cam1->size.height, engine));
        cv::Vec2d const tgt_pt = cam1->mapPointForward(*cam2, src_pt, rotation);
        cv::Vec2d const src_remapped = cam1->mapPointReverse(*cam2, tgt_pt, rotation);
        for (size_t jj = 0; jj < 2; ++jj) {
            ASSERT_NEAR(src_pt[jj], src_remapped[jj], 1e-4) << "#" << ii << ", src: " << src_pt << ", tgt: " << tgt_pt << ", remapped: " << src_remapped;
        }
    }
    // Random orientation and additional rotation
    for (size_t ii = 0; ii < num_tests; ++ii) {
        rotation = uni(360, engine);
        cam1->extr.rot = random_rot(engine);
        cam2->extr.rot = random_rot(engine);
        cv::Vec2d const src_pt(uni(cam1->size.width, engine), uni(cam1->size.height, engine));
        cv::Vec2d const tgt_pt = cam1->mapPointForward(*cam2, src_pt, rotation);
        cv::Vec2d const src_remapped = cam1->mapPointReverse(*cam2, tgt_pt, rotation);
        for (size_t jj = 0; jj < 2; ++jj) {
            ASSERT_NEAR(src_pt[jj], src_remapped[jj], 1e-4) << "#" << ii << ", src: " << src_pt << ", tgt: " << tgt_pt << ", remapped: " << src_remapped;
        }
    }
}

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

TEST(Cam, str2type) {
    ASSERT_EQ(Cam::Projection::rectilinear, Cam::str2type("Rectilinear"));
    ASSERT_EQ(Cam::Projection::equidistant, Cam::str2type("Equidistant"));

    ASSERT_EQ("rectilinear", Cam::type2str(Cam::Projection::rectilinear));
    ASSERT_EQ("equidistant", Cam::type2str(Cam::Projection::equidistant));
}

TEST(Misc, EulerAnglesToAngleAxis) {
    cv::Vec3d axis;

    axis = Misc::EulerAnglesToAngleAxis(0,0,0);
    EXPECT_NEAR(cv::norm(axis), 0, 1e-10) << axis;

    axis = Misc::EulerAnglesToAngleAxis(0,0,90);
    EXPECT_NEAR(cv::norm(axis), M_PI_2, 1e-10) << axis << std::endl;
    EXPECT_NEAR(axis[2], -M_PI_2, 1e-10) << axis << std::endl;

    axis = Misc::EulerAnglesToAngleAxis(0,90,0);
    EXPECT_NEAR(cv::norm(axis), M_PI_2, 1e-10) << axis << std::endl;
    EXPECT_NEAR(axis[1], -M_PI_2, 1e-10) << axis << std::endl;

    axis = Misc::EulerAnglesToAngleAxis(90,0,0);
    EXPECT_NEAR(cv::norm(axis), M_PI_2, 1e-10) << axis << std::endl;
    EXPECT_NEAR(axis[0], -M_PI_2, 1e-10) << axis << std::endl;

}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    std::cout << "RUN_ALL_TESTS return value: " << RUN_ALL_TESTS() << std::endl;
}
