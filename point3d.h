#ifndef POINT3D_H
#define POINT3D_H

#include <opencv2/core.hpp>

#include "cam.h"

class Point3D;
class Cam;

class Observation : public std::enable_shared_from_this<Observation> {
public:
    cv::KeyPoint pt;
    int idx;

    std::shared_ptr<Point3D> pt3d;

    std::shared_ptr<Cam> cam;

    static double distKP(cv::KeyPoint const& a, cv::KeyPoint const& b);
    ceres::ResidualBlockId addSFMBlock(ceres::Problem &problem);

    cv::Vec3d ptInCam() const;

    cv::Vec2d project() const;
    cv::Point2d projectPt() const;

    cv::Vec2d get() const;

    double error() const;
};

class Point3D
{
public:
    Point3D();

    cv::Vec3d loc;

    double max_error = -1;

    bool used = false;

    std::vector<std::shared_ptr<Observation> > observations;

    bool findByCam(std::shared_ptr<Cam> const& c, std::shared_ptr<Observation>& result);

    void addSFMBlocks(ceres::Problem& problem);
    double triangulate(const bool verbose = false);

    cv::Vec3d inCam(std::shared_ptr<Cam> cam) const;
    double triangulateRANSAC(const bool verbose = false);
};

#endif // POINT3D_H
