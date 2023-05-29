#ifndef POINT3D_H
#define POINT3D_H

#include <opencv2/core.hpp>

#include "cam.h"

class Point3D;

class Observation {
public:
    cv::KeyPoint pt;
    int idx;

    std::shared_ptr<Point3D> pt3d;

    std::shared_ptr<Cam> cam;

    static double distKP(cv::KeyPoint const& a, cv::KeyPoint const& b);
};

class Point3D
{
public:
    Point3D();

    cv::Vec3d loc;

    std::vector<std::shared_ptr<Observation> > observations;

    bool findByCam(std::shared_ptr<Cam> const& c, std::shared_ptr<Observation>& result);
};

#endif // POINT3D_H
