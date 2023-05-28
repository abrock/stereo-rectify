#ifndef POINT3D_H
#define POINT3D_H

#include <opencv2/core.hpp>

class Point3D;

class Observation {
public:
    cv::KeyPoint pt;
    int idx;

    std::shared_ptr<Point3D> pt3d;

    static double distKP(cv::KeyPoint const& a, cv::KeyPoint const& b);
};

class Point3D
{
public:
    Point3D();

    cv::Vec3d loc;

    std::vector<std::shared_ptr<Observation> > observations;
};

#endif // POINT3D_H
