#ifndef CALIB_H
#define CALIB_H

#include <vector>

#include "cam.h"
#include "point3d.h"

class Calib {
    std::shared_ptr<Point3D> findOrMake(int const idx, const cv::KeyPoint &kp);
public:
    Calib();

    std::vector<std::shared_ptr<Cam> > cams;

    void computeMatches();

    std::vector<std::shared_ptr<Point3D>> points;
    std::string matchStats();

    std::pair<double, double> computeRotation(std::shared_ptr<Cam> cam1, std::shared_ptr<Cam> cam2) const;
};

#endif // CALIB_H
