#ifndef CALIB_H
#define CALIB_H

#include <vector>

#include "cam.h"
#include "point3d.h"

#include <runningstats/runningstats.h>
namespace rs = runningstats;

class Calib {
    std::shared_ptr<Point3D> findOrMake(const std::shared_ptr<Cam> cam, int const idx, const cv::KeyPoint &kp);
public:
    Calib();

    std::vector<std::shared_ptr<Cam> > cams;

    void computeMatches();

    std::vector<std::shared_ptr<Point3D>> points;
    std::string matchStats();

    std::pair<double, double> computeRotation(std::shared_ptr<Cam> cam1, std::shared_ptr<Cam> cam2, const double initial) const;

    std::pair<double, double> computeRotationMultiple(std::shared_ptr<Cam> cam1, std::shared_ptr<Cam> cam2) const;

    static void refineOrientationSimple(std::shared_ptr<Cam> cam_ref, std::shared_ptr<Cam> cam_tgt);

    rs::QuantileStats<float> triangulation_error_stats;

    void triangulateAll(const std::string &plot_prefix = "");

    void optimizeSFM(const std::string &plot_prefix = "");

    std::string printCams() const;

    void plotResiduals() const;

    void triangulateAllRANSAC(const std::string &plot_prefix);
};

#endif // CALIB_H
