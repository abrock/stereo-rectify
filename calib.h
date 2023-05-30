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

    /**
     * @brief The idea of optimizeStereoDirect is to keep the extrinsics of the left camera zero,
     * and optimize the orientation of the right camera and target camera to achieve the following objectives:
     * 1. Each rectified point pair has the same y-value on both points
     * 2. The x coordinate of the point in the right image is smaller than on the left image (large weight)
     * 3. The difference in x coordinates is small (small weight)
     * 4. Rotation around pitch and yaw of the right camera is minimized
     * @param cam_l
     * @param cam_r
     * @param cam_target
     */
    static void optimizeStereoDirect(
            std::shared_ptr<Cam> cam_l,
            std::shared_ptr<Cam> cam_r,
            std::shared_ptr<Cam> cam_target
            );

    static void saveStereoImages(
            std::shared_ptr<Cam> cam_l,
            std::shared_ptr<Cam> cam_r,
            std::shared_ptr<Cam> cam_target_l,
            std::shared_ptr<Cam> cam_target_r,
            const std::string &suffix = "");

    static void optimizeStereoDirect2Cams(
            std::shared_ptr<Cam> cam_l,
            std::shared_ptr<Cam> cam_r,
            std::shared_ptr<Cam> cam_target_l,
            std::shared_ptr<Cam> cam_target_r);
};

#endif // CALIB_H
