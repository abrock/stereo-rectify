#ifndef EXTR_H
#define EXTR_H

#include <opencv2/core.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

/**
 * @brief The Extr class stores camera extrinsics
 */
class Extr
{
public:
    Extr();

    cv::Vec3d world2cam(cv::Vec3d const& in) const;
    cv::Vec3d cam2world(cv::Vec3d const& in) const;

    template<class T>
    static void world2cam(
            T const * const pt,
            T const * const loc,
            T const * const rot,
            T * res
            ) {
        for (size_t ii = 0; ii < 3; ++ii) {
            res[ii] = pt[ii] + loc[ii];
        }
        ceres::AngleAxisRotatePoint(rot, res, res);
    }

    template<class T>
    static void cam2world(
            T const * const pt,
            T const * const loc,
            T const * const rot,
            T * res
            ) {
        T const inverse_rot[3] = {-rot[0], -rot[1], -rot[2]};
        ceres::AngleAxisRotatePoint(inverse_rot, pt, res);
        for (size_t ii = 0; ii < 3; ++ii) {
            res[ii] -= loc[ii];
        }
    }

    void setConstant(ceres::Problem& problem);

    cv::Vec3d loc{0,0,0};
    cv::Vec3d rot{0,0,0};
};

#endif // EXTR_H
