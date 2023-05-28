#ifndef CAM_H
#define CAM_H

#include <opencv2/core.hpp>

#include <ceres/ceres.h>

class Cam
{
public:
    Cam();

    /**
     * @brief principal point in px
     */
    cv::Vec2d c;

    /**
     * @brief Focal length in px
     */
    double f;

    enum class Projection {rectilinear, equidistant};

    /**
     * @brief Projection function.
     */
    Projection proj = Projection::rectilinear;

    /**
     * @brief Image width and height in px
     */
    cv::Size size;

    cv::Vec2d project(cv::Vec3d pt);

    template<class T>
    bool project(
            const T * const pt,
            const T * const c,
            const T * const f,
            T & res_x,
            T & res_y);
};

#endif // CAM_H
