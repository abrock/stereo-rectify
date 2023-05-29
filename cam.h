#ifndef CAM_H
#define CAM_H

#include <opencv2/core.hpp>

#include <ceres/ceres.h>

#include "extr.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

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
     * @brief The "scale" is 1% or the diagonal in px
     */
    double scale = -1;

    /**
     * @brief The length of the image diagonal in px
     */
    double diag = -1;

    void setSize(cv::Size const& s);

    void setImg(const std::shared_ptr<cv::Mat> &_img);

    std::vector<cv::KeyPoint> key_pts;
    cv::Mat descriptors;
    void computeKeyPoints();

    /**
     * @brief Projection function.
     */
    Projection proj = Projection::rectilinear;

    std::shared_ptr<cv::Mat> img;

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

    Extr extr;


    cv::Vec2d getCenteredPoint(const cv::KeyPoint &pt) const;

};

#endif // CAM_H
