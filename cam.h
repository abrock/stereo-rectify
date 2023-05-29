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
    double f = -1;

    enum class Projection {rectilinear, equidistant};

    static Projection str2type(std::string const& str);

    void setProjection(std::string const& str);

    /**
     * @brief The "scale" is 1% or the diagonal in px
     */
    double scale = -1;

    /**
     * @brief The length of the image diagonal in px
     */
    double diag = -1;

    void setSize(cv::Size const& s);

    /**
     * @brief setFocal computes the focal length in px from a given focal length in mm and crop factor.
     * Before calling this function setSize must have been called, either directly or by calling setImg.
     * @param f_mm
     * @param crop_factor
     */
    void setFocal(double f_mm, double const crop_factor = 1);

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

    cv::Vec3d unproject(cv::Point2d const& src_px, const double z = 1'000) const;

    Extr extr;


    cv::Vec2d getCenteredPoint(const cv::KeyPoint &pt) const;

    cv::Mat map2target(std::shared_ptr<Cam> target);

};

#endif // CAM_H
