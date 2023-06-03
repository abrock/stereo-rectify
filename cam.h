#ifndef CAM_H
#define CAM_H

#include <QObject>
#include <vector>

#include <opencv2/core.hpp>

#include <ceres/ceres.h>

#include "extr.h"
#include "point3d.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

class Observation;

class Cam : public QObject
{
    Q_OBJECT
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

    double f_mm = -1;
    double crop_factor = 1;

    std::string name;

    enum class Projection {rectilinear, equidistant};

    static Projection str2type(std::string str);

    static std::string type2str(Projection const type);

    void setProjection(std::string const& str);

    Q_INVOKABLE void setProjection(QString const& str);

    std::vector<std::shared_ptr<Observation>> observations;

    std::string fn;

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
    void setFocal(const double _f_mm, double const _crop_factor = 1);

    Q_INVOKABLE void setFocal(QString const& str);

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
            const T * const _c,
            const T * const _f,
            T & res_x,
            T & res_y) const {
        switch (proj) {
        case Projection::rectilinear:
            res_x = pt[0]/pt[2]*_f[0] + _c[0];
            res_y = pt[1]/pt[2]*_f[0] + _c[1];
            return true;
        case Projection::equidistant:
            res_x = pt[0]/pt[2];
            res_y = pt[1]/pt[2];
            T const r = ceres::sqrt(res_x*res_x + res_y*res_y);
            T const factor = ceres::atan(r)/r;
            res_x = res_x*factor*_f[0] + _c[0];
            res_y = res_y*factor*_f[0] + _c[1];
            return true;
        }
        return false;
    }

    template<class T>
    bool project(const T * const pt, T& res_x, T& res_y) const {
        T const _c[2] = {T(c[0]), T(c[1])};
        T const _f(f);
        return project(pt, _c, &_f, res_x, res_y);
    }

    void setIntrinsicsConstant(ceres::Problem& problem);
    void setExtrinsicsConstant(ceres::Problem& problem);
    void setConstant(ceres::Problem& problem);

    cv::Vec3d unproject(cv::Point2d const& src_px, const double z = 1'000) const;

    Extr extr;


    cv::Vec2d getCenteredPoint(const cv::KeyPoint &pt) const;

    cv::Mat map2target(std::shared_ptr<Cam> target);

    std::string print() const;

    void plotResiduals() const;

    bool validSrcPx(const cv::Point2d &src_px) const;

    cv::Mat2f simCamMap(std::shared_ptr<Cam> target);
};

#endif // CAM_H
