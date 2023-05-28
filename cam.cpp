#include "cam.h"

Cam::Cam()
{

}

template<class T>
bool Cam::project(T const * const pt,
                  T const * const c,
                  T const * const f,
                  T & res_x,
                  T & res_y) {
    switch (proj)
    {
    case Projection::rectilinear:
        res_x = pt[0]/pt[2]*f[0] + c[0];
        res_y = pt[1]/pt[2]*f[0] + c[1];
        return true;
    case Projection::equidistant:
        res_x = pt[0]/pt[2];
        res_y = pt[1]/pt[2];
        T const r = ceres::sqrt(res_x*res_x + res_y*res_y);
        T const factor = ceres::atan(r)/r;
        res_x = res_x*factor*f[0] + c[0];
        res_y = res_y*factor*f[0] + c[1];
        return true;
    }

    return false;
}

cv::Vec2d Cam::project(cv::Vec3d pt) {
    cv::Vec2d result;
    project(pt.val, c.val, &f, result[0], result[1]);
    return result;
}
