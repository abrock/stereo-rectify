#include "point3d.h"

Point3D::Point3D()
{

}

double Observation::distKP(const cv::KeyPoint &a, const cv::KeyPoint &b) {
    return cv::norm(a.pt - b.pt)
            + std::abs(a.angle - b.angle)
            + std::abs(a.response - b.response)
            + std::abs(a.octave - b.octave)
            + std::abs(a.size - b.size);
}
