#include "point3d.h"

#include <random>

Point3D::Point3D()
{

}

bool Point3D::findByCam(
        const std::shared_ptr<Cam> &c,
        std::shared_ptr<Observation> &result) {
    for (auto obs : observations) {
        if (c == obs->cam) {
            result = obs;
            return true;
        }
    }
    return false;
}

void Point3D::addSFMBlocks(ceres::Problem &problem) {
    for (auto obs : observations) {
        obs->addSFMBlock(problem);
    }
}

double Point3D::triangulateRANSAC(bool const verbose) {
    // If no previous result is present, run initial triangulation
    if (max_error < 0) {
        triangulate(verbose);
    }
    std::mt19937_64 engine{std::random_device()()};
    cv::Vec3d best_result = loc;
    double best_error = max_error;
    for (size_t ii = 0; ii < 20; ++ii) {
        double const z = std::uniform_real_distribution<double>(100, 3'000)(engine);
        double const x = std::uniform_real_distribution<double>(-z, z)(engine);
        double const y = std::uniform_real_distribution<double>(-z, z)(engine);
        loc = cv::Vec3d(x,y,z);
        triangulate(verbose);
        if (max_error < best_error && max_error >= 0) {
            best_error = max_error;
            best_result = loc;
        }
    }
    loc = best_result;
    max_error = best_error;
    return max_error;
}

double Point3D::triangulate(bool const verbose) {
    if (observations.size() < 2) {
        return -1;
    }
    if (cv::norm(loc) < 1e-6) {
        loc[1] = 10;
        loc[0] = 10;
        loc[2] = 1'000;
    }
    ceres::Problem problem;
    addSFMBlocks(problem);
    for (auto obs : observations) {
        obs->cam->setConstant(problem);
        if (obs->ptInCam()[2] <= 0) {
            return -1;
        }
    }
    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = verbose;
    ceres_opts.logging_type = ceres::LoggingType::SILENT;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    if (verbose)
        std::cout << "###### Solving triangulation problem ######" << std::endl;

    ceres::Solve(ceres_opts, &problem, &summary);

    if (verbose)
        std::cout << summary.FullReport() << std::endl;

    max_error = 0;
    for (auto obs : observations) {
        max_error = std::max(max_error, obs->error());
    }

    return max_error;
}

cv::Vec3d Point3D::inCam(std::shared_ptr<Cam> cam) const {
    cv::Vec3d result;
    Extr::world2cam(loc.val, cam->extr.loc.val, cam->extr.rot.val, result.val);
    return result;
}

struct SFMCost {
    cv::Point2f pt;
    std::shared_ptr<Cam> cam;

    template<class T>
    bool operator()(
            T const * const pt_world,
            T const * const rot,
            T const * const loc,
            T const * const c,
            T const * const f,
            T * residuals
            ) const {
        T pt_cam[3];
        Extr::world2cam(pt_world, loc, rot, pt_cam);
        if (pt_cam[2] < T(0)) {
            return false;
        }
        if (!cam->project(
                    pt_cam, c, f, residuals[0], residuals[1]
                    )) {
            return false;
        }
        residuals[0] -= T(pt.x);
        residuals[1] -= T(pt.y);

        return true;
    }

    static ceres::CostFunction* create(
            std::shared_ptr<Observation> obs,
            std::shared_ptr<Cam> cam
            ) {
        return new ceres::AutoDiffCostFunction<
                SFMCost,
                2, // 2 residuals: projection errors in x and y
                3, // 3 values for 3D world point
                3, // 3 values for camera rotation
                3, // 3 values for camera location
                2, // 2 values for camera principal point
                1 // 1 value for camera focal length
                >(new SFMCost{obs->pt.pt, cam});
    }
};

ceres::ResidualBlockId Observation::addSFMBlock(ceres::Problem &problem) {
    return problem.AddResidualBlock(
                SFMCost::create(shared_from_this(), cam),
                nullptr, // new ceres::CauchyLoss(cam->scale),
                pt3d->loc.val,
                cam->extr.rot.val,
                cam->extr.loc.val,
                cam->c.val,
                &cam->f
                );
}

cv::Vec3d Observation::ptInCam() const {
    return pt3d->inCam(cam);
}

cv::Vec2d Observation::project() const {
    return cam->project(ptInCam());
}

cv::Point2d Observation::projectPt() const {
    cv::Vec2d const proj = project();
    return {proj[0], proj[1]};
}

cv::Vec2d Observation::get() const {
    return cv::Vec2d(pt.pt.x, pt.pt.y);
}

double Observation::error() const {
    return cv::norm(project() - cv::Vec2d(pt.pt.x, pt.pt.y));
}

double Observation::distKP(const cv::KeyPoint &a, const cv::KeyPoint &b) {
    return cv::norm(a.pt - b.pt)
            + std::abs(a.angle - b.angle)
            + std::abs(a.response - b.response)
            + std::abs(a.octave - b.octave)
            + std::abs(a.size - b.size);
}
