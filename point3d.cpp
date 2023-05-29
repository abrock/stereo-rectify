#include "point3d.h"

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

double Point3D::triangulate() {
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
    }
    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = true;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    std::cout << "###### Solving triangulation problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    return summary.final_cost / problem.NumResidualBlocks();
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
                new ceres::CauchyLoss(cam->scale),
                pt3d->loc.val,
                cam->extr.rot.val,
                cam->extr.loc.val,
                cam->c.val,
                &cam->f
                );
}

double Observation::distKP(const cv::KeyPoint &a, const cv::KeyPoint &b) {
    return cv::norm(a.pt - b.pt)
            + std::abs(a.angle - b.angle)
            + std::abs(a.response - b.response)
            + std::abs(a.octave - b.octave)
            + std::abs(a.size - b.size);
}
