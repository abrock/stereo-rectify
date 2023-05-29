#include "calib.h"

#include <map>

#include <opencv2/xfeatures2d.hpp>

#include "point3d.h"

#include "misc.h"

#include <runningstats/runningstats.h>
namespace rs = runningstats;

std::shared_ptr<Point3D> Calib::findOrMake(std::shared_ptr<Cam> const cam, const int idx, const cv::KeyPoint &kp) {
    for (auto& pt : points) {
        for (auto& obs : pt->observations) {
            if (cam == obs->cam
                    && idx == obs->idx
                    && Observation::distKP(kp, obs->pt) < 1e-6) {
                return pt;
            }
        }
    }
    auto result = std::make_shared<Point3D>();
    auto obs = std::make_shared<Observation>();
    obs->idx = idx;
    obs->pt3d = result;
    obs->pt = kp;
    obs->cam = cam;
    result->observations.push_back(obs);
    points.push_back(result);
    return result;
}

Calib::Calib()
{

}

void Calib::computeMatches() {
    for (auto& cam : cams) {
        cam->computeKeyPoints();
    }
    double const nn_match_ratio = 0.8;
    rs::QuantileStats<float> ratio_stats;
    rs::Stats2D<float> ratio_img;

    for (size_t ii = 1; ii < cams.size(); ++ii) {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector< std::vector<cv::DMatch> > nn_matches;
        matcher.knnMatch(cams[0]->descriptors, cams[ii]->descriptors, nn_matches, 2);
        //! [2-nn matching]

        //! [ratio test filtering]
        std::vector<cv::KeyPoint> matched1, matched2;
        for(size_t i = 0; i < nn_matches.size(); i++) {
            cv::DMatch first = nn_matches[i][0];
            float const dist1 = nn_matches[i][0].distance;
            float const dist2 = nn_matches[i][1].distance;

            cv::KeyPoint kp1 = cams[0]->key_pts[first.queryIdx];
            cv::KeyPoint kp2 = cams[ii]->key_pts[first.trainIdx];

            ratio_stats.push_unsafe(dist1/dist2);
            ratio_img.push_unsafe(dist1, dist2);

            if(dist1/dist2 < nn_match_ratio) {
                auto pt = findOrMake(cams[0], first.queryIdx, kp1);
                auto obs = std::make_shared<Observation>();
                obs->idx = first.trainIdx;
                obs->pt3d = pt;
                obs->pt = kp2;
                obs->cam = cams[ii];
                pt->observations.push_back(obs);
            }
        }
    }

    ratio_img.plotHist("dist-img", {-1,-1}, rs::HistConfig()
                       .setTitle("Distance image")
                       .setXLabel("Dist 1")
                       .setYLabel("Dist 2"));

    ratio_stats.plotHistAndCDF("dist-ratio", rs::HistConfig()
                               .setDataLabel("Distance ratio"));

    std::cout << "ratio_stats: " << ratio_stats.print() << std::endl;
}

std::string Calib::matchStats() {
    std::stringstream out;
    std::map<size_t, size_t> obs_per_pt_stats;
    for (auto & pt : points) {
        obs_per_pt_stats[pt->observations.size()]++;
    }
    for (auto& sample : obs_per_pt_stats) {
        out << sample.first << " images: " << sample.second << " points" << std::endl;
    }
    return out.str();
}

struct SimpleRotationCost {
    cv::Vec2d src;
    cv::Vec2d tgt;

    template<class T>
    bool operator()(T const * const rot, T * residuals) const {
        T const s = ceres::sin(rot[0]);
        T const c = ceres::cos(rot[0]);
        T const x = c * T(src[0]) - s * T(src[1]);
        T const y = s * T(src[0]) + c * T(src[1]);
        residuals[0] = x - T(tgt[0]);
        residuals[1] = y - T(tgt[1]);
        return true;
    }

    static ceres::CostFunction* create(cv::Vec2d const& _src, cv::Vec2d const& _dst) {
        return new ceres::AutoDiffCostFunction<SimpleRotationCost, 2, 1>(
                    new SimpleRotationCost{_src, _dst}
                    );
    }
};

std::pair<double, double> Calib::computeRotation(
        std::shared_ptr<Cam> cam1,
        std::shared_ptr<Cam> cam2,
        double const initial) const {
    double result = initial;
    ceres::Problem problem;
    for (auto pt : points) {
        std::shared_ptr<Observation> obs1, obs2;
        if (pt->findByCam(cam1, obs1) && pt->findByCam(cam2, obs2)) {
            problem.AddResidualBlock(
                        SimpleRotationCost::create(cam1->getCenteredPoint(obs1->pt), cam2->getCenteredPoint(obs2->pt)),
                        new ceres::CauchyLoss(.01),
                        &result);
        }
    }
    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = false;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    //std::cout << "###### Solving problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    //std::cout << summary.FullReport() << std::endl;
    result = Misc::pos_fmod(result, 2*M_PI);
    return {result, summary.final_cost / problem.NumResidualBlocks()};
}

std::pair<double, double> Calib::computeRotationMultiple(std::shared_ptr<Cam> cam1, std::shared_ptr<Cam> cam2) const {
    std::pair<double, double> best = computeRotation(cam1, cam2, 0);
    double const num_tries = 25;
    for (size_t ii = 1; ii < num_tries; ++ii) {
        std::pair<double, double> candidate = computeRotation(cam1, cam2, double(ii)*2.0*M_PI / num_tries);
        if (candidate.second < best.second && candidate.second >= 0) {
            best = candidate;
        }
    }
    return best;
}


