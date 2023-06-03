#include "calib.h"

#include <map>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    cam->observations.push_back(obs);
    result->observations.push_back(obs);
    points.push_back(result);
    return result;
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
                obs->cam->observations.push_back(obs);
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

struct KeepNormCost {
    double distance;
    double weight;

    template<class T>
    bool operator()(T const * const vec, T * residual) const {
        residual[0] = ceres::sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]) - T(distance);
        return true;
    }

    static ceres::CostFunction* create(double const dist, double const w = 1'000) {
        return new ceres::AutoDiffCostFunction<KeepNormCost, 1, 3>(
                    new KeepNormCost{dist, w}
                    );
    }
};

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

struct SimpleOrientationCost {
    std::shared_ptr<Cam> cam_ref;
    std::shared_ptr<Cam> cam_tgt;

    cv::Point2f ref;
    cv::Point2f tgt;

    template<class T>
    bool operator()(T const * const rot_ref, T const* const rot_tgt, T * residuals) const {

        T const loc_zero[3] = {T(0),T(0),T(0)};

        double const z = 1'000;
        cv::Vec3d const unproj = cam_tgt->unproject(tgt, z);

        T const cam_tgt_pt[3] = {T(unproj[0]), T(unproj[1]), T(unproj[2])};

        T world_pt[3];
        Extr::cam2world(cam_tgt_pt, loc_zero, rot_tgt, world_pt);

        T cam_ref_pt[3];
        Extr::world2cam(world_pt, loc_zero, rot_ref, cam_ref_pt);

        cam_ref->project(cam_ref_pt, residuals[0], residuals[1]);

        residuals[0] -= T(ref.x);
        residuals[1] -= T(ref.y);

        return true;
    }

    static ceres::CostFunction* create(
            std::shared_ptr<Cam> cam_ref,
            std::shared_ptr<Cam> cam_tgt,
            cv::Point2f const& ref,
            cv::Point2f const& tgt
            ) {
        return new ceres::AutoDiffCostFunction<SimpleOrientationCost, 2, 3, 3>(new SimpleOrientationCost{cam_ref, cam_tgt, ref, tgt});
    }
};

void Calib::refineOrientationSimple(std::shared_ptr<Cam> cam_ref, std::shared_ptr<Cam> cam_tgt) {
    ceres::Problem problem;

    for (auto obs_ref : cam_ref->observations) {
        std::shared_ptr<Observation> obs_tgt;
        if (obs_ref->pt3d->findByCam(cam_tgt, obs_tgt)) {
            if (cam_tgt->validSrcPx(obs_tgt->pt.pt)) {
                problem.AddResidualBlock(
                            SimpleOrientationCost::create(cam_ref, cam_tgt, obs_ref->pt.pt, obs_tgt->pt.pt),
                            new ceres::CauchyLoss(cam_ref->scale),
                            cam_ref->extr.rot.val,
                            cam_tgt->extr.rot.val
                            );
            }
        }
    }
    problem.SetParameterBlockConstant(cam_ref->extr.rot.val);
    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = true;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    std::cout << "###### Solving simple orientation problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void Calib::triangulateAll(std::string const& plot_prefix) {
    triangulation_error_stats.clear();
    for (auto pt : points) {
        pt->triangulate();
        triangulation_error_stats.push_unsafe(pt->max_error);
    }
    if (!plot_prefix.empty()) {
        triangulation_error_stats.plotHistAndCDF(plot_prefix + "-triangulation-errors", rs::HistConfig()
                                                 .setDataLabel("Triangulation reprojection errors [px]")
                                                 .setLogX());
    }
}

void Calib::triangulateAllRANSAC(std::string const& plot_prefix) {
    int const num = 100;
    int const step = points.size() / num;
    std::cout << "Running Calib::triangulateAllRANSAC" << std::endl << std::string(num, '-') << std::endl;
    triangulation_error_stats.clear();

    for (size_t ii = 0; ii < points.size(); ++ii) {
        auto pt = points[ii];
        pt->triangulateRANSAC();
        triangulation_error_stats.push_unsafe(pt->max_error);
        if (ii % step == 0) {
            std::cout << "." << std::flush;
        }
    }
    std::cout << std::endl;
    if (!plot_prefix.empty()) {
        triangulation_error_stats.plotHistAndCDF(plot_prefix + "-triangulation-errors", rs::HistConfig()
                                                 .setDataLabel("Triangulation reprojection errors [px]")
                                                 .setLogX());
    }
}

struct StereoDirectCost {
    std::shared_ptr<Cam> cam_l;
    std::shared_ptr<Cam> cam_r;
    std::shared_ptr<Cam> cam_target;

    cv::Vec3d _pt_world_l;
    cv::Vec3d _pt_cam_r;

    StereoDirectCost(
            std::shared_ptr<Cam> _cam_l,
            std::shared_ptr<Cam> _cam_r,
            std::shared_ptr<Cam> _cam_target,

            cv::Point2d _pt_l,
            cv::Point2d _pt_r
            ) : cam_l(_cam_l), cam_r(_cam_r), cam_target(_cam_target)
    {
        // left cam is assumed to have zero extrinsics, therefore the world point is simply the cam point
        _pt_world_l = _cam_l->unproject(_pt_l);
        _pt_cam_r = _cam_r->unproject(_pt_r);
    }

    template<class T>
    bool operator()(T const * const rot_r, T const * const rot_target, T * residuals) const {
        bool success = true;
        T const zero[3] = {T(0), T(0), T(0)};
        T const pt_world_l[3] = {T(_pt_world_l[0]), T(_pt_world_l[1]), T(_pt_world_l[2])};
        T const pt_cam_r[3] =   {  T(_pt_cam_r[0]),   T(_pt_cam_r[1]),   T(_pt_cam_r[2])};
        T pt_world_r[3] = {T(0), T(0), T(0)};
        Extr::cam2world(pt_cam_r, zero, rot_r, pt_world_r);

        T pt_tgt_r[3] = {T(0), T(0), T(0)};
        Extr::world2cam(pt_world_r, zero, rot_target, pt_tgt_r);

        T pt_tgt_l[3] = {T(0), T(0), T(0)};
        Extr::world2cam(pt_world_l, zero, rot_target, pt_tgt_l);

        T pt2d_tgt_l[2] = {T(0), T(0)};
        success &= cam_target->project(pt_tgt_l, pt2d_tgt_l[0], pt2d_tgt_l[1]);

        T pt2d_tgt_r[2] = {T(0), T(0)};
        success &= cam_target->project(pt_tgt_r, pt2d_tgt_r[0], pt2d_tgt_r[1]);

        // Epipolar lines should be on the y axis.
        residuals[0] = pt2d_tgt_l[1] - pt2d_tgt_r[1];

        T x_weight(.01);
        // Points in the right camera should be further left than in the left camera.
        if (pt2d_tgt_r[0] > pt2d_tgt_l[0]) {
            x_weight *= T(10);
        }
        residuals[1] = x_weight*(pt2d_tgt_l[0] - pt2d_tgt_r[0]);

        return success;
    }

    static ceres::CostFunction* create(
            std::shared_ptr<Cam> _cam_l,
            std::shared_ptr<Cam> _cam_r,
            std::shared_ptr<Cam> _cam_target,

            cv::Point2d _pt_l,
            cv::Point2d _pt_r
            ) {
        return new ceres::AutoDiffCostFunction<StereoDirectCost, 2, 3, 3>(
                    new StereoDirectCost(_cam_l, _cam_r, _cam_target, _pt_l, _pt_r)
                    );
    }
};

/**
 * @brief The RotatePitchYawCost struct prevents orientation of a camera to change too much,
 * except for rotation around the roll axis which is considered ok.
 * It applies the provided rotation to the test point (0,0,weight) and punishes changes in x,y, and z.
 * Rotation around z (the roll axis) is not punished.
 */
struct RotatePitchYawCost {
    double weight;
    template<class T>
    bool operator()(T const * const rot, T * residuals) const {
        T const src[3] = {T(0), T(0), T(weight)};
        ceres::AngleAxisRotatePoint(rot, src, residuals);
        residuals[2] -= src[2];
        return true;
    }

    static ceres::CostFunction* create(double const _weight = 1'000) {
        return new ceres::AutoDiffCostFunction<RotatePitchYawCost, 3, 3>(
                    new RotatePitchYawCost{_weight}
                    );
    }
};

void Calib::optimizeStereoDirect(std::shared_ptr<Cam> cam_l, std::shared_ptr<Cam> cam_r, std::shared_ptr<Cam> cam_target) {
    cam_l->extr.loc = cam_l->extr.rot = cam_target->extr.loc = cam_target->extr.rot = cv::Vec3d(0,0,0);

    ceres::Problem problem;
    for (auto obs_l : cam_l->observations) {
        std::shared_ptr<Observation> obs_r;
        if (obs_l->pt3d->findByCam(cam_r, obs_r)) {
            problem.AddResidualBlock(
                        StereoDirectCost::create(cam_l, cam_r, cam_target, obs_l->pt.pt, obs_r->pt.pt),
                        new ceres::CauchyLoss(cam_l->scale),
                        cam_r->extr.rot.val,
                        cam_target->extr.rot.val
                        );
        }
    }
    double const rotate_cost = 10*problem.NumResidualBlocks();
    problem.AddResidualBlock(
                RotatePitchYawCost::create(rotate_cost),
                nullptr,
                cam_r->extr.rot.val);
    problem.AddResidualBlock(
                RotatePitchYawCost::create(rotate_cost),
                nullptr,
                cam_target->extr.rot.val);

    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = true;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    std::cout << "###### Solving Calib::optimizeStereoDirect problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    std::cout << summary.FullReport() << std::endl << summary.BriefReport() << std::endl;

    cam_r->extr.normalize();
    cam_target->extr.normalize();
}

struct StereoDirectCost2Cams {
    std::shared_ptr<Cam> cam_l;
    std::shared_ptr<Cam> cam_r;
    std::shared_ptr<Cam> cam_target;

    cv::Vec3d _pt_world_l;
    cv::Vec3d _pt_cam_r;

    StereoDirectCost2Cams(
            std::shared_ptr<Cam> _cam_l,
            std::shared_ptr<Cam> _cam_r,
            std::shared_ptr<Cam> _cam_target,

            cv::Point2d _pt_l,
            cv::Point2d _pt_r
            ) : cam_l(_cam_l), cam_r(_cam_r), cam_target(_cam_target)
    {
        // left cam is assumed to have zero extrinsics, therefore the world point is simply the cam point
        _pt_world_l = _cam_l->unproject(_pt_l);
        _pt_cam_r = _cam_r->unproject(_pt_r);
    }

    template<class T>
    bool operator()(T const * const rot_r, T const * const rot_target_l, T const * const rot_target_r, T * residuals) const {
        bool success = true;
        T const zero[3] = {T(0), T(0), T(0)};
        T const pt_world_l[3] = {T(_pt_world_l[0]), T(_pt_world_l[1]), T(_pt_world_l[2])};
        T const pt_cam_r[3] =   {  T(_pt_cam_r[0]),   T(_pt_cam_r[1]),   T(_pt_cam_r[2])};
        T pt_world_r[3] = {T(0), T(0), T(0)};
        Extr::cam2world(pt_cam_r, zero, rot_r, pt_world_r);

        T pt_tgt_r[3] = {T(0), T(0), T(0)};
        Extr::world2cam(pt_world_r, zero, rot_target_r, pt_tgt_r);

        T pt_tgt_l[3];
        Extr::world2cam(pt_world_l, zero, rot_target_l, pt_tgt_l);

        T pt2d_tgt_l[2] = {T(0), T(0)};
        success &= cam_target->project(pt_tgt_l, pt2d_tgt_l[0], pt2d_tgt_l[1]);

        T pt2d_tgt_r[2] = {T(0), T(0)};
        success &= cam_target->project(pt_tgt_r, pt2d_tgt_r[0], pt2d_tgt_r[1]);

        // Epipolar lines should be on the y axis.
        residuals[0] = pt2d_tgt_l[1] - pt2d_tgt_r[1];

        T x_weight(.01);
        // Points in the right camera should be further left than in the left camera.
        if (pt2d_tgt_r[0] > pt2d_tgt_l[0]) {
            x_weight *= T(10);
        }
        residuals[1] = x_weight*(pt2d_tgt_l[0] - pt2d_tgt_r[0]);

        return success;
    }

    static ceres::CostFunction* create(
            std::shared_ptr<Cam> _cam_l,
            std::shared_ptr<Cam> _cam_r,
            std::shared_ptr<Cam> _cam_target,

            cv::Point2d _pt_l,
            cv::Point2d _pt_r
            ) {
        return new ceres::AutoDiffCostFunction<StereoDirectCost2Cams, 2, 3, 3, 3>(
                    new StereoDirectCost2Cams(_cam_l, _cam_r, _cam_target, _pt_l, _pt_r)
                    );
    }
};

void Calib::optimizeStereoDirect2Cams(
        std::shared_ptr<Cam> cam_l,
        std::shared_ptr<Cam> cam_r,
        std::shared_ptr<Cam> cam_target_l,
        std::shared_ptr<Cam> cam_target_r
        ) {
    cam_l->extr.loc = cam_l->extr.rot
            = cam_target_l->extr.loc = cam_target_l->extr.rot
            = cam_target_r->extr.loc = cam_target_r->extr.rot = cv::Vec3d(0,0,0);

    ceres::Problem problem;
    for (auto obs_l : cam_l->observations) {
        std::shared_ptr<Observation> obs_r;
        if (obs_l->pt3d->findByCam(cam_r, obs_r)) {
            problem.AddResidualBlock(
                        StereoDirectCost2Cams::create(cam_l, cam_r, cam_target_l, obs_l->pt.pt, obs_r->pt.pt),
                        new ceres::CauchyLoss(cam_l->scale),
                        cam_r->extr.rot.val,
                        cam_target_l->extr.rot.val,
                        cam_target_r->extr.rot.val
                        );
        }
    }
    double const rotate_cost = 10*problem.NumResidualBlocks();
    problem.AddResidualBlock(
                RotatePitchYawCost::create(rotate_cost),
                nullptr,
                cam_r->extr.rot.val);
    problem.AddResidualBlock(
                RotatePitchYawCost::create(rotate_cost),
                nullptr,
                cam_target_l->extr.rot.val);
    problem.AddResidualBlock(
                RotatePitchYawCost::create(rotate_cost),
                nullptr,
                cam_target_r->extr.rot.val);

    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = true;
    ceres_opts.linear_solver_type = ceres::DENSE_QR;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    std::cout << "###### Solving Calib::optimizeStereoDirect2Cams problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    std::cout << summary.FullReport() << std::endl << summary.BriefReport() << std::endl;

    cam_r->extr.normalize();
    cam_target_l->extr.normalize();
    cam_target_r->extr.normalize();
}

void Calib::saveStereoImages(
        std::shared_ptr<Cam> cam_l,
        std::shared_ptr<Cam> cam_r,
        std::shared_ptr<Cam> cam_target_l,
        std::shared_ptr<Cam> cam_target_r,
        std::string const& suffix
        ) {

    std::cout << "Generating maps..." << std::flush;
    cv::Mat2f map_l = cam_l->simCamMap(cam_target_l);
    cv::Mat2f map_r = cam_r->simCamMap(cam_target_r);
    std::cout << "done." << std::endl;

    auto clahe = cv::createCLAHE(4.0, {8,8});

    cv::Mat img_orig_l_grey, img_orig_r_grey;
    cv::cvtColor(*cam_l->img, img_orig_l_grey, cv::COLOR_BGR2GRAY);
    cv::cvtColor(*cam_r->img, img_orig_r_grey, cv::COLOR_BGR2GRAY);

    cv::Mat img_orig_ce_l_grey, img_orig_ce_r_grey;
    clahe->apply(img_orig_l_grey, img_orig_ce_l_grey);
    clahe->apply(img_orig_r_grey, img_orig_ce_r_grey);

    cv::Mat img_l, img_r, img_l_grey, img_r_grey, img_l_ce, img_r_ce;
    std::cout << "Remapping..." << std::flush;
    cv::remap(*cam_l->img, img_l, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(*cam_r->img, img_r, map_r, cv::noArray(), cv::INTER_LANCZOS4);

    cv::remap(img_orig_l_grey, img_l_grey, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(img_orig_r_grey, img_r_grey, map_r, cv::noArray(), cv::INTER_LANCZOS4);

    cv::remap(img_orig_ce_l_grey, img_l_ce, map_l, cv::noArray(), cv::INTER_LANCZOS4);
    cv::remap(img_orig_ce_r_grey, img_r_ce, map_r, cv::noArray(), cv::INTER_LANCZOS4);
    std::cout << "done." << std::endl;

    cv::Mat red_cyan = Misc::merge_red_cyan(img_l_grey, img_r_grey);
    cv::Mat red_cyan_ce = Misc::merge_red_cyan(img_l_ce, img_r_ce);

    std::cout << "Writing images..." << std::flush;
#pragma omp parallel sections
    {
#pragma omp section
        cv::imwrite(cam_l->fn + "-" + suffix + "-stereo-direct.tif", img_l);
#pragma omp section
        cv::imwrite(cam_r->fn + "-" + suffix + "-stereo-direct.tif", img_r);
#pragma omp section
        cv::imwrite(cam_l->fn + "-" + suffix + "-stereo-direct-red-cyan.tif", red_cyan);
#pragma omp section
        cv::imwrite(cam_l->fn + "-" + suffix + "-stereo-direct-red-cyan.jpg", red_cyan, {cv::IMWRITE_JPEG_QUALITY, 95});
#pragma omp section
        cv::imwrite(cam_l->fn + "-" + suffix + "-stereo-direct-red-cyan-ce.tif", red_cyan_ce);
#pragma omp section
        cv::imwrite(cam_l->fn + "-" + suffix + "-stereo-direct-red-cyan-ce.jpg", red_cyan_ce, {cv::IMWRITE_JPEG_QUALITY, 95});
    }
    std::cout << "done." << std::endl;

}

void Calib::optimizeSFM(std::string const& plot_prefix) {
    triangulateAllRANSAC(plot_prefix);
    ceres::Problem problem;
    double const error_threshold = triangulation_error_stats.getQuantile(.8);
    for (auto pt : points) {
        pt->used = false;
        if (pt->max_error < error_threshold) {
            pt->addSFMBlocks(problem);
            pt->used = true;
        }
    }
    for (auto cam : cams) {
        cam->setIntrinsicsConstant(problem);
        problem.AddResidualBlock(
                    KeepNormCost::create(cv::norm(cam->extr.loc)),
                    nullptr,
                    cam->extr.loc.val
                    );
    }
    cams[0]->setExtrinsicsConstant(problem);


    ceres::Solver::Options ceres_opts;
    ceres::Solver::Summary summary;

    ceres_opts.minimizer_progress_to_stdout = true;
    ceres_opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres_opts.max_num_iterations = 2000;
    //ceres_opts.num_linear_solver_threads = std::thread::hardware_concurrency();

    std::cout << "###### Solving large SFM problem ######" << std::endl;
    ceres::Solve(ceres_opts, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

std::string Calib::printCams() const {
    std::stringstream out;
    for (size_t ii = 0; ii < cams.size(); ++ii) {
        out << "#" << ii << ": " << cams[ii]->print() << std::endl;
    }
    return out.str();
}

void Calib::plotResiduals() const {
    for (auto cam : cams) {
        cam->plotResiduals();
    }
}


