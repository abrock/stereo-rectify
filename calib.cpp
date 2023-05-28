#include "calib.h"

#include <map>

#include <opencv2/xfeatures2d.hpp>

#include "point3d.h"

#include <runningstats/runningstats.h>
namespace rs = runningstats;

std::shared_ptr<Point3D> Calib::findOrMake(const int idx, const cv::KeyPoint &kp) {
    for (auto& pt : points) {
        for (auto& obs : pt->observations) {
            if (idx == obs->idx && Observation::distKP(kp, obs->pt) < 1e-6) {
                return pt;
            }
        }
    }
    auto result = std::make_shared<Point3D>();
    auto obs = std::make_shared<Observation>();
    obs->idx = idx;
    obs->pt3d = result;
    obs->pt = kp;
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
                auto pt = findOrMake(first.queryIdx, kp1);
                auto obs = std::make_shared<Observation>();
                obs->idx = first.trainIdx;
                obs->pt3d = pt;
                obs->pt = kp2;
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

std::pair<double, double> Calib::computeRotation(
        std::shared_ptr<Cam> cam1,
        std::shared_ptr<Cam> cam2) const {

    return {0,0};
}

