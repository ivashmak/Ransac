// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef TESTS_TESTS_H
#define TESTS_TESTS_H

#include "../include/opencv2/usac/estimator.hpp"
#include "../include/opencv2/usac/quality.hpp"
#include "../include/opencv2/usac/ransac.hpp"
#include "../include/opencv2/usac/model.hpp"
#include "../include/opencv2/usac/nearest_neighbors.hpp"

class Tests {
public:
    static void testLineFitting ();
    static void testHomographyFitting ();
    static void testFundamentalFitting ();
    static void testEssentialFitting ();

    static void testFindMedian ();
    static void testFindNearestNeighbors (int knn=7);
    static void evaluateRandomGenerators();
    static void testInv ();

        static void test (const cv::Mat &points,
                          cv::usac::Model * model,
                          const std::string &img_name1,
                          const std::string &img_name2,
                          bool gt,
                          const std::vector<int>& gt_inliers);

    static std::string sampler2string (cv::usac::SAMPLER sampler) {
        if (sampler == cv::usac::SAMPLER::Prosac) return "prosac";
        if (sampler == cv::usac::SAMPLER::Uniform) return "uniform";
        if (sampler == cv::usac::SAMPLER::Napsac) return "napsac";
        return "";
    }

    static std::string estimator2string (cv::usac::ESTIMATOR estimator) {
        if (estimator == cv::usac::ESTIMATOR::Line2d) return "line2d";
        if (estimator == cv::usac::ESTIMATOR::Homography) return "homography";
        if (estimator == cv::usac::ESTIMATOR::Fundamental) return "fundamental";
        if (estimator == cv::usac::ESTIMATOR::Essential) return "essential";
        return "";
    }

    static std::string nearestNeighbors2string (cv::usac::NeighborsSearch nn) {
        if (nn == cv::usac::NeighborsSearch::Grid) return "grid";
        if (nn == cv::usac::NeighborsSearch::Nanoflann) return "nanoflann";
        return "";
    }

};

#endif //TESTS_TESTS_H