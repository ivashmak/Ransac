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

namespace opencv_test {
    void testLineFitting ();
    void testHomographyFitting ();
    void testFundamentalFitting ();
    void testEssentialFitting ();

    void testFindMedian ();
    void testFindNearestNeighbors (int knn=7);
    void evaluateRandomGenerators();
    void testInv ();

    void test (const cv::Mat &points,
                          cv::usac::Model * model,
                          const std::string &img_name1,
                          const std::string &img_name2,
                          bool gt,
                          const std::vector<int>& gt_inliers);
}

#endif //TESTS_TESTS_H