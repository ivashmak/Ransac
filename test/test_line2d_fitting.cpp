// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/utils.hpp"

void opencv_test::testLineFitting() {
    std::string img_name = "../test/dataset/line2d/1";

    cv::Mat points, sorted_points;

    std::ifstream read_data_file;
    read_data_file.open (img_name+".txt");
    if (!read_data_file.is_open()) {
        std::cout << "Wrong directory for file of line2d dataset!\n";
        exit (0);
    }
    int width, height, noise, N;

    read_data_file >> width;
    read_data_file >> height;
    read_data_file >> noise;
    float a, b, c;
    read_data_file >> a;
    read_data_file >> b;
    read_data_file >> c;

    cv::Mat gt_model = (cv::Mat_<float> (1,3) << a, b, c);

    read_data_file >> N;
    points = cv::Mat_<float>(N, 2);
    float *pts_ptr = (float *) points.data;
    float x, y;
    for (int p = 0; p < N; p++) {
        read_data_file >> x >> y;
        pts_ptr[2*p] = x;
        pts_ptr[2*p+1] = y;
    }

    cv::usac::densitySort(points, 10 /*knn*/, sorted_points);

    // get GT inliers with GT model
    std::vector<int> gt_inliers, gt_sorted_inliers;
    cv::usac::Line2DEstimator est1(points);
    cv::usac::Line2DEstimator est2(sorted_points);
    cv::usac::Quality::getInliers(&est1, gt_model, 3/*threshold*/, points.rows, gt_inliers);
    cv::usac::Quality::getInliers(&est2, gt_model, 3/*threshold*/, sorted_points.rows, gt_sorted_inliers);
    //

    unsigned int knn = 7;
    cv::usac::Model * model;
    float threshold = 8, confidence = 0.99;

    // ---------------- uniform -------------------
//     model = new cv::usac::Model (threshold, confidence, knn,
//         cv::usac::ESTIMATOR::Line2d, cv::usac::SAMPLER::Uniform);
//------------------------------------------

    // --------------  prosac ---------------------
    model = new cv::usac::Model (threshold, confidence, knn,
            cv::usac::ESTIMATOR::Line2d, cv::usac::SAMPLER::Prosac);
     // ------------------------------------------------

    // ---------------- napsac -------------------------------
//     model = new cv::usac::Model (threshold, confidence, knn,
//             cv::usac::ESTIMATOR::Line2d, cv::usac::SAMPLER::Napsac);
    // ---------------------------------------------------------------------

     model->lo = cv::usac::LocOpt ::NullLO;
     model->setSprt(0);
     model->setNeighborsType(cv::usac::NeighborsSearch::Nanoflann);

     if (model->sampler == cv::usac::SAMPLER::Prosac) {
         test (sorted_points, model, img_name+".png", std::string(), true, gt_sorted_inliers);
     } else {
         test (points, model, img_name+".png", std::string(), true, gt_inliers);
     }

     delete (model);
}

//TEST (usac_test, line2d_test) {opencv_test::testLineFitting();}
