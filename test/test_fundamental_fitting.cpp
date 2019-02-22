// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/reader.hpp"

void opencv_test::testFundamentalFitting() {
    std::string img_name = "../test/dataset/fundamental/barrsmith";
    cv::Mat_<float> sorted_points, points;
    cv::usac::Reader::LoadPointsFromFile(points, (img_name+"_pts.txt").c_str());
    cv::usac::Reader::LoadPointsFromFile(sorted_points, (img_name+"_spts.txt").c_str());

    // get GT inliers with GT model
    std::vector<int> gt_inliers, gt_sorted_inliers;
    cv::Mat gt_model;
    cv::usac::Reader::getMatrix3x3(img_name+"_model.txt", gt_model);
    cv::usac::FundamentalEstimator est1(points);
    cv::usac::FundamentalEstimator est2(sorted_points);
    cv::usac::Quality::getInliers(&est1, gt_model, 1/*threshold*/, points.rows, gt_inliers);
    cv::usac::Quality::getInliers(&est2, gt_model, 1/*threshold*/, sorted_points.rows, gt_sorted_inliers);
    //

    float threshold = 2, confidence = 0.95;
    unsigned int knn = 8;

    cv::usac::Model * model;

    // -------------------------- uniform -------------------------------------
    model = new cv::usac::Model (threshold, confidence, knn,
            cv::usac::ESTIMATOR::Fundamental, cv::usac::SAMPLER::Uniform);
    // ------------------------------------------------------------------------

//     -------------------------- Prosac -------------------------------------
//    model = new cv::usac::Model (threshold, confidence, knn,
//          cv::usac::ESTIMATOR::Fundamental, cv::usac::SAMPLER::Prosac);
    // ------------------------------------------------------------------------

    model->lo = cv::usac::LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(cv::usac::NeighborsSearch::Nanoflann);
    model->ResetRandomGenerator(false);

    if (model->sampler == cv::usac::Prosac) {
        test (sorted_points, model, img_name+"A.png", img_name+"B.png", true, gt_sorted_inliers);
    } else {
        test (points, model, img_name+"A.png", img_name+"B.png", true, gt_inliers);
    }

    delete(model);
}

//TEST (usac_test, fundamental_matrix_test) {opencv_test::testFundamentalFitting();}
