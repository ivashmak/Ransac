// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/reader.hpp"

void opencv_test::testHomographyFitting() {
    float threshold = 2;
    unsigned int knn = 7;
    float confidence = 0.95;

    std::string img_name = "../test/dataset/homography/graf";
    cv::Mat points, sorted_points;
    cv::usac::Reader::LoadPointsFromFile(points, (img_name+"_pts.txt").c_str());
    cv::usac::Reader::LoadPointsFromFile(sorted_points, (img_name+"_spts.txt").c_str());

    // get GT inliers with GT model
    std::vector<int> gt_inliers, gt_sorted_inliers;
    cv::Mat gt_model;
    cv::usac::Reader::getMatrix3x3(img_name+"_model.txt", gt_model);
    cv::usac::HomographyEstimator est1(points);
    cv::usac::HomographyEstimator est2(sorted_points);
    cv::usac::Quality::getInliers(&est1, gt_model.inv(), 1/*threshold*/, points.rows, gt_inliers);
    cv::usac::Quality::getInliers(&est2, gt_model.inv(), 1/*threshold*/, sorted_points.rows, gt_sorted_inliers);
    //

    cv::usac::Model * model;

//     ---------------------- uniform ----------------------------------
//   model = new cv::usac::Model (threshold, confidence, knn,
//           cv::usac::ESTIMATOR::Homography, cv::usac::SAMPLER::Uniform);
//     --------------------------------------------------------------

//     ---------------------- napsac ----------------------------------
//    model = new cv::usac::Model (threshold, confidence, knn,
// cv::usac::ESTIMATOR::Homography, cv::usac::SAMPLER::Napsac);
//     --------------------------------------------------------------

// ------------------ prosac ---------------------
     model = new cv::usac::Model (threshold, confidence, knn,
 cv::usac::ESTIMATOR::Homography, cv::usac::SAMPLER::Prosac);
//     -------------------------------------------------


     model->lo = cv::usac::LocOpt ::GC;
     model->setSprt(0);
     model->setCellSize(50);
     model->setNeighborsType(cv::usac::NeighborsSearch::Grid);
     model->ResetRandomGenerator(false);

     if (model->sampler == cv::usac::SAMPLER::Prosac) {
         test (sorted_points, model, img_name+"A.png", img_name+"B.png", true, gt_sorted_inliers);
     } else {
         test (points, model, img_name+"A.png", img_name+"B.png", true, gt_inliers);
     }

     delete(model);
}

//TEST (usac_test, homography_matrix_test) {opencv_test::testHomographyFitting();}
