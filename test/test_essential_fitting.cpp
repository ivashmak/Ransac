// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/reader.hpp"

void Tests::testEssentialFitting() {
    std::string img_name = "../test/dataset/essential/fountain_dense";

    float threshold = 2, confidence = 0.95;
    unsigned int knn = 6;

    cv::Mat points, sorted_points;
    std::vector<int> gt_inliers, gt_sorted_inliers;
    cv::usac::Reader::LoadPointsFromFile(points, (img_name+"_pts.txt").c_str());
    cv::usac::Reader::LoadPointsFromFile(sorted_points, (img_name+"_spts.txt").c_str());
    cv::usac::Reader::readInliers (gt_inliers, img_name+"_inl.txt");

    // get sorted inliers using threshold 1!
    cv::usac::EssentialEstimator essentialEstimator(points);
    cv::usac::Model m (0, 0, 0, 0, cv::usac::ESTIMATOR::Essential, cv::usac::SAMPLER::Uniform);
    essentialEstimator.EstimateModelNonMinimalSample(&gt_inliers[0], gt_inliers.size(), m);
    cv::Mat gt_model = m.returnDescriptor().clone();
    cv::usac::EssentialEstimator essentialEstimator2(sorted_points);
    cv::usac::Quality::getInliers(&essentialEstimator2, gt_model, 1/*threshold*/, sorted_points.rows, gt_sorted_inliers);
    //

    cv::usac::Model * model;

    // -------------------------- uniform -------------------------------------
//    model = new Model (threshold, 5, confidence, knn, cv::usac::ESTIMATOR::Essential, cv::usac::SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
    model = new cv::usac::Model (threshold, 5, confidence, knn, cv::usac::ESTIMATOR::Essential, cv::usac::SAMPLER::Prosac);
    // ------------------------------------------------------------------------

    model->lo = cv::usac::LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(cv::usac::NeighborsSearch::Grid);
    model->ResetRandomGenerator(true);

    if (model->sampler == cv::usac::SAMPLER::Prosac)
        test (sorted_points, model, img_name+"A.png", img_name+"B.png", true, gt_sorted_inliers);
    else
        test (points, model, img_name+"A.png", img_name+"B.png", true, gt_inliers);
}