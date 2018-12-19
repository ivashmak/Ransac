#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"


void Tests::testEssentialFitting() {
    std::string img_name = "Brussels_vpts";
    cv::Mat points1, points2, points;
    read_points (points1, points2, "../dataset/Lebeda/strechamvs/"+img_name+"_pts.txt");
    cv::hconcat(points1, points2, points);

    std::cout << "points size = " << points.rows << "\n";

    float threshold = 5;
    float confidence = 0.95;
    int knn = 5;

    std::vector<int> gt_inliers;

    Model * model;

    // -------------------------- uniform -------------------------------------
    model = new Model (threshold, 5, confidence, knn, ESTIMATOR::Essential, SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
//    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Prosac);
//    sorted_points.copyTo(points);
    // ------------------------------------------------------------------------

    model->setStandardRansacLO(0);
    model->setGraphCutLO(0);
    model->setSprtLO(0);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Grid);

    test (points, model, img_name, false, gt_inliers);
}
