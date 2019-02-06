#include "tests.h"

#include <cstdio>
#include <iostream>
#include <opencv2/core/types.hpp>

#include "../Detector/Reader.h"


void Tests::testEssentialFitting() {
    DATASET dataset = DATASET::Strecha;

    std::string img_name = "Dresden";
    cv::Mat points;
    Reader::getPointsNby6 ("../dataset/Lebeda/strechamvs/"+img_name+"_vpts_pts.txt", points);

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

    model->lo = LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Grid);

    test (points, model, img_name, dataset, false, gt_inliers);
}
