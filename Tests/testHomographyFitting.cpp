#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"

#include "../Usac/Homographies/DLT.h"
#include "../Usac/Homographies/GetNormalizingTransformation.h"
#include "../Usac/Homographies/NormalizedDLT.h"
#include "../Usac/Model.h"
#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Ransac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Estimator/HomographyEstimator.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Helper/Logging.h"

void testDLT (cv::InputArray points1, cv::InputArray points2);
void testGetNormalizingTransformation (cv::InputArray points);
void testNormalizedDLT (cv::InputArray points1, cv::InputArray points2);
void test (cv::InputArray points, Model * const model, Sampler * const sampler);



void Tests::testHomographyFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2);

    cv::Mat H = cv::findHomography(points1, points2);
    std::cout << H << '\n';

    std::vector<cv::Mat> points;
    points.push_back(points1);
    points.push_back(points2);


    Model *homography_model = new Model (20, 4, 0.99, 0, "homography");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(homography_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);

    testDLT(points1, points2);

    test (points, homography_model, uniform_sampler);

//    testGetNormalizingTransformation(points1);
//    testGetNormalizingTransformation(points2);
//    testNormalizedDLT(points1, points2);
}

void test (cv::InputArray points, Model * const model, Sampler * const sampler) {
    Estimator * homograpy_estimator = new HomographyEstimator;;
    Quality *quality;
    Drawing drawing;
    Logging logResult;
    TerminationCriteria termination_criteria;

    Ransac ransac (*model, *sampler, termination_criteria, *quality);
    ransac.run(points, homograpy_estimator);
//    drawing.draw(ransac.most_inliers, &ransac.best_model, &ransac.non_minimal_model, points);

    std::cout << model->model_name << " time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << model->model_name << " iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";

    // save result and compare with last run
    logResult.compare(model, quality);
    logResult.saveResult(model, quality);
    std::cout << "-----------------------------------------------------------------------------------------\n";
}

void testDLT (cv::InputArray points1, cv::InputArray points2) {
    std::cout << "---------------- DLT ------------------------\n";

    cv::Mat H_DLT;
    DLT (points1, points2, H_DLT);
    std::cout << "H_DLT = \n" << H_DLT << "\n\n";

}

void testGetNormalizingTransformation (cv::InputArray points) {
    std::cout << "---------------- GetNormalizingTransformation ------------------------\n";

    float s, s1, s2;
    cv::Mat T, offset;
    GetNormalizingTransformation(points, T, offset, &s, &s1, &s2);
    std::cout << "offset =\n " << offset << "\n\n";
    std::cout << "T =\n " << T << "\n\n";
    std::cout << "s = " << s << "; s1 = " << s1 << "; s2 = " << s2 << '\n';

}

void testNormalizedDLT (cv::InputArray points1, cv::InputArray points2) {
    std::cout << "---------------- NormalizedDLT ------------------------\n";
    cv::Mat H_NDLT;
    NormalizedDLT(points1, points2, H_NDLT);
    std::cout << "H_NDLT = \n" << H_NDLT << "\n\n";
}