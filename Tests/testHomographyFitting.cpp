#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"

#include "../Usac/DLT/DLT.h"
#include "../Usac/DLT/GetNormalizingTransformation.h"
#include "../Usac/DLT/NormalizedDLT.h"
#include "../Usac/Model.h"
#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Ransac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Estimator/HomographyEstimator.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Generator/generator.h"

void test (cv::InputArray points, Model * const model, Sampler * const sampler, std::vector<std::string> images_filename, std::string points_filename);
void runNtimes (cv::InputArray points, Model * const model, Sampler * const sampler, int N);

void Tests::testHomographyFitting() {
    std::string points_filename = "../images/homography/graf_pts.txt";
    std::vector<std::string> images_filename;
    images_filename.push_back("../images/homography/grafA.png");
    images_filename.push_back("../images/homography/grafB.png");

    cv::Mat points1, points2;
    read_points (points1, points2, points_filename);

    std::vector<cv::Mat> points;
    points.push_back(points1);
    points.push_back(points2);

    Model *homography_model = new Model (10, 4, 0.99, 0, "homography");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(homography_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);

    test (points, homography_model, uniform_sampler, images_filename, points_filename);

//    runNtimes(points, homography_model, uniform_sampler, 1000);

}

void test (cv::InputArray points, Model * const model, Sampler * const sampler, std::vector<std::string> images_filename, std::string points_filename) {

    Estimator * homograpy_estimator = new HomographyEstimator;;
    Drawing drawing;
    Logging logResult;
    TerminationCriteria termination_criteria;

    Ransac ransac (*model, *sampler, termination_criteria);
    ransac.run(points, homograpy_estimator);

    std::cout << model->model_name << " time: " << ransac.getQuality()->getComputationTime() << "mcs\n";
    std::cout << model->model_name << " iterations: " << ransac.getQuality()->getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality()->getNumberOfPointsUnderThreshold() << "\n";

    // save result and compare with last run
    logResult.compare(model, ransac.getQuality());
    logResult.saveResult(model, ransac.getQuality());
    std::cout << "-----------------------------------------------------------------------------------------\n";

    drawing.drawHomographies(images_filename, points_filename, ransac.most_inliers, ransac.best_model.returnDescriptor());
}

void runNTimes (cv::InputArray points, Model * const model, Sampler * const sampler, int N) {
    Estimator * homograpy_estimator = new HomographyEstimator;;
    Drawing drawing;
    Logging logResult;
    TerminationCriteria termination_criteria;

    Ransac ransac (*model, *sampler, termination_criteria);
    float time = 0;
    for (int i = 0; i < N; i++) {
        ransac.run(points, homograpy_estimator);
        time += ransac.getQuality()->getComputationTime();
    }
    std::cout << "average time of "<< N <<" runs is " << (time/N) << "mcs using " << model->model_name
              << " points size is " << points.size().width << "\n";

}