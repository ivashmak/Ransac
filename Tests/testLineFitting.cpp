#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Detector/detector.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/RandomGenerator/ArrayRandomGenerator.h"
#include "../Usac/RandomGenerator/XorRandomGenerator.h"
#include "../Usac/RandomGenerator/PrimeNumberRandomGenerator.h"
#include "../Usac/Utils.h"


void testRansac(cv::InputArray points);
void testNapsac(cv::InputArray points);
void testProsac(cv::InputArray points);
void testEvsac(cv::InputArray points);

Estimator *estimator2d;
Quality quality;
Drawing drawing;

int knn = 2;
cv::Mat indicies, dists1, dists2;
cv::flann::Index *flannIndex;

void init () {
    estimator2d = new Line2DEstimator;
}

// sort by nearest neighbors
bool qualitySort (const cv::Point_<float>& a, const cv::Point_<float>& b) {
    flannIndex->knnSearch(cv::Mat_<float>(a), indicies, dists1, knn);
    flannIndex->knnSearch(cv::Mat_<float>(b), indicies, dists2, knn);
    return dists1.at<float>(1) < dists2.at<float>(1);
}

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;
    generate(points);

    std::cout << "generated points\n";

    init();

//    Utils utils;
//    utils.getNullSpace();

//   testRansac(points);

//   testNapsac(points);

//    cv::Mat mat_points, p (points);
//    mat_points = cv::Mat(points.size(), 2, CV_32F, p.data);
//    cv::flann::LinearIndexParams flannIndexParams;
//    flannIndex = new cv::flann::Index (cv::Mat(mat_points).reshape(1), flannIndexParams);
//    std::vector<cv::Point_<float>> sorted_points (points);
//
//    std::sort(sorted_points.begin(), sorted_points.end(), qualitySort);
//    testProsac(sorted_points);

//   testEvsac(points);
}


void testRansac (cv::InputArray points) {
    Model model(10, 2, 0.99, "ransac");

    Sampler *sampler = new UniformSampler (model.sample_number, points.size().width);

    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Ransac time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Ransac iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << "Ransac points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}

void testNapsac (cv::InputArray points) {
    int knn = 10;
    Model model(10, 2, 0.99, "napsac");
    Sampler *napsac_sampler = new NapsacSampler(points, knn, model.sample_number, points.size().width);

    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *napsac_sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Napsac time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Napsac iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << "Napsac points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}

void testEvsac (cv::InputArray points) {
    Model model(10, 2, 0.99, "evsac");
    int knn = 7;
    int num_q = points.size().width;

    Sampler *evsac_sampler = new EvsacSampler(points, num_q, knn, model.sample_number, points.size().width);

    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *evsac_sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Evsac time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Evsac iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << "Evsac points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}


void testProsac (cv::InputArray points) {
    Model model(10, 2, 0.99, "prosac");
    Sampler *prosac_sampler = new ProsacSampler(model.sample_number, points.size().width);

    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *prosac_sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Prosac time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Prosac iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << "Prosac points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}
