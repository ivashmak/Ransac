#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Detector/detector.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Prosac.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Evsac.h"
#include "../Detector/ReadPoints.h"

void testRansac(cv::InputArray points);
void testNapsac(cv::InputArray points);
void testProsac(cv::InputArray points);
void testEvsac(cv::InputArray points);

Sampler *sampler;
Estimator *estimator2d;
Quality quality;
Drawing drawing;

void init () {
    sampler = new UniformSampler;
    estimator2d = new Line2DEstimator;
}

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;
    generate(points);
    std::cout << "generated points\n";

    init();

//    testRansac(points);
//    testNapsac(points);
//    testProsac(points);
    testEvsac(points);
}


void testRansac (cv::InputArray points) {
    Model model(10, 2, 0.99, "ransac");
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
    Sampler *napsac_sampler = new NapsacSampler(points, knn);

    Model model(10, 2, 0.99, "napsac");
    TerminationCriteria termination_criteria (model);

    Ransac napsac (points, model, *napsac_sampler, termination_criteria, quality);
    napsac.run(points, estimator2d);
    drawing.draw(napsac.most_inliers, napsac.best_model, napsac.non_minimal_model, points);

    std::cout << "Napsac time: " << napsac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Napsac iterations: " << napsac.getQuality().getIterations() << "\n";
    std::cout << "Napsac points under threshold: " << napsac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}

void testEvsac (cv::InputArray points) {
    Model model(10, 2, 0.95, "prosac");
    TerminationCriteria termination_criteria (model);

    cv::Mat points1, points2;
    read_points (points1, points2);

    Evsac evsac (points, model, *sampler, termination_criteria, quality);
    evsac.run(points1, points2, estimator2d);
}


void testProsac (cv::InputArray points) {
    Model model(10, 2, 0.95, "prosac");
    TerminationCriteria termination_criteria (model);

    Prosac prosac (points, model, *sampler, termination_criteria, quality);
    prosac.run(points, estimator2d);
    drawing.draw(prosac.most_inliers, prosac.best_model, prosac.non_minimal_model, points);

    std::cout << "Prosac time: " << prosac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Prosac iterations: " << prosac.getQuality().getIterations() << "\n";
    std::cout << "Prosac points under threshold: " << prosac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}
