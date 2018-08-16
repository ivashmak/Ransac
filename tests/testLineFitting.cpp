#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Detector/detector.h"
#include "../Usac/Line2DEstimator.h"
#include "../Usac/Ransac.h"
#include "../Usac/UniformSampler.h"
#include "../Usac/Drawing.h"
#include "../Usac/Prosac.h"

void testRansac(cv::InputArray points);
void testProsac(cv::InputArray points);

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

    testRansac(points);
//    testProsac(points);
}


void testRansac (cv::InputArray points) {
    Model model(10, 2, 0.99, "ransac");
    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Naive Usac time: " << ransac.quality->getComputationTime() << "mcs\n";
    std::cout << "Naive Usac iterations: " << ransac.quality->getIterations() << "\n";
    std::cout << "Naive Usac points under threshold: " << ransac.quality->getNumberOfPointsUnderThreshold() << "\n";
}

void testProsac (cv::InputArray points) {
    Model model(10, 2, 0.95, "prosac");
    TerminationCriteria termination_criteria (model);

    Prosac prosac (points, model, *sampler, termination_criteria, quality);
    prosac.run(points, estimator2d);
    drawing.draw(prosac.most_inliers, prosac.best_model, prosac.non_minimal_model, points);

    std::cout << "Naive Usac time: " << prosac.quality->getComputationTime() << "mcs\n";
    std::cout << "Naive Usac iterations: " << prosac.quality->getIterations() << "\n";
    std::cout << "Naive Usac points under threshold: " << prosac.quality->getNumberOfPointsUnderThreshold() << "\n";
}
