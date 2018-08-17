#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Detector/detector.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Prosac.h"
#include "../Usac/Napsac.h"
#include "../Usac/Sampler/NapsacSampler.h"

void testRansac(cv::InputArray points);
void testNapsac(cv::InputArray points);
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
    testNapsac(points);
//    testProsac(points);
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
    Sampler *napsac_sampler = new NapsacSampler;

    Model model(10, 2, 0.99, "napsac");
    TerminationCriteria termination_criteria (model);

    Napsac napsac (points, model, *sampler, termination_criteria, quality);
    napsac.run(points, estimator2d);
    drawing.draw(napsac.most_inliers, napsac.best_model, napsac.non_minimal_model, points);

    std::cout << "Napsac time: " << napsac.quality->getComputationTime() << "mcs\n";
    std::cout << "Napsac iterations: " << napsac.quality->getIterations() << "\n";
    std::cout << "Napsac points under threshold: " << napsac.quality->getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}

void testProsac (cv::InputArray points) {
    Model model(10, 2, 0.95, "prosac");
    TerminationCriteria termination_criteria (model);

    Prosac prosac (points, model, *sampler, termination_criteria, quality);
    prosac.run(points, estimator2d);
    drawing.draw(prosac.most_inliers, prosac.best_model, prosac.non_minimal_model, points);

    std::cout << "Prosac time: " << prosac.quality->getComputationTime() << "mcs\n";
    std::cout << "Prosac iterations: " << prosac.quality->getIterations() << "\n";
    std::cout << "Prosac points under threshold: " << prosac.quality->getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}
