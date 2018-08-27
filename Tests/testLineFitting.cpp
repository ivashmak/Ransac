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


void testRansac(cv::InputArray points);
void testNapsac(cv::InputArray points);
void testProsac(cv::InputArray points);
void testEvsac(cv::InputArray points);

Estimator *estimator2d;
Quality quality;
Drawing drawing;

void init () {
    estimator2d = new Line2DEstimator;
}

bool dummyQualitySort(const cv::Point_<float>& a, const cv::Point_<float>& b) {
    if (a.x < b.x) return true;
    if (a.x > b.x) return false;
    if (a.y < b.y) return true;
    if (a.y > b.y) return false;
    return false;
}

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;
    generate(points);
    std::cout << "generated points\n";

    init();

   testRansac(points);

   testNapsac(points);

   std::vector<cv::Point_<float>> sortes_points (points);
   std::sort(sortes_points.begin(), sortes_points.end(), dummyQualitySort);
   testProsac(sortes_points);

   testEvsac(points);
}


void testRansac (cv::InputArray points) {
    Sampler *sampler = new UniformSampler;

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

    Sampler *evsac_sampler = new EvsacSampler(points, num_q, knn);

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
    Sampler *prosac_sampler = new ProsacSampler();

    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, model, *prosac_sampler, termination_criteria, quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);

    std::cout << "Prosac time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << "Prosac iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << "Prosac points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";
}
