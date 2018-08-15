#include <cstdlib>
#include "Tests.h"

#include "../Ransac/Generator/generator.h"
#include "../Ransac/Detector/detector.h"
#include "../Ransac/Line2DEstimator.h"
#include "../Ransac/Ransac.h"
#include "../Ransac/UniformSampler.h"
#include "../Ransac/Drawing.h"

void Tests::testRansac() {

    srand (time(NULL));

    std::vector<cv::Point_<float>> points;
    generate(points);
    std::cout << "generated points\n";

    Model model(10, 2, 0.99, "ransac");
    UniformSampler sampler;
    TerminationCriteria termination_criteria (model);
    Quality quality;

    Estimator *estimator2d = new Line2DEstimator;

    Ransac naive_ransac (points, model, sampler, termination_criteria, quality);

    naive_ransac.run(points, estimator2d);

    Drawing drawing;
    drawing.draw(naive_ransac, points);

    std::cout << "Naive Ransac time: " << naive_ransac.quality->getComputationTime() << "mcs\n";
    std::cout << "Naive Ransac iterations: " << naive_ransac.quality->getIterations() << "\n";
    std::cout << "Naive Ransac points under threshold: " << naive_ransac.quality->getNumberOfPointsUnderThreshold() << "\n";

}