#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Detector/detector.h"
#include "../Usac/Line2DEstimator.h"
#include "../Usac/Ransac.h"
#include "../Usac/UniformSampler.h"
#include "../Usac/Drawing.h"

void Tests::testLineFitting() {

    srand (time(NULL));

    std::vector<cv::Point_<float>> points;
    generate(points);
    std::cout << "generated points\n";

    Model model(10, 2, 0.99, "ransac");
    UniformSampler sampler;
    TerminationCriteria termination_criteria (model);
    Quality quality;

    Estimator *estimator2d = new Line2DEstimator;

    Ransac ransac (points, model, sampler, termination_criteria, quality);


    ransac.run(points, estimator2d);

    Drawing drawing;
    drawing.draw(ransac, points);

    std::cout << "Naive Usac time: " << ransac.quality->getComputationTime() << "mcs\n";
    std::cout << "Naive Usac iterations: " << ransac.quality->getIterations() << "\n";
    std::cout << "Naive Usac points under threshold: " << ransac.quality->getNumberOfPointsUnderThreshold() << "\n";

}