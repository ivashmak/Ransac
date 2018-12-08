#include "Tests.h"

void Tests::test (cv::Mat points,
           Estimator * estimator,
           Sampler * sampler,
           Model * model,
           Quality * quality,
           TerminationCriteria * termination_criteria,
           const cv::Mat& neighbors,
           const std::string &img_name,
           int GT_num_inliers) {

    Drawing drawing;
    Logging logResult;

    Ransac ransac (model, sampler, termination_criteria, quality, estimator);
    ransac.setNeighbors(neighbors);
    ransac.run(points);

    RansacOutput * ransacOutput = ransac.getRansacOutput();

    std::cout << model->getName() << "\n";
    std::cout << "\ttime: ";
    ransacOutput->printTime();
    std::cout << "\titerations: " << ransacOutput->getTotalIters() <<
              " (where " << ransacOutput->getLOInnerIters () << " (inner iters) and " <<
              ransacOutput->getLOIterativeIters() << " (iterative iters)) and " << ransacOutput->getGCIters() << " (GC iters)\n";

    std::cout << "\tpoints under threshold: " << ransacOutput->getNumberOfInliers() << "\n";
    std::cout << "\tAverage error " << ransacOutput->getAverageError() << "\n";

    std::cout << "Best model = ...\n" << ransacOutput->getModel ()->returnDescriptor() << "\n";

    std::cout << "Ground Truth number of inliers for same model parametres is " << GT_num_inliers << "\n";

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";


    if (model->estimator == ESTIMATOR::Homography) {
        drawing.drawHomographies(img_name, points, ransacOutput->getInliers(), ransacOutput->getModel()->returnDescriptor());
    } else
    if (model->estimator == ESTIMATOR::Fundamental) {
        drawing.drawEpipolarLines(img_name, points.colRange(0,2), points.colRange(2,4), ransacOutput->getModel()->returnDescriptor());
    } else
    if (model->estimator == ESTIMATOR::Line2d) {
        drawing.draw(ransacOutput->getInliers(), ransacOutput->getModel(), points, img_name+".png");
    } else
    if (model->estimator == ESTIMATOR::Essential) {

    } else {
        std::cout << "UNKNOWN ESTIMATOR\n";
    }

}
