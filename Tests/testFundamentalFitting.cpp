#include "Tests.h"

#include "../Usac/SevenPointsAlgorithm.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename);
void runNTimesFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, int N);

void Tests::testFundamentalFitting() {
    std::string img_name = "graf";
    std::string points_filename = "../images/homography/"+img_name+"_pts.txt";
    std::vector<std::string> images_filename;
    images_filename.push_back("../images/homography/"+img_name+"A.png");
    images_filename.push_back("../images/homography/"+img_name+"B.png");

    cv::Mat points, points1, points2;
    read_points (points1, points2, points_filename);
    cv::hconcat(points1, points2, points);

    Model *fundamental_model = new Model (5, 7, 0.99, 0, "fundamental");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(fundamental_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);

    testFundamental (points, fundamental_model, uniform_sampler, images_filename, points_filename);

//    runNTimesFundamental(points, fundamental_model, uniform_sampler, 1000);
}

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename) {
    Estimator * fundamental_estimator = new FundamentalEstimator (points);
    Drawing drawing;
    Logging logResult;
    TerminationCriteria termination_criteria;

    Ransac ransac (*model, *sampler, termination_criteria);
    ransac.run(points, fundamental_estimator);
    std::cout << model->model_name << " time: ";
    ransac.getQuality()->printTime();
    std::cout << model->model_name << " iterations: " << ransac.getQuality()->getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality()->getNumberOfPointsUnderThreshold() << "\n";

    // save result and compare with last run
    logResult.compare(model, ransac.getQuality());
    logResult.saveResult(model, ransac.getQuality());
    std::cout << "-----------------------------------------------------------------------------------------\n";

    cv::Mat pts = points.getMat();
    drawing.drawEpipolarLines(images_filename, pts.colRange(0,2), pts.colRange(2,4), model->returnDescriptor());
}

void runNTimesFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, int N) {
    Estimator * fundamental_estimator = new FundamentalEstimator (points);
    TerminationCriteria termination_criteria;

    Ransac ransac (*model, *sampler, termination_criteria);
    float time = 0;
    for (int i = 0; i < N; i++) {
        ransac.run(points, fundamental_estimator);
        time += ransac.getQuality()->getComputationTime();
    }
    std::cout << "average time of "<< N <<" runs is " << (time/N) << "mcs using " << model->model_name
              << " points size is " << points.getMat().rows << "\n";

}