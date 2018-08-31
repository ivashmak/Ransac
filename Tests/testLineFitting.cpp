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
#include "../Usac/Helper/Logging.h"


void test (cv::InputArray points, Sampler * sampler, Model * model);

Estimator *estimator2d;
Quality *quality;
Drawing drawing;
Logging logResult;

int knn = 2;
cv::Mat indicies, dists1, dists2;
cv::flann::Index *flannIndex;

void init () {
    estimator2d = new Line2DEstimator;
    quality = new Quality;
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


    cv::Mat mat_points, p (points);
    mat_points = cv::Mat(points.size(), 2, CV_32F, p.data);
    cv::flann::LinearIndexParams flannIndexParams;
    flannIndex = new cv::flann::Index (cv::Mat(mat_points).reshape(1), flannIndexParams);
    std::vector<cv::Point_<float>> sorted_points (points);

    std::sort(sorted_points.begin(), sorted_points.end(), qualitySort);


//    Model *ransac_model = new Model (10, 2, 0.99, "ransac");
//    Sampler *uniform_sampler = new UniformSampler (ransac_model->sample_number, points.size());

//    Model *napsac_model = new Model (10, 2, 0.99, "napsac");
//    Sampler *napsac_sampler = new NapsacSampler(points, 10, napsac_model->sample_number, points.size());
//
//    Model *evsac_model = new Model (10, 2, 0.99, "evsac");
//    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), 7, evsac_model->sample_number, points.size());
//
    Model *prosac_model = new Model (10, 2, 0.99, "prosac");
    Sampler *prosac_sampler = new ProsacSampler(prosac_model->sample_number, points.size());

//    test (points, uniform_sampler, ransac_model);
//    test (points, napsac_sampler, napsac_model);
//    test (points, evsac_sampler, evsac_model);
    test (sorted_points, prosac_sampler, prosac_model);

}


void test (cv::InputArray points, Sampler * sampler, Model * model) {
    TerminationCriteria termination_criteria (model);

    Ransac ransac (points, *model, *sampler, termination_criteria, *quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);


    std::cout << model->model_name << " time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << model->model_name << " iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    logResult.compare(model, quality);
    logResult.saveResult(model, quality);
    std::cout << "-----------------------------------------------------------------------------------------\n";

}
