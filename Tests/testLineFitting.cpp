#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"

#include "../Usac/Helper/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Detector/ReadPoints.h"

#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/UniformSampler.h"


void test (cv::InputArray points, Sampler * sampler, Model * model);
void runNTimes (cv::InputArray points, Sampler * sampler, Model * model, int N);

Estimator *estimator2d;
Quality *quality;
Drawing drawing;
Logging logResult;

int knn = 2;
cv::Mat indicies, dists1, dists2;
cv::flann::Index *flannIndex;

void init () {
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

    estimator2d = new Line2DEstimator (points);

    std::cout << "generated points\n";

    init();

    cv::Mat p (points);
    cv::flann::LinearIndexParams flannIndexParams;
    flannIndex = new cv::flann::Index (p.reshape(1), flannIndexParams);
    std::vector<cv::Point_<float>> sorted_points (points);

    std::sort(sorted_points.begin(), sorted_points.end(), qualitySort);


    Model *ransac_model = new Model (10, 2, 0.99, 0, "ransac");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(ransac_model->sample_number);
    uniform_sampler->setRange(0, points.size()-1);

    Model *napsac_model = new Model (10, 2, 0.99, 6, "napsac");
    Sampler *napsac_sampler = new NapsacSampler(points, napsac_model->k_nearest_neighbors, napsac_model->sample_number);

    Model *evsac_model = new Model (10, 2, 0.99, 7, "evsac");
    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);

    Model *prosac_model = new Model (10, 2, 0.99, 0, "prosac");
    Sampler *prosac_sampler = new ProsacSampler(prosac_model->sample_number, points.size());

    test (points, uniform_sampler, ransac_model);
//    test (points, napsac_sampler, napsac_model);
//    test (points, evsac_sampler, evsac_model);
//    test (sorted_points, prosac_sampler, prosac_model);

//    runNTimes(points, uniform_sampler, ransac_model, 1000);
}


void test (cv::InputArray points, Sampler * sampler, Model * model) {
    TerminationCriteria termination_criteria (model);

    Ransac ransac (*model, *sampler, termination_criteria, *quality);
    ransac.run(points, estimator2d);
    drawing.draw(ransac.most_inliers, ransac.best_model, ransac.non_minimal_model, points);


    std::cout << model->model_name << " time: " << ransac.getQuality().getComputationTime() << "mcs\n";
    std::cout << model->model_name << " iterations: " << ransac.getQuality().getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality().getNumberOfPointsUnderThreshold() << "\n";
    logResult.compare(model, quality);
    logResult.saveResult(model, quality);
    std::cout << "-----------------------------------------------------------------------------------------\n";

}

void runNTimes (cv::InputArray points, Sampler * sampler, Model * model, int N) {
    TerminationCriteria termination_criteria (model);
    Ransac ransac (*model, *sampler, termination_criteria, *quality);
    double time = 0;
    for (int i = 0; i < N; i++) {
        ransac.run(points, estimator2d);
        time += ransac.getQuality().getComputationTime();
    }
    std::cout << "average time of "<< N <<" runs is " << (time/N) << "mcs using " << model->model_name
              << " points size is " << points.size().width << "\n";
}