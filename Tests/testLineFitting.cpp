#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"

#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Detector/ReadPoints.h"

#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Sampler/ProsacSampler.h"


void testLine (cv::InputArray points, Sampler * sampler, Model * model);

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;

    // change false to true to reset time for random points generator
    generate(points, false);
    std::cout << "generated points\n";


    // sort points for Prosac
    cv::Mat indicies, dists1, dists2, p (points);
    int knn = 2;
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index * flannIndex = new cv::flann::Index (p.reshape(1), flannIndexParams);
    std::vector<cv::Point_<float>> sorted_points (points);

    /*
     * Prosac quality sort.
     * Sorting by distance of first nearest neighbor.
     */
    std::sort(sorted_points.begin(), sorted_points.end(), [&] (const cv::Point_<float>& a, const cv::Point_<float>& b) {
        flannIndex->knnSearch(cv::Mat_<float>(a), indicies, dists1, knn);
        flannIndex->knnSearch(cv::Mat_<float>(b), indicies, dists2, knn);
        return dists1.at<float>(1) < dists2.at<float>(1);
    });
    //---

    Model *ransac_model = new Model (10, 2, 0.99, 0, "ransac");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(ransac_model->sample_number);
    uniform_sampler->setPointsSize(points.size());

    Model *napsac_model = new Model (10, 2, 0.99, 20, "napsac");
    Sampler *napsac_sampler = new NapsacSampler(points, napsac_model->k_nearest_neighbors, napsac_model->sample_number);

    Model *evsac_model = new Model (10, 2, 0.99, 7, "evsac");
    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);

    Model *prosac_model = new Model (10, 2, 0.99, 0, "prosac");
    Sampler *prosac_sampler = new ProsacSampler(prosac_model->sample_number, points.size());

    testLine (points, uniform_sampler, ransac_model);
//    testLine (points, napsac_sampler, napsac_model);
//    testLine (points, evsac_sampler, evsac_model);
//    testLine (sorted_points, prosac_sampler, prosac_model);


    Estimator *line2destimator = new Line2DEstimator;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;
    runNTimes(points, line2destimator, ransac_model, uniform_sampler, termination_criteria, quality, 1000);
}


void testLine (cv::InputArray points, Sampler * const sampler, Model * const model) {
    Estimator *estimator2d = new Line2DEstimator;
    Drawing drawing;
    Logging logResult;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    Ransac ransac (*model, *sampler, *termination_criteria, *quality);
    ransac.run(points, estimator2d);

    std::cout << model->model_name << " time: ";
    ransac.getQuality()->printTime();
    std::cout << model->model_name << " iterations: " << ransac.getQuality()->getIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransac.getQuality()->getNumberOfPointsUnderThreshold() << "\n";

    // save result and compare with last run
    logResult.compare(model, ransac.getQuality());
    logResult.saveResult(model, ransac.getQuality());
    std::cout << "-----------------------------------------------------------------------------------------\n";

    //    drawing.draw(ransac.most_inliers, ransac.getBestModel(), ransac.getNonMinimalModel(), points);
    drawing.draw(ransac.most_inliers, &ransac.best_model, &ransac.non_minimal_model, points);

}


