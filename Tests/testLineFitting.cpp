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
#include "../Usac/Sampler/GradualNapsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/Utils/NearestNeighbors.h"

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;
    int gt_inliers;
    // change false to true to reset time for random points generator
    // get number of ground truth inliers too.
    generate(points, false, true, &gt_inliers);
    std::cout << "generated points\n";

    // sort points for Prosac
    cv::Mat indicies, dists1, dists2, pts (points);
    int knn = 2;
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index * flannIndex = new cv::flann::Index (pts.reshape(1), flannIndexParams);
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

     Model *model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Uniform);
     Sampler *uniform_sampler = new UniformSampler;
     uniform_sampler->setSampleSize(model->sample_number);
     uniform_sampler->setPointsSize(points.size());
     uniform_sampler->initRandomGenerator();

//    Model *gradual_napsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//    Sampler *gradual_napsac_sampler = new GradualNapsacSampler(points, gradual_napsac_model->sample_number);
//
//    knn = 7;
//    Model *napsac_model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
//    cv::Mat neighbors;
//    NearestNeighbors nn;
//    nn.getNearestNeighbors_flann(pts, knn+1, neighbors);
//    Sampler *napsac_sampler = new NapsacSampler(neighbors, napsac_model->k_nearest_neighbors, napsac_model->sample_number);

//    Model *evsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);
//
//    Model *prosac_model = new Model (10, 2, 0.99, 0, ESTIMATOR::Line2d, SAMPLER::Prosac);
//    Sampler *prosac_sampler = new ProsacSampler(prosac_model->sample_number, points.size());

    Estimator *line2destimator = new Line2DEstimator (points);
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    test (pts, line2destimator, uniform_sampler, model, quality, termination_criteria, "", 0);
    // test (pts, line2destimator, napsac_sampler, model, quality, termination_criteria, "", 0);
    // test (pts, line2destimator, evsac_sampler, model, quality, termination_criteria, "", 0);
    // test (pts, line2destimator, prosac_sampler, model, quality, termination_criteria, "", 0);
    // test (pts, line2destimator, gradual_napsac_sampler, model, quality, termination_criteria, "", 0);
    
    // getAverageResults(pts, line2destimator, model, uniform_sampler, termination_criteria, quality, 1000, true, false, gt_inliers);
}

