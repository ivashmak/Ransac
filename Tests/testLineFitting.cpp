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
#include "../Usac/Sampler/ProsacSimpleSampler.h"
#include "../Usac/Utils/NearestNeighbors.h"
#include "../Usac/TerminationCriteria/ProsacTerminationCriteria.h"
#include "../Usac/Sampler/ProsacSampler.h"

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

    // Main Ransac components
    Estimator *estimator = new Line2DEstimator (points);
    TerminationCriteria * termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    Model * model;
    Sampler * sampler;
    //

    unsigned int points_size = points.size();

    // ---------------- uniform -------------------
    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Uniform);

    model->setSprtLO(false);
    model->setGraphCutLO(false);
    model->setStandardRansacLO(false);

    initUniform(sampler, model->sample_number, points_size);
     test (pts, estimator, sampler, model, quality, termination_criteria, "", gt_inliers);
    //------------------------------------------



    // --------------  prosac ---------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Prosac);
//    initProsac(sampler, model->sample_number, points.size());std::cout << "inited\n";
//    ProsacSampler *prosac_sampler_ = (ProsacSampler *) sampler;
//
//    ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
//    prosac_termination_criteria_->initProsacTerminationCriteria (prosac_sampler_->getGrowthFunction(),
//                                                model, points.size());
//
//    TerminationCriteria * prosac_termination_criteria = prosac_termination_criteria_;
//    cv::Mat sorted_pts (sorted_points);
//    estimator = new Line2DEstimator (sorted_points);
//
//    test (sorted_pts, estimator, sampler, model, quality, prosac_termination_criteria, "", gt_inliers);
//
//    // switch to unsorted points back (not necessary, just for testing)
//    estimator = new Line2DEstimator (points);
    // ------------------------------------------------




    // ---------------- napsac -------------------------------
//    knn = 7;
//    model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
//    cv::Mat neighbors;
//    NearestNeighbors nn;
//    nn.getNearestNeighbors_flann(pts, knn+1, neighbors);
//    initNapsac(sampler, neighbors, model->k_nearest_neighbors, model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, "", gt_inliers);
    // ---------------------------------------------------------------------




    // ----------------- evsac ------------------------------
//    Model *evsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);

    // test (pts, estimator, evsac_sampler, model, quality, termination_criteria, "", gt_inliers);
    // ------------------------------------------------------------




    // ------------------ gradually increasing ransac ----------------------
//    Model *gradual_napsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//    Sampler *gradual_napsac_sampler = new GradualNapsacSampler(points, gradual_napsac_model->sample_number);

    // test (pts, estimator, gradual_napsac_sampler, model, quality, termination_criteria, "", gt_inliers);
    // --------------------------------------------------------



    // getStatisticalResults(pts, estimator, model, uniform_sampler, termination_criteria, quality, 1000, true, false, gt_inliers);
}

