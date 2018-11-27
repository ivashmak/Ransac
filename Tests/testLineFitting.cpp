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
    unsigned int points_size = points.size();


    int knn = 7;
    cv::Mat_<float> pts = cv::Mat (points);

    cv::Mat_<float> neighbors, neighbors_dists;
    NearestNeighbors nn;
    nn.getNearestNeighbors_nanoflann(pts, knn, neighbors, true, neighbors_dists);
    std::vector<int> sorted_idx (points_size);
    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);

    /*
     * Prosac quality sort.
     * Sorting by sum of distances of the (3) nearest neighbors.
     */
    float sum1, sum2;
    int idxa, idxb;
    float * neighbors_dists_ptr = (float *) neighbors_dists.data;
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&] (int a, int b) {
        sum1 = 0, sum2 = 0;
        idxa = knn*a, idxb = knn*b;
        for (int i = 0; i < 3; i++) {
            sum1 += neighbors_dists_ptr[idxa + i];
            sum2 += neighbors_dists_ptr[idxb + i];
        }
        return sum1 < sum2;
    });

    std::vector<cv::Point_<float>> sorted_points;
    for (int i = 0; i < points_size; i++) {
        sorted_points.push_back(points[sorted_idx[i]]);
    }


    // Main Ransac components
    Estimator *estimator = new Line2DEstimator (points);
    TerminationCriteria * termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    Model * model;
    Sampler * sampler;
    //



    // ---------------- uniform -------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Uniform);
//
//    model->setSprtLO(false);
//    model->setGraphCutLO(false);
//    model->setStandardRansacLO(false);
//
//    initUniform(sampler, model->sample_number, points_size);
//    test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, "", gt_inliers);
    //------------------------------------------





    // --------------  prosac ---------------------
//    model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Prosac);
//    initProsac(sampler, model->sample_number, points.size());
//    ProsacSampler *prosac_sampler_ = (ProsacSampler *) sampler;
//
//    ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
//    prosac_termination_criteria_->initProsacTerminationCriteria (prosac_sampler_->getGrowthFunction(),
//                                                model, points_size);
//
//    TerminationCriteria * prosac_termination_criteria = prosac_termination_criteria_;
//    cv::Mat sorted_pts (sorted_points);
//    estimator = new Line2DEstimator (sorted_points);
//
//    test (sorted_pts, estimator, sampler, model, quality, prosac_termination_criteria, neighbors, "", gt_inliers);
//
//    // switch to unsorted points back (not necessary, just for testing)
//    estimator = new Line2DEstimator (points);
    // ------------------------------------------------





    // ---------------- napsac -------------------------------
//    knn = 7;
//    model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
//    initNapsac(sampler, neighbors, model->k_nearest_neighbors, model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, "", gt_inliers);
    // ---------------------------------------------------------------------




    // ----------------- evsac ------------------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//    sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, "", gt_inliers);
    // ------------------------------------------------------------





    // ------------------ gradually increasing ransac ----------------------
//    model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//    sampler = new GradualNapsacSampler(points, gradual_napsac_model->sample_number);
//
//     test (pts, estimator, sampler, model, quality, termination_criteria, neighbors, "", gt_inliers);
    // --------------------------------------------------------



//     getStatisticalResults(pts, estimator, model, sampler, termination_criteria, quality, neighbors,
//                           1000, true, false, gt_inliers);
}

