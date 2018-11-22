#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"

#include "../Usac/Model.h"
#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Ransac/Ransac.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Estimator/HomographyEstimator.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Generator/generator.h"
#include "../dataset/Dataset.h"
#include "../Usac/Utils/NearestNeighbors.h"
#include "../Usac/TerminationCriteria/ProsacTerminationCriteria.h"

void storeResults ();
int getGTNumInliers (const std::string &filename, float threshold);

void Tests::testHomographyFitting() {
    std::string img_name = "graf";
    cv::Mat points, points1, points2;
    read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");

   // points1 = (cv::Mat_<float> (10, 2) << 0.616188105841203, 0.511878887258433,
   //           -1.235108941720951,   0.298181899685778,
   //           -0.602445781444621, 0.395316341521395,
   //            0.638537560775656  , 0.476360331650075,
   //            -0.646735338229623 ,  0.404851073762976,
   //            -0.359599737840777,   0.408160300564896,
   //             0.496360069223658  , 0.485702841945883,
   //             0.445172420567822  , 0.455828982746341,
   //             1.138768685240342  , 0.573991962861691,
   //             1.522641846151249  , 0.657347408738955);

   // points2 = (cv::Mat_<float> (10, 2) << 5.239541394377807,   4.952122087513180,
   // 2.409419451041671,   5.424131589524132,
   // 3.252365668983476,   5.382115580979669,
   // 5.294847441491644,   4.723592364956511,
   // 3.236358303018495,   5.440276232429847,
   // 3.545647126029825,   5.273469089815864,
   // 4.987517232629965,   4.928677226681248,
   // 4.868698783886824,   4.814382625168967,
   // 6.474987110484027,   4.756703530439805,
   // 7.390912908272599,   4.837087993931045);

   // points1 = 1000 * points1;
   // points2 = 100 * points2;

    /*
     * Gt homography:

   1.0e+02 *

  [-0.000394568713042   0.034427299615096   0.852682108468499
  -0.010737816748454   0.051400467575938  -2.462277854245654
  -0.000014881986801   0.000066358177039   0.010000000000000]

     */
    cv::hconcat(points1, points2, points);

//    std::cout << cv::findHomography(points1, points2) << '\n';

    unsigned int points_size = (unsigned int) points.rows;
    int knn = 7;

    cv::Mat_<float> neighbors, neighbors_dists;
    NearestNeighbors nn;
    nn.getNearestNeighbors_nanoflann(points1, knn+1, neighbors, true, neighbors_dists);
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

    cv::Mat_<float> sorted_points;
    for (int i = 0; i < points_size; i++) {
        sorted_points.push_back(points.row(i));
    }

    Model * model;
    Sampler * sampler;
    Estimator * estimator;
    TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    int gt_inliers = getGTNumInliers (img_name, 3 /*model->threshold*/);


    // ---------------------- uniform ----------------------------------
//    model = new Model (3, 4, 0.99, 0, ESTIMATOR::Homography, SAMPLER::Uniform);
//    model->setStandardRansacLO(false);
//    model->setGraphCutLO(false);
//    model->setSprtLO(false);
//
//    sampler = new UniformSampler;
//    initUniform(sampler, model->sample_number, points_size);
//
//    estimator = new HomographyEstimator (points);
//
//    test (points, estimator, sampler, model, quality, termination_criteria,
//            img_name, gt_inliers);
    // --------------------------------------------------------------




// ------------------ prosac ---------------------
//    model = new Model (3, 4, 0.99, 0, ESTIMATOR::Homography, SAMPLER::Prosac);
//    model->setStandardRansacLO(false);
//    model->setGraphCutLO(false);
//    model->setSprtLO(false);
//    initProsac(sampler, model->sample_number, points_size);
//    ProsacSampler *prosac_sampler_ = (ProsacSampler *) sampler;
//
//    ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
//    prosac_termination_criteria_->initProsacTerminationCriteria (prosac_sampler_->getGrowthFunction(),
//                                                                 model, points_size);
//
//    termination_criteria = prosac_termination_criteria_;
//
//    estimator = new HomographyEstimator (sorted_points);
//    test (points, estimator, sampler, model, quality, termination_criteria,
//          img_name, gt_inliers);
    // -------------------------------------------------




//    getStatisticalResults(points, estimator, model, sampler, termination_criteria,
//                          quality, 300, true, false, gt_inliers, nullptr);

     storeResults();
}


/*
 * Store results from dataset to csv file.
 */
void storeResults () {
    std::vector<std::string> points_filename = getHomographyDatasetPoints();

    TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    Model *model = new Model (3, 4, 0.99, 0, ESTIMATOR::Homography, SAMPLER::Uniform);
    Tests tests;

    model->setStandardRansacLO(true);
    model->setGraphCutLO(true);
    model->setSprtLO(true);

    int N_runs = 50;

    std::ofstream results_total;
    results_total.open ("../results/homography/all_uniform.csv");
    results_total << tests.getComputerInfo();
    results_total << model->getName() << ",,,,,,,,,,,,,\n";
    results_total << "Runs for each image = " << N_runs << "\n";
    results_total << "Threshold for each image = " << model->threshold << "\n";
    results_total << "Desired probability for each image = " << model->desired_prob << "\n";
    results_total << "Standard LO = " << (bool) model->LO << "\n";
    results_total << "Graph Cut LO = " << (bool) model->GraphCutLO << "\n";
    results_total << "SPRT = " << (bool) model->SprtLO << "\n\n\n";

    results_total << "Filename,Avg num inl/gt,Std dev num inl,Med num inl,"
                     "Avg num iters,Std dev num iters,Med num iters,"
                     "Avg time (mcs),Std dev time,Med time,"
                     "Num fails\n";


    for (std::string img_name : points_filename) {
        std::cout << img_name << '\n';
        cv::Mat points1, points2;
        read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
        cv::hconcat(points1, points2, points1);

        Estimator * estimator = new HomographyEstimator (points1);
        Sampler * sampler = new UniformSampler;

        tests.initUniform(sampler, model->sample_number, points1.rows);

        int gt_inliers = getGTNumInliers (img_name, model->threshold);
        StatisticalResults * statistical_results = new StatisticalResults;

        tests.getStatisticalResults(points1, estimator, model, sampler, termination_criteria,
                              quality, N_runs, true, true, gt_inliers, statistical_results);

        // save to csv file
        results_total << img_name << ",";
        results_total << statistical_results->avg_num_inliers << " / " << gt_inliers << ",";
        results_total << statistical_results->std_dev_num_inliers << ",";
        results_total << statistical_results->median_num_inliers << ",";

        results_total << statistical_results->avg_num_iters << ",";
        results_total << statistical_results->std_dev_num_iters << ",";
        results_total << statistical_results->median_num_iters << ",";

        results_total << statistical_results->avg_time_mcs << ",";
        results_total << statistical_results->std_dev_time_mcs << ",";
        results_total << statistical_results->median_time_mcs << ",";

        results_total << statistical_results->num_fails << "\n";
    }

    results_total.close();
    delete model, quality, termination_criteria;
}



int getGTNumInliers (const std::string &filename, float threshold) {
    cv::Mat_<float> H;
    getMatrix3x3 ("../dataset/homography/"+filename+"_model.txt", H);
    Quality * quality = new Quality;
    cv::Mat points, points1, points2;
    read_points (points1, points2, "../dataset/homography/"+filename+"_pts.txt");
    cv::hconcat(points1, points2, points);
    int * inliers = new int [points.rows];
    Score * score = new Score;
    Estimator * estimator = new HomographyEstimator(points);
    Model * model = new Model (threshold, 4, 0.99, 0, ESTIMATOR::Homography, SAMPLER::Uniform);


    // In some ground truth model, the correct homography matrix is inverse.
    model->setDescriptor (H.inv());
    quality->GetModelScore (estimator, model, points, points.rows, *score, inliers, false);

    int score1 = score->inlier_number;

    model->setDescriptor (H);
    quality->GetModelScore (estimator, model, points, points.rows, *score, inliers, false);

    int score2 = score->inlier_number;

    delete inliers, score, model, quality, estimator;
    return std::max (score1, score2);
}