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
#include "../Usac/Sampler/NapsacSampler.h"

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
    std::cout << "points size " << points_size << "\n";
    int knn = 40;

    cv::Mat_<float> neighbors, neighbors_dists;
    NearestNeighbors nn;
    // get nearest neighbors by first correspondence?
    nn.getNearestNeighbors_nanoflann(points, knn, neighbors, true, neighbors_dists);

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
        sorted_points.push_back(points.row(sorted_idx[i]));
    }

    Model * model;
    Sampler * sampler;
    Estimator * estimator;
    TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    int gt_inliers = getGTNumInliers (img_name, 3 /*model->threshold*/);

//     ---------------------- uniform ----------------------------------
//   model = new Model (3, 4, 0.99, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
//   model->setStandardRansacLO(0);
//   model->setGraphCutLO(0);
//   model->setSprtLO(0);
//
//   sampler = new UniformSampler;
//   initUniform(sampler, model->sample_number, points_size);
//
//   estimator = new HomographyEstimator (points);

//    test (points, estimator, sampler, model, quality, termination_criteria, neighbors,
//            img_name, gt_inliers);
    // --------------------------------------------------------------



//     ---------------------- napsac ----------------------------------
    model = new Model (3, 4, 0.99, knn, ESTIMATOR::Homography, SAMPLER::Napsac);
    model->setStandardRansacLO(0);
    model->setGraphCutLO(0);
    model->setSprtLO(0);

//    std::cout << neighbors << "\n";

    initNapsac(sampler, neighbors, model->k_nearest_neighbors, model->sample_number);

    estimator = new HomographyEstimator (points);

//    test (points, estimator, sampler, model, quality, termination_criteria, neighbors,
//          img_name, gt_inliers);
    // --------------------------------------------------------------



// ------------------ prosac ---------------------
//     model = new Model (3, 4, 0.99, knn, ESTIMATOR::Homography, SAMPLER::Prosac);
//     model->setStandardRansacLO(0);
//     model->setGraphCutLO(0);
//     model->setSprtLO(0);
//        // get neigbors for sorted points
//        nn.getNearestNeighbors_nanoflann(sorted_points, model->k_nearest_neighbors, neighbors, false, neighbors_dists);
   //     estimator = new HomographyEstimator (sorted_points);

// //    initProsac(sampler, model->sample_number, points_size);
//     initSampler(sampler, model, points_size, points, neighbors);

//     ProsacTerminationCriteria * prosac_termination_criteria_ = new ProsacTerminationCriteria;
//     prosac_termination_criteria_->initProsacTerminationCriteria (((ProsacSampler *)sampler)->getGrowthFunction(),
//                                                                  model, points_size, estimator);
//     termination_criteria = prosac_termination_criteria_;

//    test (points, estimator, sampler, model, quality, termination_criteria, neighbors,
//          img_name, gt_inliers);
//     -------------------------------------------------
//
    getStatisticalResults(points, estimator, model, sampler, termination_criteria,
                          quality, neighbors, 500, true, false, gt_inliers, nullptr);
//
//     storeResults();

}


/*
// * Store results from dataset to csv file.
 */
void storeResults () {
    std::vector<std::string> points_filename = getHomographyDatasetPoints();
    Tests tests;
    NearestNeighbors nn;

    int N_runs = 100;

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
    samplers.push_back(SAMPLER::Prosac);

    int lo_combinations = 5;
    bool lo[lo_combinations][3] = {
            {0, 0, 0},
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
            {1, 1, 1},
    };

    for (SAMPLER smplr : samplers) {
        for (int l = 0; l < lo_combinations; l++) {
            std::ofstream results_total;
            std::ofstream results_matlab;
            std::string mfname = "../results/homography/"+tests.sampler2string(smplr)+ "_"+
                    std::to_string(lo[l][0])+std::to_string(lo[l][1])+std::to_string(lo[l][2])+"_m.csv";
            std::string fname = "../results/homography/"+tests.sampler2string(smplr)+"_"+
                    std::to_string(lo[l][0])+std::to_string(lo[l][1])+std::to_string(lo[l][2])+".csv";

            results_matlab.open (mfname);
            results_total.open (fname);
            int knn = 7;

            Model *model = new Model (3, 4, 0.99, knn, ESTIMATOR::Homography, smplr);
            model->setStandardRansacLO(lo[l][0]);
            model->setGraphCutLO(lo[l][1]);
            model->setSprtLO(lo[l][2]);

            results_total << tests.getComputerInfo();
            results_total << model->getName() << "\n";
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

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << lo[l][0] << " " << lo[l][1] << " " << lo[l][2] << "\n";

            for (std::string img_name : points_filename) {
                std::cout << img_name << '\n';

                cv::Mat points1, points2, points;
                read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
                cv::hconcat(points1, points2, points);
                unsigned int points_size = points1.rows;
                Estimator * estimator;
                TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
                Quality *quality = new Quality;

                // get neighbors
                cv::Mat neighbors, neighbors_dists;
                nn.getNearestNeighbors_nanoflann(points, knn, neighbors, true, neighbors_dists);
                //

                if (smplr == SAMPLER::Prosac) {
                    std::vector<int> sorted_idx(points_size);
                    std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
                    float sum1, sum2;
                    int idxa, idxb;
                    float *neighbors_dists_ptr = (float *) neighbors_dists.data;
                    std::sort(sorted_idx.begin(), sorted_idx.end(), [&](int a, int b) {
                        sum1 = 0, sum2 = 0;
                        idxa = knn * a, idxb = knn * b;
                        for (int i = 0; i < 4; i++) {
                            sum1 += neighbors_dists_ptr[idxa + i];
                            sum2 += neighbors_dists_ptr[idxb + i];
                        }
                        return sum1 < sum2;
                    });

                    cv::Mat_<float> sorted_points;
                    for (int i = 0; i < points_size; i++) {
                        sorted_points.push_back(points.row(sorted_idx[i]));
                    }
                    sorted_points.copyTo(points);
                }

                estimator = new HomographyEstimator(points);
                Sampler * sampler;
                tests.initSampler(sampler, model, points_size, points, neighbors);

                int gt_inliers = getGTNumInliers (img_name, model->threshold);

                StatisticalResults * statistical_results = new StatisticalResults;

                tests.getStatisticalResults(points, estimator, model, sampler, termination_criteria,
                                                quality, neighbors, N_runs, true, true, gt_inliers, statistical_results);

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

                results_matlab << img_name << ",";
                results_matlab << statistical_results->avg_num_inliers << ",";
                results_matlab << statistical_results->std_dev_num_inliers << ",";

                results_matlab << statistical_results->avg_num_iters << ",";
                results_matlab << statistical_results->std_dev_num_iters << ",";

                results_matlab << statistical_results->avg_time_mcs << ",";
                results_matlab << statistical_results->std_dev_time_mcs << ",";

                results_matlab << statistical_results->num_fails << "\n";
            }

            results_total.close();
            results_matlab.close();
        }
    }
}



int getGTNumInliers (const std::string &filename, float threshold) {
    cv::Mat_<float> H;
    getMatrix3x3 ("../dataset/homography/"+filename+"_model.txt", H);

    Quality * quality = new Quality;
    cv::Mat points, points1, points2;
    read_points (points1, points2, "../dataset/homography/"+filename+"_pts.txt");

    cv::hconcat(points1, points2, points);

    Model * model = new Model (threshold, 4, 0.99, 5, ESTIMATOR::Homography, SAMPLER::Uniform);
    Score * score = new Score;
    Estimator * estimator = new HomographyEstimator(points);

    quality->init(points.rows, 3, estimator);

    // In some ground truth model, the correct homography matrix is inverse.
    model->setDescriptor (H.inv());
    quality->getNumberInliers (score, model, false, nullptr);

    int score1 = score->inlier_number;
//    std::cout << score1 << " = score 1 \n";

    model->setDescriptor (H);
    quality->getNumberInliers (score, model, false, nullptr);

    int score2 = score->inlier_number;
//    std::cout << score2 << " = score 2 \n";

    delete score, model, quality, estimator;
    return std::max (score1, score2);
}