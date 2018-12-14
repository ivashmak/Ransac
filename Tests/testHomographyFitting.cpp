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
#include "../Detector/detector.h"
#include "../Usac/Utils/Utils.h"

void storeResults ();
void getGTInliersFromGTModelHomography (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers);

void detectAndSaveFeatures (const std::vector<std::string>& dataset) {
    for (const std::string &name : dataset) {
        std::cout << name << "\n";
        cv::Mat points;

        cv::Mat image2, image1 = cv::imread ("../dataset/homography/"+name+"A.png");
        if (image1.empty()) {
            image1 = cv::imread ("../dataset/homography/"+name+"A.jpg");
            if (image1.empty()) {
                std::cout << "invalid image name!\n";
                exit (111);
            }
            image2 = cv::imread ("../dataset/homography/"+name+"B.jpg");
        } else {
            image2 = cv::imread ("../dataset/homography/"+name+"B.png");
        }

        DetectFeatures("../dataset/homography/sift_update/"+name+"_pts.txt", image1, image2, points);
    }
}

void Tests::testHomographyFitting() {

//    detectAndSaveFeatures(getHomographyDatasetPoints());
//    exit (0);

    std::string img_name = "Brussels";
    cv::Mat points, points1, points2;
//    read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
//    cv::hconcat(points1, points2, points);

    LoadPointsFromFile(points, ("../dataset/homography/sift_update/"+img_name+"_pts.txt").c_str());

    unsigned int points_size = (unsigned int) points.rows;
    std::cout << "points size " << points_size << "\n";

    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

//    cv::Mat_<float> sorted_points;
//    densitySort(points, 3, sorted_points);


// ------------ get Ground truth inliers and model ----------------------
    std::vector<int> gt_inliers;
    getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", points, threshold, gt_inliers);
    // -------------------------------------------
    std::cout << "gt inliers " << gt_inliers.size() << "\n";

    Model * model;

//     ---------------------- uniform ----------------------------------
   model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
    // --------------------------------------------------------------


    // ---------------------- napsac ----------------------------------
//    model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Napsac);
    // --------------------------------------------------------------


// ------------------ prosac ---------------------
//     model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Prosac);
//     -------------------------------------------------


     model->setStandardRansacLO(0);
     model->setGraphCutLO(0);
     model->setSprtLO(0);

     test (points, model, img_name, true, gt_inliers);

//    getStatisticalResults(points, model, 100, true, gt_inliers, false, nullptr);

//     storeResults();
}


/*
// * Store results from dataset to csv file.
 */
void storeResults () {
    std::vector<std::string> points_filename = getHomographyDatasetPoints();
    Tests tests;
    Logging log;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_inliers_sorted;

    int N_runs = 100;
    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat_<float> points1, points2, points;
//        read_points (points1, points2, "../dataset/homography/sift_update/"+img_name+"_pts.txt");
        LoadPointsFromFile(points, ("../dataset/homography/sift_update/"+img_name+"_pts.txt").c_str());
        // read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
        // cv::hconcat(points1, points2, points);

        cv::Mat_<float> sorted_points;
//        densitySort (points, 3, sorted_points);
        LoadPointsFromFile(sorted_points, ("../dataset/homography/sift_update/"+img_name+"_spts.txt").c_str());

        // ------------ get Ground truth inliers and model ----------------------
        std::vector<int> gt_inliers_;
        getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", points, threshold, gt_inliers_);
        std::vector<int> gt_inliers_sorted_;
        getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", sorted_points, threshold, gt_inliers_sorted_);
        // -------------------------------------------

        gt_inliers.push_back(gt_inliers_);
        gt_inliers_sorted.push_back(gt_inliers_sorted_);
        points_imgs.push_back(points);
        sorted_points_imgs.push_back(sorted_points);
    }

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
    samplers.push_back(SAMPLER::Prosac);

    int lo_combinations = 2;
    bool lo[lo_combinations][3] = {
            {0, 1, 0},
            {0, 1, 1},
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

            Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, smplr);
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
            results_total << "SPRT = " << (bool) model->Sprt << "\n\n\n";

            results_total << "Filename,GT Inl,Avg num inl/gt,Std dev num inl,Med num inl,"
                             "Avg num iters,Std dev num iters,Med num iters,"
                             "Avg num LO iters,Std dev num LO iters,Med num LO iters,"
                             "Avg time (mcs),Std dev time,Med time,"
                             "Avg err,Std dev err,Med err,"
                             "Worst case num Inl,Worst case Err,"
                             "Num fails (<10%),Num fails (<25%),Num fails (<50%)\n";

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << lo[l][0] << " " << lo[l][1] << " " << lo[l][2] << "\n";

            int img = 0;
            for (const std::string &img_name : points_filename) {

                std::cout << img_name << "\n";

                StatisticalResults * statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    tests.getStatisticalResults(sorted_points_imgs[img], model, N_runs,
                                                true, gt_inliers_sorted[img], true, statistical_results);
                } else {
                    tests.getStatisticalResults(points_imgs[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
                }

                // save to csv file
                results_total << img_name << ",";
                results_total << gt_inliers[img].size() << ",";
                log.saveResultsCSV(results_total, statistical_results);

                // save results for matlab
                results_matlab << img_name << ",";
                log.saveResultsMatlab(results_matlab, statistical_results);

                img++;
            }

            results_total.close();
            results_matlab.close();
        }
    }
}


void getGTInliersFromGTModelHomography (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers) {
    cv::Mat gt_model;
    getMatrix3x3(filename, gt_model);
//    std::cout << gt_model << "\n";

    Estimator * estimator = new HomographyEstimator (points);
    std::vector<int> inliers2;

    Quality::getInliers(estimator, gt_model, threshold, points.rows, gt_inliers);
    Quality::getInliers(estimator, gt_model.inv(), threshold, points.rows, inliers2);

//    std::cout << "gt inl1 " << gt_inliers.size() << "\n";
//    std::cout << "gt inl2 " << inliers2.size() << "\n";

    if (inliers2.size() > gt_inliers.size()) {
        gt_inliers = inliers2;
    }
}