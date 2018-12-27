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

void Tests::testHomographyFitting() {

//    detectAndSaveFeatures(getHomographyDatasetPoints());
//    exit (0);
    DATASET dataset = DATASET::Homogr;
    std::string img_name = "graf";

    ImageData gt_data (dataset, img_name);
    std::vector<int> gt_inliers = gt_data.getGTInliers();
    cv::Mat points = gt_data.getPoints();

    // points are already sorted
//    readEVDpoints(points, "../dataset/EVD/EVD_tentatives/"+img_name+".png_m.txt");

//    LoadPointsFromFile(points, ("../dataset/homography/sift_update/"+img_name+"_spts.txt").c_str());

    unsigned int points_size = (unsigned int) points.rows;
    std::cout << "points size " << points_size << "\n";

    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

    cv::Mat_<float> sorted_points;
    densitySort(points, 3, sorted_points);


// ------------ get Ground truth inliers and model ----------------------
//    getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", points, threshold, gt_inliers);
//    getGTInliersFromGTModelHomography ("../dataset/EVD/h/"+img_name+".txt", points, threshold, gt_inliers);
    // -------------------------------------------
    std::cout << "gt inliers " << gt_inliers.size() << "\n";

    Model * model;

//     ---------------------- uniform ----------------------------------
//   model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Uniform);
    // --------------------------------------------------------------


    // ---------------------- napsac ----------------------------------
    model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Napsac);
    // --------------------------------------------------------------


// ------------------ prosac ---------------------
//     model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, SAMPLER::Prosac);
//     -------------------------------------------------


     model->setStandardRansacLO(0);
     model->setGraphCutLO(0);
     model->setSprtLO(0);
     model->setCellSize(50);
     model->setNeighborsType(NeighborsSearch::Grid);

     test (points, model, img_name, dataset, true, gt_inliers);
//
//    getStatisticalResults(points, model, 500, true, gt_inliers, false, nullptr);

//     storeResults();
}


/*
// * Store results from dataset to csv file.
 */
void storeResults () {
//    std::vector<std::string> points_filename = getHomographyDatasetPoints();
    std::vector<std::string> points_filename = getEVDDataset();
//    std::vector<std::string> points_filename = getProblemHomographyDatasetPoints();

    Tests tests;
    Logging log;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;
    std::vector<std::vector<int>> gt_inliers_sorted;

    int N_runs = 50;
    int knn = 5;
    float threshold = 2;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat_<float> points1, points2, points;
//        LoadPointsFromFile(points, ("../dataset/homography/sift_update/"+img_name+"_pts.txt").c_str());
        readEVDpoints(points, "../dataset/EVD/EVD_tentatives/"+img_name+".png_m.txt");
//         read_points (points1, points2, "../dataset/homography/"+img_name+"_pts.txt");
//         cv::hconcat(points1, points2, points);

        cv::Mat_<float> sorted_points;
//        densitySort (points, 3, sorted_points);
//        LoadPointsFromFile(sorted_points, ("../dataset/homography/sift_update/"+img_name+"_spts.txt").c_str());
        // points are already sorted for EVD
        sorted_points = points.clone();

        // ------------ get Ground truth inliers and model ----------------------
        std::vector<int> gt_inliers_;
//        getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", points, threshold, gt_inliers_);
        getGTInliersFromGTModelHomography ("../dataset/EVD/h/"+img_name+".txt", points, threshold, gt_inliers_);
        std::vector<int> gt_inliers_sorted_;
//        getGTInliersFromGTModelHomography ("../dataset/homography/"+img_name+"_model.txt", sorted_points, threshold, gt_inliers_sorted_);
        gt_inliers_sorted_ = gt_inliers_;
        // -------------------------------------------
        std::cout << "gt inliers size = " << gt_inliers_.size() << "\n";

        gt_inliers.push_back(gt_inliers_);
        gt_inliers_sorted.push_back(gt_inliers_sorted_);
        points_imgs.push_back(points);
        sorted_points_imgs.push_back(sorted_points);
    }

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
    samplers.push_back(SAMPLER::Prosac);

    std::vector<NeighborsSearch> neighbors_searching;
    neighbors_searching.push_back(NeighborsSearch::Grid);
    neighbors_searching.push_back(NeighborsSearch::Nanoflann);

    std::vector<int> cell_sizes;
//    cell_sizes.push_back(25);
    cell_sizes.push_back(50);
//    cell_sizes.push_back(100);

    int lo_combinations = 2;
    bool lo[lo_combinations][3] = {
            {0, 1, 0},
            {0, 1, 1},
    };

    for (SAMPLER smplr : samplers) {
        for (NeighborsSearch neighbors_search : neighbors_searching) {
            for (auto cell_size : cell_sizes) {
                for (int l = 0; l < lo_combinations; l++) {
                    std::ofstream results_total;
                    std::ofstream results_matlab;
                    std::string name = "../results/EVD/";
                    name += Tests::sampler2string(smplr);
                    if (lo[l][0] == 1) name += "_lo";
                    if (lo[l][1] == 1) name += "_gc";
                    if (lo[l][2] == 1) name += "_sprt";
                    name += "_"+Tests::nearestNeighbors2string(neighbors_search) + "_c_sz_"+std::to_string(cell_size);

                    std::string mfname = name+"_m.csv";
                    std::string fname = name+".csv";

                    Model *model = new Model (threshold, 4, confidence, knn, ESTIMATOR::Homography, smplr);
                    model->setStandardRansacLO(lo[l][0]);
                    model->setGraphCutLO(lo[l][1]);
                    model->setSprtLO(lo[l][2]);
                    model->setNeighborsType(neighbors_search);
                    model->setCellSize(cell_size);

                    results_matlab.open (mfname);
                    results_total.open (fname);

                    log.saveHeadOfCSV (results_total, model, N_runs);

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
                    std::cout << "------------------------------------------------\n";

                    results_total.close();
                    results_matlab.close();
                }

                if (neighbors_search == NeighborsSearch::Nanoflann) break;
            }
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