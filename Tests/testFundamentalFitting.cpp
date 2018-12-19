#include "Tests.h"

#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../dataset/Dataset.h"
#include "../Usac/Utils/NearestNeighbors.h"
#include "../Usac/Utils/Utils.h"

void storeResultsFundamental ();
void getGTInliersFromGTModelFundamental (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers);

void Tests::testFundamentalFitting() {
//    detectAndSaveFeatures(getAdelaidermfDataset());
//    detectAndSaveFeatures(getKusvod2Dataset());
//    exit (0);

    std::string img_name = "barrsmith";
    cv::Mat_<float> points1, points2, points;
//    getPointsNby6("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_pts.txt", points);
    read_points(points1, points2, "../dataset/adelaidermf/"+img_name+"_pts.txt");
//    read_points(points1, points2, "../dataset/fundamental/"+img_name+"_pts.txt");
        cv::hconcat(points1, points2, points);

//    LoadPointsFromFile(points, ("../dataset/adelaidermf/sift_update/"+img_name+"_spts.txt").c_str());


//        std::cout << "opencv " << cv::findFundamentalMat(points1, points2) << "\n\n";
    // GT F =
//    -0.000000837710510  -0.000022207792842   0.004660634536193
//    0.000020754689916   0.000000443452588  -0.012405343594436
//    -0.004753112593655   0.009718696572695   1.000000000000000

    cv::Mat_<float> sorted_points;
    densitySort (points, 4, sorted_points);

    std::cout << "points size = " << points.rows << "\n";

    float threshold = 5;
    float confidence = 0.95;
    int knn = 5;

    // ------------ get Ground truth inliers and model ----------------------
    std::vector<int> gt_inliers;
//    getGTInliersFromGTModelFundamental (img_name, points, threshold, gt_inliers);
    getInliers("../dataset/adelaidermf/"+img_name+"_pts.txt", gt_inliers);
//    getInliers("../dataset/fundamental/"+img_name+"_pts.txt", gt_inliers);
    // -------------------------------------------

    Model * model;

    // -------------------------- uniform -------------------------------------
    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
//    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Prosac);
//    sorted_points.copyTo(points);
    // ------------------------------------------------------------------------

    model->setStandardRansacLO(0);
    model->setGraphCutLO(0);
    model->setSprtLO(1);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Grid);

    test (points, model, img_name, true, gt_inliers);
//    test (points, model, img_name, false, gt_inliers);

//    getStatisticalResults(points, model, 400, true, gt_inliers, false, nullptr);

//     storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */
void storeResultsFundamental () {
    std::vector<std::string> points_filename = getKusvod2Dataset();
//    std::vector<std::string> points_filename = getAdelaidermfDataset();

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
//        read_points(points1, points2, "../dataset/adelaidermf/"+img_name+"_pts.txt");
//         cv::hconcat(points1, points2, points);
//        LoadPointsFromFile(points, ("../dataset/adelaidermf/sift_update/"+img_name+"_pts.txt").c_str());
        LoadPointsFromFile(points, ("../dataset/Lebeda/kusvod2/sift_update/"+img_name+"_pts.txt").c_str());

        cv::Mat_<float> sorted_points;
//        densitySort (points, 3, sorted_points);
//        LoadPointsFromFile(sorted_points, ("../dataset/adelaidermf/sift_update/"+img_name+"_spts.txt").c_str());
        LoadPointsFromFile(sorted_points, ("../dataset/Lebeda/kusvod2/sift_update/"+img_name+"_spts.txt").c_str());
        // points are already sorted for EVD
//        sorted_points = points.clone();

        // ------------ get Ground truth inliers and model ----------------------
        std::vector<int> gt_inliers_;
//        getInliers ("../dataset/adelaidermf/"+img_name+"_pts.txt", gt_inliers_);
        getGTInliersFromGTModelFundamental("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_model.txt", points, threshold, gt_inliers_);
        std::vector<int> gt_inliers_sorted_;
//        getInliers (img_name, gt_inliers_sorted_);
        getGTInliersFromGTModelFundamental("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_model.txt", sorted_points, threshold, gt_inliers_sorted_);
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

    bool GT = true;

    for (SAMPLER smplr : samplers) {
        for (NeighborsSearch neighbors_search : neighbors_searching) {
            for (auto cell_size : cell_sizes) {
                for (int l = 0; l < lo_combinations; l++) {
                    std::ofstream results_total;
                    std::ofstream results_matlab;
                    std::string name = "../results/kusvod2/";
                    name += Tests::sampler2string(smplr);
                    if (lo[l][0] == 1) name += "_lo";
                    if (lo[l][1] == 1) name += "_gc";
                    if (lo[l][2] == 1) name += "_sprt";
                    name += "_"+Tests::nearestNeighbors2string(neighbors_search) + "_c_sz_"+std::to_string(cell_size);

                    std::string mfname = name+"_m.csv";
                    std::string fname = name+".csv";

                    Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, smplr);
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

                        if (gt_inliers[img].size() == 0) GT = false;
                        else GT = true;

                        StatisticalResults * statistical_results = new StatisticalResults;
                        if (smplr == SAMPLER::Prosac) {
                            tests.getStatisticalResults(sorted_points_imgs[img], model, N_runs,
                                                        GT, gt_inliers_sorted[img], true, statistical_results);
                        } else {
                            tests.getStatisticalResults(points_imgs[img], model, N_runs,
                                                        GT, gt_inliers[img], true, statistical_results);
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


void getGTInliersFromGTModelFundamental (const std::string& filename, const cv::Mat& points, float threshold, std::vector<int> &gt_inliers) {
    cv::Mat gt_model;
    getMatrix3x3(filename, gt_model);
//    std::cout << gt_model << "\n";

    Estimator * estimator = new FundamentalEstimator (points);
    std::vector<int> inliers2;

    Quality::getInliers(estimator, gt_model, threshold, points.rows, gt_inliers);
    Quality::getInliers(estimator, gt_model.t(), threshold, points.rows, inliers2);

    if (inliers2.size() > gt_inliers.size()) {
        gt_inliers = inliers2;
    }
}