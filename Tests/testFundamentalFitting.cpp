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
    std::string img_name = "napiera";
    cv::Mat_<float> points1, points2, points;
//    getPointsNby6("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_pts.txt", points);
//    read_points(points1, points2, "../dataset/adelaidermf/"+img_name+"_pts.txt");
    read_points(points1, points2, "../dataset/fundamental/"+img_name+"_pts.txt");

//    points1 = (cv::Mat_<float> (10, 2) <<
//            3.020519301751723,   2.596786959172094,
//            2.621351894036264,   2.757951047738636,
//            3.605800286632482,   3.791987038121546,
//            2.475761663001728,   2.340303837863212,
//            2.746552415058527,   3.013370222948234,
//            2.683418010219510,   1.691394650172999,
//            3.812414245722914,   2.044879731424070,
//            1.948319539540133,   3.429684834622727,
//            3.176411253015324,   1.827122055282097,
//            3.182138632063733,   2.429665766838721);
//
//    points2 = (cv::Mat_<float> (10, 2) <<
//            1.920459921166140,   3.034291347636251,
//            1.533733195053770,   3.180365190371057,
//            2.456364070816144,   4.151752011453615,
//            1.399696530725253,   2.783069240578678,
//            1.650942479317877,   3.423627688352502,
//            1.613597545828692,   2.186102119417673,
//            2.654105419292067,   2.549231570092227,
//            0.841556026244410,   3.830470840421660,
//            2.076636085082961,   2.330698186038357,
//            2.074849087666451,   2.882361072224565);
//
//    points1 = 100 * points1;
//    points2 = 100 * points2;
//
    cv::hconcat(points1, points2, points);

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
    int knn = 7;

    // ------------ get Ground truth inliers and model ----------------------
    std::vector<int> gt_inliers;
//    getGTInliersFromGTModelFundamental (img_name, points, threshold, gt_inliers);
//    getInliers("../dataset/adelaidermf/"+img_name+"_pts.txt", gt_inliers);
    getInliers("../dataset/fundamental/"+img_name+"_pts.txt", gt_inliers);
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
    model->setSprtLO(0);


    test (points, model, img_name, true, gt_inliers);
//    test (points, model, img_name, false, std::vector<int>());

//    getStatisticalResults(points, model, 400, true, gt_inliers, false, nullptr);

//     storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */
void storeResultsFundamental () {
//    std::vector<std::string> points_filename = getKusvod2Dataset();
    std::vector<std::string> points_filename = getAdelaidermfDataset();

    Tests tests;
    Logging log;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<std::vector<int>> gt_inliers;

    int N_runs = 50;
    int knn = 4;
    float threshold = 3;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat points1, points2, points;
        read_points(points1, points2, "../dataset/adelaidermf/"+img_name+"_pts.txt");
        cv::hconcat(points1, points2, points);
//        getPointsNby6 ("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_pts.txt", points);

        cv::Mat_<float> sorted_points;
        densitySort(points, 4, sorted_points);

        // ------------ get Ground truth inliers and model ----------------------
        std::vector<int> gt_inliers_;
//        getGTInliersFromGTModelFundamental (img_name, points, threshold, gt_inliers_);
        getInliers("../dataset/adelaidermf/"+img_name+"_pts.txt", gt_inliers_);
        // -------------------------------------------

        gt_inliers.push_back(gt_inliers_);
        points_imgs.push_back(points);
        sorted_points_imgs.push_back(sorted_points);
    }

    std::vector<SAMPLER> samplers;
    samplers.push_back(SAMPLER::Uniform);
//    samplers.push_back(SAMPLER::Prosac);

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
            std::string mfname = "../results/adelaidermf/"+tests.sampler2string(smplr)+ "_"+
                                 std::to_string(lo[l][0])+std::to_string(lo[l][1])+std::to_string(lo[l][2])+"_m.csv";
            std::string fname = "../results/adelaidermf/"+tests.sampler2string(smplr)+"_"+
                                std::to_string(lo[l][0])+std::to_string(lo[l][1])+std::to_string(lo[l][2])+".csv";

            results_matlab.open (mfname);
            results_total.open (fname);

            Model *model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, smplr);
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

            results_total << "Filename,GT Inl,Avg num inl/gt,Std dev num inl,Med num inl,"
                             "Avg num iters,Std dev num iters,Med num iters,"
                             "Avg num LO iters,Std dev num LO iters,Med num LO iters,"
                             "Avg time (mcs),Std dev time,Med time,"
                             "Avg err,Std dev err,Med err,"
                             "Worst case num Inl,Worst case Err,"
                             "Num fails\n";

            std::cout << tests.sampler2string(smplr) << "\n";
            std::cout << lo[l][0] << " " << lo[l][1] << " " << lo[l][2] << "\n";

            int img = 0;
            for (const std::string &img_name : points_filename) {
                std::cout << img_name << "\n";

                StatisticalResults * statistical_results = new StatisticalResults;
                if (smplr == SAMPLER::Prosac) {
                    tests.getStatisticalResults(sorted_points_imgs[img], model, N_runs,
                                                true, gt_inliers[img], true, statistical_results);
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