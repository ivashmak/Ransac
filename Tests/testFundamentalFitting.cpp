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
void getFundamentalGT (const std::string& filename, const cv::Mat& points, float threshold, int * gt_inliers, cv::Mat &gt_model);

void Tests::testFundamentalFitting() {
    std::string img_name = "booksh";
    cv::Mat_<float> points1, points2, points;
    getPointsNby6("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_pts.txt", points);

//    read_points (points1, points2, "../dataset/fundamental/"+img_name+"_pts.txt");

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
//    std::cout << "opencv " << cv::findFundamentalMat(points1, points2) << "\n\n";
    // GT F =
//    -0.000000837710510  -0.000022207792842   0.004660634536193
//    0.000020754689916   0.000000443452588  -0.012405343594436
//    -0.004753112593655   0.009718696572695   1.000000000000000

//    cv::hconcat(points1, points2, points);
    cv::Mat_<float> sorted_points;
    densitySort (points, 4, sorted_points);

    std::cout << "points size = " << points.rows << "\n";

    float threshold = 3;
    float confidence = 0.95;
    int knn = 7;

    cv::Mat_<float> gt_model;
    int gt_inliers_;
    getFundamentalGT("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_model.txt", points, threshold, &gt_inliers_, gt_model);

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


//    test (points, model, img_name, true, gt_model);
//
//    getStatisticalResults(points, model, 400, true, false, gt_model, nullptr);

     storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */
void storeResultsFundamental () {
    std::vector<std::string> points_filename = getKusvod2Dataset();
    Tests tests;
    NearestNeighbors nn;

    std::vector<cv::Mat_<float>> points_imgs;
    std::vector<cv::Mat_<float>> sorted_points_imgs;
    std::vector<cv::Mat_<float>> gt_models;
    std::vector<int> gt_inliers;

    int N_runs = 100;
    int knn = 7;
    float threshold = 3;
    float confidence = 0.95;

    for (const std::string &img_name : points_filename) {
        std::cout << "get points for " << img_name << "\n";
        cv::Mat points;
        getPointsNby6 ("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_pts.txt", points);

        cv::Mat_<float> sorted_points;
        densitySort(points, 4, sorted_points);

        // ------------ get Ground truth inliers and model ----------------------
        cv::Mat_<float> gt_model;
        int gt_inliers_;
        getFundamentalGT("../dataset/Lebeda/kusvod2/"+img_name+"_vpts_model.txt", points, threshold, &gt_inliers_, gt_model);
        // -------------------------------------------

        gt_inliers.push_back(gt_inliers_);
        gt_models.push_back(gt_model);
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
            std::string mfname = "../results/kusvod2/"+tests.sampler2string(smplr)+ "_"+
                                 std::to_string(lo[l][0])+std::to_string(lo[l][1])+std::to_string(lo[l][2])+"_m.csv";
            std::string fname = "../results/kusvod2/"+tests.sampler2string(smplr)+"_"+
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

            results_total << "Filename,Avg num inl/gt,Std dev num inl,Med num inl,"
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
                    tests.getStatisticalResults(sorted_points_imgs[img], model, N_runs, true, true, gt_models[img], statistical_results);
                } else {
                    tests.getStatisticalResults(points_imgs[img], model, N_runs, true, true, gt_models[img], statistical_results);
                }

                // save to csv file
                results_total << img_name << ",";
                results_total << statistical_results->avg_num_inliers << " / " << gt_inliers[img] << ",";
                results_total << statistical_results->std_dev_num_inliers << ",";
                results_total << statistical_results->median_num_inliers << ",";

                results_total << statistical_results->avg_num_iters << ",";
                results_total << statistical_results->std_dev_num_iters << ",";
                results_total << statistical_results->median_num_iters << ",";

                results_total << statistical_results->avg_num_lo_iters << ",";
                results_total << statistical_results->std_dev_num_lo_iters << ",";
                results_total << statistical_results->median_num_lo_iters << ",";

                results_total << statistical_results->avg_time_mcs << ",";
                results_total << statistical_results->std_dev_time_mcs << ",";
                results_total << statistical_results->median_time_mcs << ",";

                results_total << statistical_results->avg_error << ",";
                results_total << statistical_results->std_dev_error << ",";
                results_total << statistical_results->median_error << ",";

                results_total << statistical_results->worst_case_num_inliers << ",";
                results_total << statistical_results->worst_case_error << ",";

                results_total << statistical_results->num_fails << "\n";

                // save results for matlab
                results_matlab << img_name << ",";
                results_matlab << statistical_results->avg_num_inliers << ",";
                results_matlab << statistical_results->avg_num_iters << ",";
                results_matlab << statistical_results->avg_num_lo_iters << ",";
                results_matlab << statistical_results->avg_time_mcs << ",";
                results_matlab << statistical_results->avg_error << ",";
                results_matlab << statistical_results->num_fails << "\n";

                img++;
            }

            results_total.close();
            results_matlab.close();
        }
    }
}


void getFundamentalGT (const std::string& filename, const cv::Mat& points, float threshold, int * gt_inliers, cv::Mat &gt_model) {
    getMatrix3x3(filename, gt_model);
//    std::cout << gt_model << "\n";

    Estimator * estimator = new FundamentalEstimator (points);
    int inliers1 = 0, inliers2 = 0;

    estimator->setModelParameters(gt_model);

    for (int p = 0; p < points.rows; p++) {
        inliers1 += (estimator->GetError(p) < threshold);
    }

    estimator->setModelParameters(gt_model.t());

    for (int p = 0; p < points.rows; p++) {
        inliers2 += (estimator->GetError(p) < threshold);
    }

    if (inliers1 > inliers2) {
        *gt_inliers = inliers1;
    } else {
        *gt_inliers = inliers2;
        gt_model = cv::Mat (gt_model.t());
    }
}