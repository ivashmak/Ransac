#include "Tests.h"

#include "../Usac/Estimator/Fundamental/SevenPoints.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../dataset/Dataset.h"
#include "../Usac/Utils/NearestNeighbors.h"

void storeResultsFundamental ();

void Tests::testFundamentalFitting() {
    std::string img_name = "ladysymon";
    
    cv::Mat points, points1, points2;
    read_points (points1, points2, "../dataset/fundamental/"+img_name+"_pts.txt");

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

    cv::hconcat(points1, points2, points);

    Model *model = new Model (5, 7, 0.99, 0, ESTIMATOR::Fundamental, SAMPLER::Uniform);
    Sampler *sampler = new UniformSampler;
    sampler->setSampleSize(model->sample_number);
    sampler->setPointsSize(points1.rows);
    sampler->initRandomGenerator();

    Estimator *estimator = new FundamentalEstimator (points);
    TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;

    // get neighbors
    NearestNeighbors nn;
    cv::Mat neighbors, neighbors_dist;
    nn.getNearestNeighbors_nanoflann(points1, model->k_nearest_neighbors, neighbors, false, neighbors_dist);
    //

    test (points, estimator, sampler, model, quality, termination_criteria, neighbors,
            img_name, -1);

//    getStatisticalResults (points, estimator, model, sampler, termination_criteria, quality, neighbors, 1000);

    // storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */
void storeResultsFundamental () {
    std::vector<std::string> points_filename = getFundamentalDatasetPoints();

    TerminationCriteria *termination_criteria = new StandardTerminationCriteria;
    Quality *quality = new Quality;
    Model *model = new Model (3, 4, 0.99, 7, ESTIMATOR::Fundamental, SAMPLER::Uniform);
    Tests tests;

    model->setStandardRansacLO(false);
    model->setGraphCutLO(false);
    model->setSprtLO(false);

    int N_runs = 50;

    std::ofstream results_total;
    results_total.open ("../results/fundamentla/all_uniform.csv");
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

    NearestNeighbors nn;


    for (std::string img_name : points_filename) {
        std::cout << img_name << '\n';
        cv::Mat points1, points2;
        read_points (points1, points2, "../dataset/fundamental/"+img_name+"_pts.txt");
        cv::hconcat(points1, points2, points1);

        Estimator * estimator = new FundamentalEstimator (points1);
        Sampler * sampler = new UniformSampler;

        tests.initUniform(sampler, model->sample_number, points1.rows);

        int gt_inliers = -1;

        // get neighbors
        cv::Mat neighbors, neighbors_dist;
        nn.getNearestNeighbors_nanoflann(points1, model->k_nearest_neighbors, neighbors, false, neighbors_dist);
        //

        StatisticalResults * statistical_results = new StatisticalResults;
        tests.getStatisticalResults(points1, estimator, model, sampler, termination_criteria,
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
    }

    results_total.close();
    delete model, quality, termination_criteria;
}