#include "Tests.h"

#include "../Usac/Estimator/Fundamental/SevenPoints.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../dataset/Dataset.h"

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

    Model *fundamental_model = new Model (5, 7, 0.99, 0, ESTIMATOR::Fundamental, SAMPLER::Uniform);
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(fundamental_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);
    uniform_sampler->initRandomGenerator();

    Estimator *fundamental_estimator = new FundamentalEstimator (points);
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    test (points, fundamental_estimator, uniform_sampler, fundamental_model, quality, termination_criteria,
            img_name, 0);

//    getAverageResults (points, fundamental_estimator, fundamental_model, uniform_sampler, termination_criteria, quality, 1000);

    // storeResultsFundamental ();
}
 
/*
 * Store results from dataset to csv file.
 */
void storeResultsFundamental () {
    std::vector<std::string> points_filename = getFundamentalDatasetPoints();

    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    Model *fundamental_model = new Model(3, 7, 0.99, 0, ESTIMATOR::Fundamental, SAMPLER::Uniform);

    std::ofstream results_total;
    results_total.open ("../results/fundamental/ALL.csv");
    results_total << "Filename,Number of Inliers (found / total points),Number of Iterations,Time (mcs),"
                     "Average Error (threshold = " << fundamental_model->threshold <<"),,,,,\n";

    for (std::string img_name : points_filename) {
        std::cout << img_name << '\n';
        cv::Mat points1, points2;
        read_points(points1, points2, "../dataset/fundamental/" + img_name);
        cv::hconcat(points1, points2, points1);

        Estimator *fundamental_estimator = new FundamentalEstimator(points1);
        Sampler *uniform_sampler = new UniformSampler;

        uniform_sampler->setSampleSize(fundamental_model->sample_number);
        uniform_sampler->setPointsSize(points1.rows);
        uniform_sampler->initRandomGenerator();

        Ransac ransac(fundamental_model, uniform_sampler, termination_criteria, quality, fundamental_estimator);
        ransac.run(points1);

        RansacOutput *ransacOutput = ransac.getRansacOutput();

        cv::Mat F = ransacOutput->getModel()->returnDescriptor();

//        std::cout << F << '\n';

        std::ofstream save_model;
        save_model.open("../results/fundamental/" + img_name.substr(0, img_name.find('.')) + ".csv");

        save_model << F.at<float>(0, 0) << "," << F.at<float>(0, 1) << "," << F.at<float>(0, 2) << ",\n"
                   << F.at<float>(1, 0) << "," << F.at<float>(1, 1) << "," << F.at<float>(1, 2) << ",\n"
                   << F.at<float>(2, 0) << "," << F.at<float>(2, 1) << "," << F.at<float>(2, 2) << ",\n";

        save_model << ransacOutput->getNumberOfInliers() << '\n';
        save_model << ransacOutput->getNumberOfIterations() << '\n';
        save_model << ransacOutput->getTimeMicroSeconds() << '\n';

        results_total << img_name << ",";
        results_total << ransacOutput->getNumberOfInliers() << "/" << points1.rows << ",";
        results_total << ransacOutput->getNumberOfIterations() << ",";
        results_total << ransacOutput->getTimeMicroSeconds() << ",";
        results_total << ransacOutput->getAverageError() << "\n";

        save_model.close();
    }

    results_total.close();
}