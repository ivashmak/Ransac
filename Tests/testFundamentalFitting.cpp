#include "Tests.h"

#include "../Usac/Estimator/NPointsAlgorithms/SevenPointsFundamentalEstimation.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../dataset/Dataset.h"

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename);
void storeResultsFundamental ();

void Tests::testFundamentalFitting() {
    std::string img_name = "johnssonb";
    std::string points_filename = "../dataset/fundamental/"+img_name+"_pts.txt";
    std::vector<std::string> images_filename;
    images_filename.push_back("../dataset/fundamental/"+img_name+"A.png");
    images_filename.push_back("../dataset/fundamental/"+img_name+"B.png");

    cv::Mat points, points1, points2;
    read_points (points1, points2, points_filename);
    cv::hconcat(points1, points2, points);

    bool LO = false;

    Model *fundamental_model = new Model (5, 7, 0.99, 0, "fundamental");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(fundamental_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);
    uniform_sampler->initRandomGenerator();

    testFundamental (points, fundamental_model, uniform_sampler, images_filename, points_filename);

    Estimator *fundamental_estimator = new FundamentalEstimator (points);
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

//    runNTimes(points, fundamental_estimator, fundamental_model, uniform_sampler, termination_criteria, quality, 1000, LO);

//    storeResultsFundamental ();
}

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename) {
    Estimator * fundamental_estimator = new FundamentalEstimator (points);
    Drawing drawing;
    Logging logResult;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality * quality = new Quality;

    Ransac ransac (*model, *sampler, *termination_criteria, *quality, *fundamental_estimator);
    ransac.run(points);

    RansacOutput *ransacOutput = ransac.getRansacOutput();

    std::cout << model->model_name << " time: ";
    ransacOutput->printTime();
    std::cout << model->model_name << " iterations: " << ransacOutput->getNumberOfIterations() <<
              " (" << ((int)ransacOutput->getNumberOfIterations () -(int)ransacOutput->getNumberOfLOIterations ()) << 
              " + " << ransacOutput->getNumberOfLOIterations () << " (" << ransacOutput->getLORuns() << " lo inner + iterative runs)) \n";
    
    std::cout << model->model_name << " points under threshold: " << ransacOutput->getNumberOfInliers() << "\n";
    std::cout << "Average error " << quality->getAverageError(fundamental_estimator, ransacOutput->getModel(), points, points.getMat().rows) << "\n";

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";

    cv::Mat pts = points.getMat();
    drawing.drawEpipolarLines(images_filename, pts.colRange(0,2), pts.colRange(2,4), model->returnDescriptor());
}

void storeResultsFundamental () {
    std::vector<std::string> points_filename = getFundamentalDatasetPoints();

    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    bool LO = false;

    Model *fundamental_model = new Model(3, 7, 0.99, 0, "fundamental");

    std::ofstream results_total;
    results_total.open ("../results/fundamental/ALL.csv");
    results_total << "LO " << LO << '\n';
    results_total << "Filename,Number of Inliers (found / total points),Number of Iterations,Time (mcs),"
                     "Average Error (threshold = " << fundamental_model->threshold <<"),,,,,\n";

    for (std::string img_name : points_filename) {
        cv::Mat points1, points2;
        read_points(points1, points2, "../dataset/fundamental/" + img_name);
        cv::hconcat(points1, points2, points1);

        Estimator *fundamental_estimator = new FundamentalEstimator(points1);
        Sampler *uniform_sampler = new UniformSampler;

        uniform_sampler->setSampleSize(fundamental_model->sample_number);
        uniform_sampler->setPointsSize(points1.rows);
        uniform_sampler->initRandomGenerator();

        Ransac ransac(*fundamental_model, *uniform_sampler, *termination_criteria, *quality, *fundamental_estimator);
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
        results_total << quality->getAverageError(fundamental_estimator, ransacOutput->getModel(), points1, points1.rows) << "\n";

        save_model.close();
    }

    results_total.close();
}