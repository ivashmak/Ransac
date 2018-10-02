#include "Tests.h"

#include "../Usac/Estimator/NPointsAlgorithms/SevenPointsAlgorithm.h"
#include "../Detector/ReadPoints.h"
#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"
#include "../Usac/Estimator/FundamentalEstimator.h"
#include "../Usac/Sampler/UniformSampler.h"

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename);
void storeResultsFundamental ();

void Tests::testFundamentalFitting() {
    std::string img_name = "graf";
    std::string points_filename = "../dataset/homography/"+img_name+"_pts.txt";
    std::vector<std::string> images_filename;
    images_filename.push_back("../dataset/homography/"+img_name+"A.png");
    images_filename.push_back("../dataset/homography/"+img_name+"B.png");

    cv::Mat points, points1, points2;
    read_points (points1, points2, points_filename);
    cv::hconcat(points1, points2, points);

    Model *fundamental_model = new Model (5, 7, 0.99, 0, "fundamental");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(fundamental_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);

    testFundamental (points, fundamental_model, uniform_sampler, images_filename, points_filename);

    Estimator *fundamental_estimator = new FundamentalEstimator (points);
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

//    runNTimes(points, fundamental_estimator, fundamental_model, uniform_sampler, termination_criteria, quality, 1000);

//    storeResultsFundamental ();
}

void testFundamental (cv::InputArray points, Model * const model, Sampler * const sampler, const std::vector<std::string>& images_filename, std::string points_filename) {
    Estimator * fundamental_estimator = new FundamentalEstimator (points);
    Drawing drawing;
    Logging logResult;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality * quality = new Quality;

    Ransac ransac (*model, *sampler, *termination_criteria, *quality);
    ransac.run(points, fundamental_estimator);

    RansacOutput *ransacOutput = ransac.getRansacOutput();

    std::cout << model->model_name << " time: ";
    ransacOutput->printTime();
    std::cout << model->model_name << " iterations: " << ransacOutput->getNumberOfIterations() << "\n";
    std::cout << model->model_name << " points under threshold: " << ransacOutput->getNumberOfInliers() << "\n";

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";

    cv::Mat pts = points.getMat();
    drawing.drawEpipolarLines(images_filename, pts.colRange(0,2), pts.colRange(2,4), model->returnDescriptor());
}

void storeResultsFundamental () {
    std::vector<std::string> points_filename = {"barrsmith_annot.txt", "barrsmith_pts.txt", "bonhall_pts.txt",
                                                "bonython_pts.txt", "elderhalla_pts.txt", "elderhallb_pts.txt",
                                                "hartley_pts.txt", "johnssona_pts.txt", "johnssonb_pts.txt",
                                                "ladysymon_pts.txt", "library_pts.txt", "napiera_pts.txt",
                                                "napierb_pts.txt", "neem_pts.txt", "unihouse_pts.txt",
                                                "oldclassicswing_pts.txt", "physics_pts.txt", "sene_pts.txt",
                                                "unionhouse_pts.txt"};

    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    for (std::string img_name : points_filename) {
        cv::Mat points1, points2;
        read_points(points1, points2, "../dataset/fundamental/" + img_name);
        cv::hconcat(points1, points2, points1);

        Estimator *fundamental_estimator = new FundamentalEstimator(points1);
        Sampler *uniform_sampler = new UniformSampler;
        Model *fundamental_model = new Model(3, 7, 0.99, 0, "fundamental");

        uniform_sampler->setSampleSize(fundamental_model->sample_number);
        uniform_sampler->setPointsSize(points1.rows);

        Ransac ransac(*fundamental_model, *uniform_sampler, *termination_criteria, *quality);
        ransac.run(points1, fundamental_estimator);

        RansacOutput *ransacOutput = ransac.getRansacOutput();

        cv::Mat F = ransacOutput->getModel()->returnDescriptor();

        std::ofstream save_model;
        std::string filename = "../results/fundamental/" + img_name.substr(0, img_name.find('.')) + "_Rmodel.txt";

        save_model.open(filename);

        save_model << F.at<float>(0, 0) << " " << F.at<float>(0, 1) << " " << F.at<float>(0, 2) << '\n'
                   << F.at<float>(1, 0) << " " << F.at<float>(1, 1) << " " << F.at<float>(1, 2) << '\n'
                   << F.at<float>(2, 0) << " " << F.at<float>(2, 1) << " " << F.at<float>(2, 2) << '\n';

        save_model << ransacOutput->getNumberOfInliers() << '\n';
        save_model << ransacOutput->getNumberOfIterations() << '\n';
        save_model << ransacOutput->getTimeMicroSeconds() << '\n';
        save_model.close();
    }
}