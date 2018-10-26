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
void testHomography (cv::InputArray points, Model * const model, Sampler * const sampler, std::vector<std::string> images_filename, std::string points_filename);
void storeResults ();

void Tests::testHomographyFitting() {
    std::string img_name = "boat";
    std::string points_filename = "../dataset/homography/"+img_name+"_pts.txt";
    std::vector<std::string> images_filename;
    images_filename.push_back("../dataset/homography/"+img_name+"A.png");
    images_filename.push_back("../dataset/homography/"+img_name+"B.png");

    cv::Mat points, points1, points2;
    read_points (points1, points2, points_filename);

//    points1 = (cv::Mat_<float> (10, 2) << 0.616188105841203, 0.511878887258433,
//              -1.235108941720951,   0.298181899685778,
//              -0.602445781444621, 0.395316341521395,
//               0.638537560775656  , 0.476360331650075,
//               -0.646735338229623 ,  0.404851073762976,
//               -0.359599737840777,   0.408160300564896,
//                0.496360069223658  , 0.485702841945883,
//                0.445172420567822  , 0.455828982746341,
//                1.138768685240342  , 0.573991962861691,
//                1.522641846151249  , 0.657347408738955);
//
//    points2 = (cv::Mat_<float> (10, 2) << 5.239541394377807,   4.952122087513180,
//    2.409419451041671,   5.424131589524132,
//    3.252365668983476,   5.382115580979669,
//    5.294847441491644,   4.723592364956511,
//    3.236358303018495,   5.440276232429847,
//    3.545647126029825,   5.273469089815864,
//    4.987517232629965,   4.928677226681248,
//    4.868698783886824,   4.814382625168967,
//    6.474987110484027,   4.756703530439805,
//    7.390912908272599,   4.837087993931045);
//
//    points1 = 1000 * points1;
//    points2 = 100 * points2;

    cv::hconcat(points1, points2, points);

//    std::cout << cv::findHomography(points1, points2) << '\n';

    bool LO = false;

    Model *homography_model = new Model (3, 4, 0.99, 0, "homography");
    Sampler *uniform_sampler = new UniformSampler;
    uniform_sampler->setSampleSize(homography_model->sample_number);
    uniform_sampler->setPointsSize(points1.rows);
    uniform_sampler->initRandomGenerator();

//   testHomography (points, homography_model, uniform_sampler, images_filename, points_filename);

    Estimator * homograpy_estimator = new HomographyEstimator (points);
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

//    runNTimes(points, homograpy_estimator, homography_model, uniform_sampler, termination_criteria, quality, 1000, LO);

     storeResults();
}

void testHomography (cv::InputArray points, Model * const model, Sampler * const sampler, std::vector<std::string> images_filename, std::string points_filename) {
    Estimator * homograpy_estimator = new HomographyEstimator (points);
    Drawing drawing;
    Logging logResult;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    Ransac ransac (*model, *sampler, *termination_criteria, *quality, *homograpy_estimator);
    ransac.run(points);

    RansacOutput * ransacOutput = ransac.getRansacOutput();

    std::cout << model->model_name << " time: ";
    ransacOutput->printTime();
    std::cout << model->model_name << " iterations: " << ransacOutput->getNumberOfIterations() <<
              " (" << ((int)ransacOutput->getNumberOfIterations () -(int)ransacOutput->getNumberOfLOIterations ()) << 
              " + " << ransacOutput->getNumberOfLOIterations () << " (" << ransacOutput->getLORuns() << " lo inner + iterative runs)) \n";
    
    std::cout << model->model_name << " points under threshold: " << ransacOutput->getNumberOfInliers() << "\n";
    std::cout << ransacOutput->getModel()->returnDescriptor() << '\n';
    cv::Mat H = ransacOutput->getModel()->returnDescriptor();

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";

    drawing.drawHomographies(images_filename, points_filename, ransacOutput->getInliers(), ransacOutput->getModel()->returnDescriptor());
}

void storeResults () {
    std::vector<std::string> points_filename = getHomographyDatasetPoints();

    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;
    Model *homography_model = new Model (3, 4, 0.99, 0, "homography");
    bool LO = false;

    std::ofstream results_total;
    results_total.open ("../results/homography/ALL.csv");
    results_total << "LO " << LO << '\n';
    results_total << "Filename,Number of Inliers (found / total points),Number of Iterations,Time (mcs),"
                     "Average Error (threshold = " << homography_model->threshold <<"),,,,,\n";

    for (std::string img_name : points_filename) {
        cv::Mat points1, points2;
        read_points (points1, points2, "../dataset/homography/"+img_name);
        cv::hconcat(points1, points2, points1);

        Estimator * homograpy_estimator = new HomographyEstimator (points1);
        Sampler *uniform_sampler = new UniformSampler;

        uniform_sampler->setSampleSize(homography_model->sample_number);
        uniform_sampler->setPointsSize(points1.rows);
        uniform_sampler->initRandomGenerator();

        Ransac ransac (*homography_model, *uniform_sampler, *termination_criteria, *quality, *homograpy_estimator);
        ransac.run(points1);

        RansacOutput *ransacOutput = ransac.getRansacOutput();

        cv::Mat H = ransacOutput->getModel()->returnDescriptor();

        std::ofstream save_model;
        std::string filename = "../results/homography/" + img_name.substr(0, img_name.find('_')) +".csv";
        save_model.open(filename);

        save_model << H.at<float>(0,0) << "," << H.at<float>(0,1) << "," << H.at<float>(0,2) << ",\n"
                   << H.at<float>(1,0) << "," << H.at<float>(1,1) << "," << H.at<float>(1,2) << ",\n"
                   << H.at<float>(2,0) << "," << H.at<float>(2,1) << "," << H.at<float>(2,2) << ",\n";

        save_model << ransacOutput->getNumberOfInliers() << '\n';
        save_model << ransacOutput->getNumberOfIterations() << '\n';
        save_model << ransacOutput->getTimeMicroSeconds() << '\n';

        results_total << img_name << ",";
        results_total << ransacOutput->getNumberOfInliers() << "/" << points1.rows << ",";
        results_total << ransacOutput->getNumberOfIterations() << ",";
        results_total << ransacOutput->getTimeMicroSeconds() << ",";
        results_total << quality->getAverageError(homograpy_estimator, ransacOutput->getModel(), points1, points1.rows) << "\n";
        save_model.close();
    }

    results_total.close();
}