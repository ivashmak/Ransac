#include <cstdlib>
#include "Tests.h"

#include "../Generator/generator.h"
#include "../Usac/Estimator/Line2DEstimator.h"
#include "../Usac/Ransac/Ransac.h"

#include "../Usac/Helper/Drawing/Drawing.h"
#include "../Usac/Helper/Logging.h"

#include "../Detector/ReadPoints.h"

#include "../Usac/Sampler/Sampler.h"
#include "../Usac/Sampler/NapsacSampler.h"
#include "../Usac/Sampler/GradualNapsacSampler.h"
#include "../Usac/Sampler/EvsacSampler.h"
#include "../Usac/Sampler/UniformSampler.h"
#include "../Usac/Sampler/ProsacSampler.h"
#include "../Usac/Utils/NearestNeighbors.h"


void testLine (cv::InputArray points, Sampler * sampler, Model * model);

void Tests::testLineFitting() {

    std::vector<cv::Point_<float>> points;

    // change false to true to reset time for random points generator
    generate(points, false);
    std::cout << "generated points\n";


    // sort points for Prosac
    cv::Mat indicies, dists1, dists2, p (points);
    int knn = 2;
    cv::flann::LinearIndexParams flannIndexParams;
    cv::flann::Index * flannIndex = new cv::flann::Index (p.reshape(1), flannIndexParams);
    std::vector<cv::Point_<float>> sorted_points (points);

    /*
     * Prosac quality sort.
     * Sorting by distance of first nearest neighbor.
     */
    std::sort(sorted_points.begin(), sorted_points.end(), [&] (const cv::Point_<float>& a, const cv::Point_<float>& b) {
        flannIndex->knnSearch(cv::Mat_<float>(a), indicies, dists1, knn);
        flannIndex->knnSearch(cv::Mat_<float>(b), indicies, dists2, knn);
        return dists1.at<float>(1) < dists2.at<float>(1);
    });
    //---
    bool LO = false;

     Model *ransac_model = new Model (10, 2, 0.99, 0, ESTIMATOR::Line2d, SAMPLER::Uniform);
     Sampler *uniform_sampler = new UniformSampler;
     uniform_sampler->setSampleSize(ransac_model->sample_number);
     uniform_sampler->setPointsSize(points.size());
     uniform_sampler->initRandomGenerator();

//    Model *gradual_napsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::GradualNapsac);
//    Sampler *gradual_napsac_sampler = new GradualNapsacSampler(points, gradual_napsac_model->sample_number);
//
//    knn = 7;
//    Model *napsac_model = new Model (10, 2, 0.99, knn, ESTIMATOR::Line2d, SAMPLER::Napsac);
//    cv::Mat neighbors;
//    NearestNeighbors nn;
//    nn.getNearestNeighbors_flann(p, knn+1, neighbors);
//    Sampler *napsac_sampler = new NapsacSampler(neighbors, napsac_model->k_nearest_neighbors, napsac_model->sample_number);

//    Model *evsac_model = new Model (10, 2, 0.99, 7, ESTIMATOR::Line2d, SAMPLER::Evsac);
//    Sampler *evsac_sampler = new EvsacSampler(points, points.size(), evsac_model->k_nearest_neighbors, evsac_model->sample_number);
//
//    Model *prosac_model = new Model (10, 2, 0.99, 0, ESTIMATOR::Line2d, SAMPLER::Prosac);
//    Sampler *prosac_sampler = new ProsacSampler(prosac_model->sample_number, points.size());

     testLine (points, uniform_sampler, ransac_model);
//     testLine (points, gradual_napsac_sampler, gradual_napsac_model);
//     testLine (points, napsac_sampler, napsac_model);
    // testLine (points, evsac_sampler, evsac_model);
    // testLine (sorted_points, prosac_sampler, prosac_model);

//    Estimator *line2destimator = new Line2DEstimator (points);
//    TerminationCriteria *termination_criteria = new TerminationCriteria;
//    Quality *quality = new Quality;
//
//    getAverageResults(points, line2destimator, ransac_model, uniform_sampler, termination_criteria, quality, 2000, LO);
}

/*
 * test line fitting estimation for one data image.
 */
void testLine (cv::InputArray points, Sampler * const sampler, Model * const model) {
    Estimator *estimator2d = new Line2DEstimator (points);
    Drawing drawing;
    Logging logResult;
    TerminationCriteria *termination_criteria = new TerminationCriteria;
    Quality *quality = new Quality;

    Ransac ransac (*model, *sampler, *termination_criteria, *quality, *estimator2d);
    ransac.run(points);

    RansacOutput *ransacOutput = ransac.getRansacOutput();

    std::cout << model->getName() << "\n";
    std::cout << "\ttime: ";
    ransacOutput->printTime();
    std::cout <<"\titerations: " << ransacOutput->getNumberOfIterations() <<
              " (" << ((int)ransacOutput->getNumberOfIterations () -(int)ransacOutput->getNumberOfLOIterations ()) << 
              " + " << ransacOutput->getNumberOfLOIterations () << " (" << ransacOutput->getLORuns() << " lo inner + iterative runs)) \n";
    
    std::cout <<"\tpoints under threshold: " << ransacOutput->getNumberOfInliers() << "\n";
    std::cout << "\tAverage error " << ransacOutput->getAverageError() << "\n";

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";

    //    drawing.draw(ransac.most_inliers, ransac.getBestModel(), ransac.getNonMinimalModel(), points);
    drawing.draw(ransacOutput->getInliers(), ransacOutput->getModel(), points);
}


