// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include "tests.hpp"
#include "../include/opencv2/usac/math.hpp"
#include "../include/opencv2/usac/drawing.hpp"

void opencv_test::test (const cv::Mat &points,
                  cv::usac::Model * model,
                  const std::string &img_name1,
                  const std::string &img_name2,
                  bool gt,
                  const std::vector<int>& gt_inliers) {

    cv::Mat neighbors, neighbors_dists;

    std::vector<std::vector<int>> neighbors_v;

    long nn_time = 0;
    if (model->sampler == cv::usac::SAMPLER::Napsac || model->lo == cv::usac::LocOpt::GC) {
        // calculate time of nearest neighbor calculating
        auto begin_time = std::chrono::steady_clock::now();
        if (model->neighborsType == cv::usac::NeighborsSearch::Nanoflann) {
            cv::usac::NearestNeighbors::getNearestNeighbors_nanoflann(points, model->k_nearest_neighbors, neighbors, false, neighbors_dists);
        } else {
            cv::usac::NearestNeighbors::getGridNearestNeighbors(points, model->cell_size, neighbors_v);
        }
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = end_time - begin_time;
        nn_time = std::chrono::duration_cast<std::chrono::microseconds>(fs).count();
    }

    cv::usac::Ransac ransac (model, points);

    ransac.run();

    cv::usac::RansacOutput * ransacOutput = ransac.getRansacOutput();

    if (model->sampler == cv::usac::SAMPLER::Prosac) std::cout << "prosac\n";
    else if (model->sampler == cv::usac::SAMPLER::Uniform) std::cout <<  "uniform\n";
    else if (model->sampler == cv::usac::SAMPLER::Napsac) std::cout <<  "napsac\n";
    if (model->estimator == cv::usac::ESTIMATOR::Line2d) std::cout <<  "line2d\n";
    else if (model->estimator == cv::usac::ESTIMATOR::Homography) std::cout <<  "homography\n";
    else if (model->estimator == cv::usac::ESTIMATOR::Fundamental) std::cout <<  "fundamental\n";
    else if (model->estimator == cv::usac::ESTIMATOR::Essential) std::cout <<  "essential\n";
    if (model->lo == cv::usac::LocOpt::GC) std::cout << "Graph Cut LO\n";
    else if (model->lo == cv::usac::LocOpt::InItFLORsc) std::cout << "Locally Optimized\n";
    else if (model->lo == cv::usac::LocOpt::InItLORsc) std::cout << "Fixing Locally Optimized\n";
    if (model->sprt) std::cout << "Seq. Prob. test is on\n";

    std::cout << "\ttime: ";
    long time_mcs = ransacOutput->getTimeMicroSeconds();
    if (model->sampler == cv::usac::SAMPLER::Napsac || model->lo == cv::usac::LocOpt::GC) {
        time_mcs += nn_time;
    }
    cv::usac::Time time;
    cv::usac::splitTime(&time, time_mcs);
    std::cout << &time;
    std::cout << "\tMain iterations: " << ransacOutput->getNumberOfMainIterations() << "\n";
    std::cout << "\tLO iterations: " << ransacOutput->getNUmberOfLOIterarations() << "\n";
    std::cout << "\tpoints under threshold: " << ransacOutput->getNumberOfInliers() << "\n";

    std::cout << "Best model = ...\n" << ransacOutput->getModel ()->returnDescriptor() << "\n";

    if (gt) {
        cv::usac::Estimator * estimator;
        initEstimator(estimator, model->estimator, points);
        float error = cv::usac::Quality::getErrorToGTInliers(estimator, ransacOutput->getModel()->returnDescriptor(), gt_inliers);
        std::cout << "Ground Truth number of inliers for same model parametres is " << gt_inliers.size() << "\n";
        std::cout << "Error to GT inliers " << error << "\n";
        delete estimator;
    }
    std::cout << "-----------------------------------------------------------------------------------------\n";

    cv::usac::draw::draw(ransacOutput->getModel(), points, img_name1, img_name2);
//    delete ransacOutput;
}
