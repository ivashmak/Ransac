#include "tests.h"
#include "../helper/Logging.h"


void Tests::test (cv::Mat points,
                   Model * model,
                   const std::string &img_name,
                   DATASET dataset,
                   bool gt,
                   const std::vector<int>& gt_inliers) {

    cv::Mat neighbors, neighbors_dists;

//    std::cout << "get neighbors\n";
    std::vector<std::vector<int>> neighbors_v;

    long nn_time = 0;
    if (model->sampler == SAMPLER::Napsac || model->lo == LocOpt::GC) {
        // calculate time of nearest neighbor calculating
        auto begin_time = std::chrono::steady_clock::now();
        if (model->neighborsType == NeighborsSearch::Nanoflann) {
            NearestNeighbors::getNearestNeighbors_nanoflann(points, model->k_nearest_neighbors, neighbors, false, neighbors_dists);
        } else {
            NearestNeighbors::getGridNearestNeighbors(points, model->cell_size, neighbors_v);
            std::cout << "GOT neighbors\n";
        }
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> fs = end_time - begin_time;
        nn_time = std::chrono::duration_cast<std::chrono::microseconds>(fs).count();
    }

//    std::cout << "got neighbors\n";


    Drawing drawing;
    Logging logResult;

    Ransac ransac (model, points);

//    std::cout << "RUN ransac\n";
    ransac.run();

    RansacOutput * ransacOutput = ransac.getRansacOutput();

    std::cout << model->getName() << "\n";
    std::cout << "\ttime: ";
    long time_mcs = ransacOutput->getTimeMicroSeconds();
    if (model->sampler == SAMPLER::Napsac || model->lo == LocOpt::GC) {
        time_mcs += nn_time;
    }
    Time * time = new Time;
    splitTime(time, time_mcs);
    std::cout << time;
    std::cout << "\tMain iterations: " << ransacOutput->getNumberOfMainIterations() << "\n";
    std::cout << "\tLO iterations: " << ransacOutput->getLOIters() <<
    " (where " << ransacOutput->getLOInnerIters () << " (inner iters) and " <<
              ransacOutput->getLOIterativeIters() << " (iterative iters) and " << ransacOutput->getGCIters() << " (GC iters))\n";

    std::cout << "\tpoints under threshold: " << ransacOutput->getNumberOfInliers() << "\n";

    std::cout << "Best model = ...\n" << ransacOutput->getModel ()->returnDescriptor() << "\n";

    if (gt) {
        Estimator * estimator;
        initEstimator(estimator, model->estimator, points);
        float error = Quality::getErrorGT_inl(estimator, ransacOutput->getModel(), gt_inliers);
        std::cout << "Ground Truth number of inliers for same model parametres is " << gt_inliers.size() << "\n";
        std::cout << "Error to GT inliers " << error << "\n";
    }

    // save result and compare with last run
    logResult.compare(model, ransacOutput);
    logResult.saveResult(model, ransacOutput);
    std::cout << "-----------------------------------------------------------------------------------------\n";


    if (model->estimator == ESTIMATOR::Homography) {
        drawing.drawHomographies(img_name, dataset, points, ransacOutput->getInliers(), ransacOutput->getModel()->returnDescriptor());
    } else
    if (model->estimator == ESTIMATOR::Fundamental) {
        drawing.drawEpipolarLines(img_name, dataset, ransacOutput->getInliers(), points.colRange(0,2), points.colRange(2,4), ransacOutput->getModel()->returnDescriptor());
    } else
    if (model->estimator == ESTIMATOR::Line2d) {
        drawing.draw(ransacOutput->getInliers(), ransacOutput->getModel(), points, img_name+".png");
    } else
    if (model->estimator == ESTIMATOR::Essential) {
        drawing.drawEpipolarLines(img_name, dataset, ransacOutput->getInliers(), points.colRange(0,2), points.colRange(2,4), ransacOutput->getModel()->returnDescriptor());
    } else {
        std::cout << "UNKNOWN ESTIMATOR\n";
    }
}
