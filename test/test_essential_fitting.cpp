#include "tests.h"



void detectFeaturesMVS () {
    std::vector<std::string> mvs = Dataset::getStrechaDataset();
    cv::Mat P1, P2;
    float threshold = 1;
    std::string folder = "../dataset/MVS/";
    for (const std::string &filename : mvs) {
        Reader::readProjectionMatrix(P1, folder+filename+"A.P");
        Reader::readProjectionMatrix(P2, folder+filename+"B.P");

        detectFeaturesForEssentialMatrix(P1, P2, folder+filename, "png", threshold);
    }
}

void Tests::testEssentialFitting() {
//    detectFeaturesMVS();
//    exit (0);

    DATASET dataset = DATASET::Strecha;
    std::string img_name = "fountain_dense_1";

    float threshold = 2;
    float confidence = 0.95;
    int knn = 6;

    ImageData img_data (dataset, img_name);
    cv::Mat points = img_data.getPoints();
    cv::Mat sorted_points = img_data.getPoints();
    std::vector<int> gt_inliers = img_data.getGTInliers(threshold);
    std::vector<int> gt_sorted_inliers = img_data.getGTInliersSorted(threshold);

    Model * model;

    // -------------------------- uniform -------------------------------------
    model = new Model (threshold, 5, confidence, knn, ESTIMATOR::Essential, SAMPLER::Uniform);
    // ------------------------------------------------------------------------

    // -------------------------- Prosac -------------------------------------
//    model = new Model (threshold, 7, confidence, knn, ESTIMATOR::Fundamental, SAMPLER::Prosac);
    // ------------------------------------------------------------------------

    model->lo = LocOpt ::NullLO;
    model->setSprt(0);
    model->setCellSize(50);
    model->setNeighborsType(NeighborsSearch::Grid);
    model->ResetRandomGenerator(true);

    test (points, model, img_name, dataset, true, gt_inliers);

}
