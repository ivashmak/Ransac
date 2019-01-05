#include <iostream>
#include <opencv2/core/mat.hpp>
#include "SaveGTModel.h"
#include "../Detector/Reader.h"
#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Estimator/FundamentalEstimator.h"

void saveGTModel (DATASET dataset) {
    if (dataset != DATASET::Adelaidermf) {
        std::cout << "CURRENTLY NOT IMPLEMENTED\n";
        return;
    }

    float threshold = 2;
    std::string folder = "../dataset/adelaidermf/";
    std::vector<std::string> images = Dataset::getAdelaidermfDataset();
    for (std::string &img : images) {
        cv::Mat_<float> pts1, pts2;
        std::vector<int> inliers;
        Reader::read_points (pts1, pts2, folder+img+"_pts.txt");
        Reader::getInliers (folder+img+"_pts.txt", inliers);

        cv::hconcat(pts1, pts2, pts1);
        Estimator * estimator = new FundamentalEstimator (pts1);
        Model * model = new Model (threshold, 7, 0.95, 5, ESTIMATOR::Fundamental, SAMPLER::Uniform);
        estimator->EstimateModelNonMinimalSample(&inliers[0], inliers.size(), *model);
        std::ofstream save_model;
        save_model.open (folder + img + "_model.txt");
        cv::Mat F = model->returnDescriptor();
        save_model << F.at<float>(0,0) << " " << F.at<float>(0,1) << " " << F.at<float>(0,2) << "\n" <<
                      F.at<float>(1,0) << " " << F.at<float>(1,1) << " " << F.at<float>(1,2) << "\n" <<
                      F.at<float>(2,0) << " " << F.at<float>(2,1) << " " << F.at<float>(2,2) << "\n";
        save_model.close();
    }
}
