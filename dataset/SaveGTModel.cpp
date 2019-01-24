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

void saveGTModelKusvod2 () {
    float threshold = 2;
    std::string folder =  "../dataset/Lebeda/kusvod2/";
    std::vector<std::string> images = Dataset::getKusvod2Dataset();
    for (std::string &img : images) {

        std::ifstream orig_model;
        orig_model.open (folder + img + "_vpts_model.txt");
        float f11 = 0, f12 = 0, f13 = 0, f21 = 0, f22 = 0, f23 = 0, f31 = 0, f32 = 0, f33 = 0;
        orig_model >> f11; orig_model >> f12; orig_model >> f13;
        orig_model >> f21; orig_model >> f22; orig_model >> f23;
        orig_model >> f31; orig_model >> f32; orig_model >> f33;
        if (f11 != 0 || f12 != 0 || f13 != 0 || f21 != 0 || f22 != 0 || f23 != 0 || f31 != 0 || f32 != 0 || f33 != 0) {
            // continue if gt model is valid
            std::cout << "continue\n";
            continue;
        }

        cv::Mat_<float> pts;
        Reader::getPointsNby6 (folder+img+"_vpts_pts.txt", pts);
        Estimator * estimator = new FundamentalEstimator (pts);
        Model * model = new Model (threshold, 7, 0.95, 5, ESTIMATOR::Fundamental, SAMPLER::Uniform);
        std::vector<int> inliers;
        for (int i = 0; i < pts.rows; i++) {
            inliers.push_back(i);
        }
        estimator->EstimateModelNonMinimalSample(&inliers[0], inliers.size(), *model);




        std::ofstream save_model;
        save_model.open (folder + img + "_vpts_model.txt");
        cv::Mat F = model->returnDescriptor();
        save_model << F.at<float>(0,0) << " " << F.at<float>(0,1) << " " << F.at<float>(0,2) << "\n" <<
                   F.at<float>(1,0) << " " << F.at<float>(1,1) << " " << F.at<float>(1,2) << "\n" <<
                   F.at<float>(2,0) << " " << F.at<float>(2,1) << " " << F.at<float>(2,2) << "\n";
        save_model.close();
    }
}
