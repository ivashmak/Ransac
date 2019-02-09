#ifndef USAC_GETIMAGE_H
#define USAC_GETIMAGE_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "Dataset.h"
#include "../detector/Reader.h"
#include "../usac/estimator/homography_estimator.hpp"
#include "../usac/estimator/fundamental_estimator.hpp"
#include "../usac/quality/quality.hpp"
#include "../detector/detector.h"
#include "../usac/estimator/essential_estimator.hpp"
#include "../usac/utils/utils.hpp"
#include "../usac/estimator/line2d_estimator.hpp"

class ImageData {
private:
    cv::Mat_<float> pts1, pts2, pts, sorted_pts, model;
    cv::Mat img1, img2;
    std::vector<int> inliers, sorted_inliers;
    ESTIMATOR estimator = ESTIMATOR ::NullE;
public:
    ImageData (DATASET dataset, const std::string &img_name) {

        std::string folder;
        if (dataset == DATASET::Adelaidermf) {
            estimator = ESTIMATOR::Fundamental;

            folder = "../dataset/adelaidermf/";
            Reader::read_points(pts1, pts2, folder + img_name + "_pts.txt");
            Reader::getInliers(folder + img_name + "_pts.txt", inliers);
            Reader::getMatrix3x3(folder + img_name + "_model.txt", model);
            cv::hconcat(pts1, pts2, pts);
        } else if (dataset == DATASET::Adelaidermf_SIFT) {
            estimator = ESTIMATOR::Fundamental;
            folder = "../dataset/adelaidermf/";

            Reader::LoadPointsFromFile(pts, (folder+"sift_update/"+img_name+"_pts.txt").c_str());
            Reader::LoadPointsFromFile(sorted_pts, (folder+"sift_update/"+img_name+"_spts.txt").c_str());
            Reader::getMatrix3x3(folder + img_name + "_model.txt", model);

            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);

        } else if (dataset == DATASET::Kusvod2) {
            estimator = ESTIMATOR::Fundamental;

            folder = "../dataset/Lebeda/kusvod2/";
            Reader::getPointsNby6(folder + img_name + "_vpts_pts.txt", pts);
            Reader::getMatrix3x3(folder + img_name + "_vpts_model.txt", model);

            pts1 = pts.colRange(0, 2);
            pts2 = pts.colRange(2, 4);

        } else if (dataset == DATASET::Kusvod2_SIFT) {
            estimator = ESTIMATOR::Fundamental;
            folder = "../dataset/Lebeda/kusvod2/";

            Reader::LoadPointsFromFile(pts, (folder+"sift_update/"+img_name+"_pts.txt").c_str());
            Reader::LoadPointsFromFile(sorted_pts, (folder+"sift_update/"+img_name+"_spts.txt").c_str());
            Reader::getMatrix3x3(folder + img_name + "_vpts_model.txt", model);

            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);

        } else if (dataset == DATASET::Strecha) {
            estimator = ESTIMATOR ::Essential;

            folder = "../dataset/MVS/";
            Reader::LoadPointsFromFile(pts, (folder+img_name+"_pts.txt").c_str());
            Reader::LoadPointsFromFile(sorted_pts, (folder+img_name+"_spts.txt").c_str());
            Reader::readInliers (inliers, folder+img_name+"_inl.txt");
            EssentialEstimator essentialEstimator(pts);
            Model m (1, 0, 0, 0, ESTIMATOR::Essential, SAMPLER::Uniform);
            essentialEstimator.EstimateModelNonMinimalSample(&inliers[0], inliers.size(), m);
            model = m.returnDescriptor().clone();

            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);

        } else if (dataset == DATASET::Syntectic) {
            estimator = ESTIMATOR ::Line2d;

            folder = "../dataset/line2d/";
            img1 = cv::imread(folder+img_name+".png");
            std::ifstream read_data_file;
            read_data_file.open (folder+img_name+".txt");
            if (!read_data_file.is_open()) {
                std::cout << "Wrong directory for file of line2d dataset!\n";
                exit (0);
            }
            int width, height, noise, N;

            read_data_file >> width;
            read_data_file >> height;
            read_data_file >> noise;
            float a, b, c;
            read_data_file >> a;
            read_data_file >> b;
            read_data_file >> c;

            model = (cv::Mat_<float> (1,3) << a, b, c);

            read_data_file >> N;
            pts = cv::Mat_<float>(N, 2);
            float *pts_ptr = (float *) pts.data;
            float x, y;
            for (int p = 0; p < N; p++) {
                read_data_file >> x >> y;
                pts_ptr[2*p] = x;
                pts_ptr[2*p+1] = y;
            }

            densitySort (pts, 3, sorted_pts);

            return;

        } else if (dataset == DATASET::EVD) {
            estimator = ESTIMATOR::Homography;

            img1 = cv::imread("../dataset/EVD/1/" + img_name + ".png");
            img2 = cv::imread("../dataset/EVD/2/" + img_name + ".png");
            cv::Mat points;
            Reader::readEVDPointsInliers(pts, inliers, "../dataset/EVD/EVD_tentatives/" + img_name + ".png_m.txt");
            pts1 = pts.colRange(0, 2);
            pts2 = pts.colRange(2, 4);

            // points (inliers) are already sorted
            sorted_inliers = inliers;
            sorted_pts = pts.clone();
            Reader::getMatrix3x3("../dataset/EVD/h/" + img_name + ".txt", model);
            return;

        } else if (dataset == DATASET ::Homogr_SIFT) {
            estimator = ESTIMATOR ::Homography;

            folder = "../dataset/homography/";
            Reader::LoadPointsFromFile(pts, (folder+"sift_update/"+img_name+"_pts.txt").c_str());
            Reader::LoadPointsFromFile(sorted_pts, (folder+"sift_update/"+img_name+"_spts.txt").c_str());
            Reader::getMatrix3x3(folder+img_name+"_model.txt", model);
            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);

        } else if (dataset == DATASET::Homogr) {
            estimator = ESTIMATOR::Homography;

            folder = "../dataset/homography/";
            Reader::read_points(pts1, pts2, folder + img_name + "_pts.txt");
            Reader::getInliers(folder + img_name + "_pts.txt", inliers);
            Reader::getMatrix3x3(folder + img_name + "_model.txt", model);
            cv::hconcat(pts1, pts2, pts);

        } else {
            std::cout << "UNKNOWN DATASET!\n";
            exit (0);
        }

        img1 = cv::imread(folder+img_name+"A.png");
        img2 = cv::imread(folder+img_name+"B.png");
        if (img1.empty()) {
            img1 = cv::imread(folder+img_name+"A.jpg");
            img2 = cv::imread(folder+img_name+"B.jpg");
            if (img1.empty()) {
                std::cout << "WRONG IMAGE DIRECTORY\n";
                exit (0);
            }
        }
    }

    cv::Mat getPoints1 () {
        if (pts1.empty()) std::cout << "POINTS 1 ARE EMPTY!\n";
        return pts1;
    }

    cv::Mat getPoints2 () {
        if (pts2.empty()) std::cout << "POINTS 2 ARE EMPTY!\n";
        return pts2;
    }

    cv::Mat getPoints () {
        if (pts.empty()) std::cout << "POINTS ARE EMPTY!\n";
        return pts;
    }

    cv::Mat getSortedPoints () {
        if (sorted_pts.empty()) std::cout << "SORTED POINTS ARE EMPTY!\n";
        return sorted_pts;
    }

    cv::Mat getImage1 () {
        if (img1.empty()) std::cout << "IMAGE 1 IS EMPTY!\n";
        return img1;
    }

    cv::Mat getImage2 () {
        if (img2.empty()) std::cout << "IMAGE 2 IS EMPTY!\n";
        return img2;
    }

    cv::Mat getModel () {
        if (model.empty()) std::cout << "MODEL IS EMPTY!\n";
        return model;
    }

    std::vector<int> getGTInliers (float threshold) {
        if (inliers.empty()) {
            // if inliers are empty get them by model
            if (estimator == ESTIMATOR::Homography) {
                getGTInliersFromGTModelHomography (threshold, false);
            } else if (estimator == ESTIMATOR::Fundamental){
                getGTInliersFromGTModelFundamental (threshold, false);
            } else if (estimator == ESTIMATOR::Essential) {
                getGTInliersFromGTModelEssential (threshold, false);
            } else if (estimator == ESTIMATOR::Line2d) {
                getGTInliersFromGTModelLine2D (threshold, false);
            } else {
                std::cout << "unkown estimator in getGTinliers in GetImage\n";
                exit(111);
            }
        }
        return inliers;
    }

    std::vector<int> getGTInliersSorted (float threshold) {
        if (sorted_inliers.empty()) {
            // if inliers are empty get them by model
            if (estimator == ESTIMATOR::Homography) {
                getGTInliersFromGTModelHomography (threshold, true);
            } else if (estimator == ESTIMATOR::Fundamental) {
                getGTInliersFromGTModelFundamental (threshold, true);
            } else if (estimator == ESTIMATOR::Essential) {
                getGTInliersFromGTModelEssential (threshold, true);
            } else if (estimator == ESTIMATOR::Line2d) {
                getGTInliersFromGTModelLine2D (threshold, true);
            } else {
                std::cout << "unkown estimator in getGTinliersSorted in GetImage\n";
                exit(111);
            }
        }

        return sorted_inliers;
    }


    // get GT inliers using GT Model
    void getGTInliersFromGTModelHomography (float threshold, bool sorted) {
        Estimator * estimator;
        std::vector<int> inliers2;

        if (! sorted) {
            estimator = new HomographyEstimator(pts);

            Quality::getInliers(estimator, model, threshold, pts.rows, inliers);
            Quality::getInliers(estimator, model.inv(), threshold, pts.rows, inliers2);

            if (inliers2.size() > inliers.size()) {
                inliers = inliers2;
                cv::Mat temp = model.inv();
                model = temp.clone();
            }
        } else {
            estimator = new HomographyEstimator(sorted_pts);

            Quality::getInliers(estimator, model, threshold, sorted_pts.rows, sorted_inliers);
            Quality::getInliers(estimator, model.inv(), threshold, sorted_pts.rows, inliers2);

            if (inliers2.size() > sorted_inliers.size()) {
                sorted_inliers = inliers2;
                cv::Mat temp = model.inv();
                model = temp.clone();
            }
        }
    }

    void getGTInliersFromGTModelFundamental (float threshold, bool sorted) {
        Estimator * estimator;
        std::vector<int> inliers2;

        if (! sorted) {
            estimator = new FundamentalEstimator(pts);

            Quality::getInliers(estimator, model, threshold, pts.rows, inliers);
            Quality::getInliers(estimator, model.t(), threshold, pts.rows, inliers2);

            if (inliers2.size() > inliers.size()) {
                inliers = inliers2;
                cv::Mat temp = model.t();
                model = temp.clone();
            }
        } else {
            estimator = new FundamentalEstimator(sorted_pts);

            Quality::getInliers(estimator, model, threshold, sorted_pts.rows, sorted_inliers);
            Quality::getInliers(estimator, model.t(), threshold, sorted_pts.rows, inliers2);

            if (inliers2.size() > sorted_inliers.size()) {
                sorted_inliers = inliers2;
                cv::Mat temp = model.t();
                model = temp.clone();
            }
        }
    }

    void getGTInliersFromGTModelEssential (float threshold, bool sorted) {
        Estimator * estimator;

        if (sorted) {
            estimator = new EssentialEstimator(sorted_pts);
            Quality::getInliers(estimator, model, threshold, sorted_pts.rows, sorted_inliers);
        } else {
            estimator = new EssentialEstimator(pts);
            Quality::getInliers(estimator, model, threshold, pts.rows, inliers);
        }
    }

    void getGTInliersFromGTModelLine2D (float threshold, bool sorted) {
        Estimator * estimator;
        if (sorted) {
            estimator = new Line2DEstimator(sorted_pts);
            Quality::getInliers(estimator, model, threshold, sorted_pts.rows, sorted_inliers);
        } else {
            estimator = new Line2DEstimator(pts);
            Quality::getInliers(estimator, model, threshold, pts.rows, inliers);
        }

    }

};

#endif //USAC_GETIMAGE_H
