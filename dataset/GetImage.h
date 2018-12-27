#ifndef USAC_GETIMAGE_H
#define USAC_GETIMAGE_H

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "Dataset.h"
#include "../Detector/ReadPoints.h"

class ImageData {
private:
    cv::Mat_<float> pts1, pts2, pts, model;
    cv::Mat img1, img2;
    std::vector<int> inliers;
public:
    ImageData (DATASET dataset, const std::string &img_name) {

        std::string folder;
        if (dataset == DATASET::Adelaidermf) {
            folder = "../dataset/adelaidermf/";
            read_points(pts1, pts2, folder+img_name+"_pts.txt");
            getInliers(folder+img_name+"_pts.txt", inliers);

        } else if (dataset == DATASET::Kusvod2) {
            folder = "../dataset/Lebeda/kusvod2/";
            getPointsNby6(folder+img_name+"_vpts_pts.txt", pts);
            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);
            getMatrix3x3(folder+img_name+"_model.txt", model);
            return;

        } else if (dataset == DATASET::Strecha) {
            folder = "../dataset/Lebeda/strechamvs/";
            getPointsNby6(folder+img_name+"_vpts_pts.txt", pts);
            img1 = cv::imread("../dataset/Lebeda/strechamvs/"+img_name+"A.jpg");
            img2 = cv::imread("../dataset/Lebeda/strechamvs/"+img_name+"B.jpg");
            if (img1.empty()) {
                img1 = cv::imread("../dataset/Lebeda/strechamvs/"+img_name+"A.png");
                img2 = cv::imread("../dataset/Lebeda/strechamvs/"+img_name+"B.png");
            }
            pts1 = pts.colRange(0,2);
            pts2 = pts.colRange(2,4);
            return;

        } else if (dataset == DATASET::Syntectic) {
            img1 = cv::imread("../dataset/line2d/"+img_name+".png");
            return;

        } else if (dataset == DATASET::EVD) {
            img1 = cv::imread("../dataset/EVD/1/"+img_name+".png");
            img2 = cv::imread("../dataset/EVD/2/"+img_name+".png");
            cv::Mat points;
            readEVDpoints(pts, "../dataset/EVD/EVD_tentatives/"+img_name+".png_m.txt");
            return;

        } else if (dataset == DATASET::Homogr) {
            folder = "../dataset/homography/";
            read_points(pts1, pts2, folder+img_name+"_pts.txt");
            getInliers(folder+img_name+"_pts.txt", inliers);

        } else {
            std::cout << "UNKNOWN DATASET!\n";
            exit (0);
        }

        cv::hconcat(pts1, pts2, pts);
        getMatrix3x3(folder+img_name+"_model.txt", model);

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

    std::vector<int> getGTInliers () {
        if (inliers.empty()) std::cout << "INLIERS ARE EMPTY!\n";
        return inliers;
    }

};

#endif //USAC_GETIMAGE_H
