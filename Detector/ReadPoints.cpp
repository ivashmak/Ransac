#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "ReadPoints.h"

void read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2, inl;
    cv::Mat tmp = cv::Mat_<float>(1, 2);
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);
    }
}

void getInliers (const std::string &filename, std::vector<int> &inliers) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2;
    int inl;
    int p = 0;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        if (inl) inliers.push_back(p);
        p++;
    }
}

void getH (const std::string &filename, cv::OutputArray H) {
    cv::Mat H_ = cv::Mat_<float>(3,3);
    std::fstream file(filename, std::ios_base::in);

    float val;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file >> val;
            H_.at<float>(i,j) = val;
        }
    }

    H_.copyTo (H);
}