#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "ReadPoints.h"

void read_points (cv::Mat &pts1, cv::Mat &pts2) {
    std::fstream myfile("../points/graf_pts.txt", std::ios_base::in);

    float x1, y1, z1, x2, y2, z2, t;
    cv::Mat tmp = cv::Mat(1, 2, CV_32FC1);
    while (myfile >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> t) {
        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);
    }
}