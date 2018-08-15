#ifndef RANSAC_DETECTOR_H
#define RANSAC_DETECTOR_H

#include <iostream>
#include <stdio.h>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

// std::vector<cv::KeyPoint> points = detect("data/image1.jpg", "sift");

std::vector<cv::KeyPoint> detect(std::string filename, std::string detector_name);

#endif //RANSAC_DETECTOR_H
