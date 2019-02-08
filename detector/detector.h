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

std::vector<cv::KeyPoint> detect(std::string filename, std::string detector_name);
void DetectFeatures(const std::string &name, const cv::Mat &image1, const cv::Mat &image2, cv::Mat &points);
void detectFeaturesForEssentialMatrix (const cv::Mat &P1, const cv::Mat &P2,
                                       const std::string &img_path_name,
                                       const std::string &img_ext,
                                       float threshold,
                                       cv::Mat &E_gt, cv::Mat &points, std::vector<int> &gt_inliers);

#endif //RANSAC_DETECTOR_H
