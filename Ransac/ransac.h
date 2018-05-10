#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

struct Line{
	float x1;
	float y1;
	float x2;
	float y2;	
};


bool fit_point (float x, float y, float k, float b1, float b2);
Line ransac (std::vector<cv::KeyPoint> keypoints);
void draw_function (float k, float b, float max_dimen, cv::Scalar color, cv::Mat img);

#endif //RANSAC_RANSAC_H