#ifndef RANSAC_GENERATOR_H
#define RANSAC_GENERATOR_H

#include <stdio.h>
#include <opencv2/opencv.hpp>

void generate(std::vector<cv::Point2f> &points, bool reset_time);

#endif //RANSAC_GENERATOR_H
