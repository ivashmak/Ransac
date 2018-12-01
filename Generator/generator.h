#ifndef RANSAC_GENERATOR_H
#define RANSAC_GENERATOR_H

#include <stdio.h>
#include <opencv2/opencv.hpp>

void generate (std::vector<cv::Point2f> &points_out, bool reset_time, bool getGT=false,  int * gt_inliers = 0);
void generate_syntectic_dataset ();

#endif //RANSAC_GENERATOR_H
