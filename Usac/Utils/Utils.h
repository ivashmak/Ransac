
#ifndef USAC_UTILS_H
#define USAC_UTILS_H

#include <opencv2/core/mat.hpp>

void densitySort (const cv::Mat &points, int max_neighbor, cv::Mat &sorted_points);

#endif //USAC_UTILS_H
