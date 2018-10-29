#ifndef USAC_UTILS_MATH_H
#define USAC_UTILS_MATH_H

#include <opencv2/core/mat.hpp>
#include <cassert>
#include <iostream>

bool inverse3x3 (cv::Mat& A);

/*
 * n^k, k >= 2
 */
float fast_pow (float n, int k);

//void fast_
#endif //USAC_UTILS_MATH_H
