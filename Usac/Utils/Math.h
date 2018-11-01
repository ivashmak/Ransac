#ifndef USAC_UTILS_MATH_H
#define USAC_UTILS_MATH_H

#include <opencv2/core/mat.hpp>
#include <cassert>
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

bool inverse3x3 (cv::Mat& A);
bool inverse3x3 (const cv::Mat& A, cv::Mat& A_inv);

void testInv ();

float fast_pow (float n, int k);

int fast_factorial (int n);

//void fast_
#endif //USAC_UTILS_MATH_H
