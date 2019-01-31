#ifndef USAC_FUNDEMANTALSOLVER_H
#define USAC_FUNDEMANTALSOLVER_H

#include <opencv2/core/mat.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

bool EightPointsAlgorithm (const float * const pts, const int * const sample, unsigned int sample_number, cv::Mat &F);
bool EightPointsAlgorithmEigen (const float * const pts, const int * const sample, unsigned int sample_number, cv::Mat &F);
bool EightPointsAlgorithm (const float * const pts, const int * const sample, const float * const weights, unsigned int sample_number, cv::Mat &F);
unsigned int SevenPointsAlgorithm (const float * const pts, const int * const sample, cv::Mat &F);

#endif //USAC_FUNDEMANTALSOLVER_H
