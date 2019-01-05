#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Direct Linear Transformation
bool DLT (const float * const points, int sample_number, cv::Mat &H);
bool DLT (const float * const points, const int * const sample, int sample_number, cv::Mat &H);
bool DLTLeastSquares (const float * const points, int sample_number, cv::Mat &H);

bool NormalizedDLT (const float * const points, const int * const sample, unsigned int sample_number, cv::Mat &H);
bool NormalizedDLTLeastSquares (const float * const points, const int * const sample, unsigned int sample_number, cv::Mat &H);
bool NormalizedDLT (const float * const points, const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &H);

void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, cv::Mat &T1, cv::Mat &T2);

void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &T1, cv::Mat &T2);

#endif // RANSAC_DLT_H