#ifndef RANSAC_NDLT_H
#define RANSAC_NDLT_H

#include <opencv2/core.hpp>
#include "GetNormalizingTransformation.h"
#include "DLT.h"

//void NormalizedDLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
void NormalizedDLT (cv::InputArray pts, const int * const sample, int sample_number, cv::Mat &H) {

//    cv::Mat points1 = pts1.getMat();
//    cv::Mat points2 = pts2.getMat();


    float * const points = (float *) pts.getMat().data;
    float *points1_ptr = new float[2*sample_number];
    float *points2_ptr = new float[2*sample_number];

    for (int i = 0; i < sample_number; i++) {
        points1_ptr[2*i] = points[4*sample[i]];
        points1_ptr[2*i+1] = points[4*sample[i]+1];
        points2_ptr[2*i] = points[4*sample[i]+2];
        points2_ptr[2*i+1] = points[4*sample[i]+3];
    }

    cv::Mat_<float> points1 (sample_number, 2, points1_ptr);
    cv::Mat_<float> points2 (sample_number, 2, points2_ptr);

    int NUMP = sample_number; //points1.rows;

    cv::Mat T1, T2, offset1, offset2;
    // float s1, s2, s1_1, s1_2, s2_1, s2_2;
     float tmp;

    GetNormalizingTransformation(points1, T1, offset1, &tmp, &tmp, &tmp);
    GetNormalizingTransformation(points2, T2, offset2, &tmp, &tmp, &tmp);

    cv::Mat ones = cv::Mat_<float>::ones(1, NUMP);

    // transpose to multiply T (size 3x3) by points size 3 x N
    cv::transpose (points1, points1);
    cv::transpose (points2, points2);

    // make points size from 2 x N to 3 x N
    cv::vconcat(points1, ones, points1);
    cv::vconcat(points2, ones, points2);

    points1 = T1 * points1; // pts1Tr
    points2 = T2 * points2; // pts2Tr

//    DLT(points1.rowRange(0,2), points2.rowRange(0,2), H);

    int * newsample = new int [sample_number];
    for (int i = 0; i < sample_number; i++) {
        newsample[i] = i;
    }
    cv::Mat newpts;
    cv::vconcat(points1.rowRange(0,2), points2.rowRange(0,2), newpts);

    cv::transpose(newpts, newpts);

//    std::cout << newpts << "\n\n";

    DLT(newpts, newsample, sample_number, H);

    H = T2.inv()*H*T1;
}

#endif // RANSAC_NDLT_H