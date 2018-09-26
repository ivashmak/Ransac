#ifndef RANSAC_NDLT_H
#define RANSAC_NDLT_H

#include <opencv2/core.hpp>
#include "GetNormalizingTransformation.h"
#include "DLT.h"

void NormalizedDLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
//    assert(!pts1.empty());
//    assert(!pts2.empty());

    cv::Mat points1 = pts1.getMat();
    cv::Mat points2 = pts2.getMat();

    int NUMP = points1.rows;

    cv::Mat T1, T2, offset1, offset2;
    float s1, s2, s1_1, s1_2, s2_1, s2_2;

    GetNormalizingTransformation(pts1, T1, offset1, &s1, &s1_1, &s1_2);
    GetNormalizingTransformation(pts2, T2, offset2, &s2, &s2_1, &s2_2);

    cv::Mat ones = cv::Mat_<float>::ones(1, NUMP), points1_3d, points2_3d;

    cv::transpose (points1, points1);
    cv::transpose (points2, points2);

    cv::vconcat(points1, ones, points1_3d);
    cv::vconcat(points2, ones, points2_3d);

    cv::Mat pts1Tr = T1 * points1_3d;
    cv::Mat pts2Tr = T2 * points2_3d;

    pts1Tr.rowRange(0,2).copyTo(pts1Tr);
    pts2Tr.rowRange(0,2).copyTo(pts2Tr);

    DLT(pts1Tr, pts2Tr, H);

    H = T2.inv()*H*T1;
}

#endif // RANSAC_NDLT_H