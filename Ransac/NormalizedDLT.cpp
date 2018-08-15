#include <opencv2/core.hpp>
#include "HomographyMethods.h"

void HomographyMethods::NormalizedDLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
    CV_Assert(!pts1.empty());
    CV_Assert(!pts2.empty());

    cv::Mat points1 = pts1.getMat();
    cv::Mat points2 = pts2.getMat();

    int NUMP = points1.rows;

    cv::Mat T1, T2, offset1, offset2;
    float s1, s2, s1_1, s1_2, s2_1, s2_2;

    GetNormalizingTransformation(pts1, T1, offset1, &s1, &s1_1, &s1_2);
    GetNormalizingTransformation(pts2, T2, offset2, &s2, &s2_1, &s2_2);

    cv::Mat ones = cv::Mat::ones(1, NUMP, CV_32FC1), tmp1, tmp2;

    cv::transpose (points1, points1);
    cv::transpose (points2, points2);

    cv::vconcat(points1, ones, tmp1);
    cv::vconcat(points2, ones, tmp2);

    cv::Mat pts1Tr = T1 * tmp1; cv::transpose(pts1Tr, pts1Tr);
    cv::Mat pts2Tr = T2 * tmp2; cv::transpose(pts2Tr, pts2Tr);

    pts1Tr.colRange(0,2).copyTo(pts1Tr);
    pts2Tr.colRange(0,2).copyTo(pts2Tr);

    DLT(pts1Tr, pts2Tr, H);
    H = T2.inv()*H*T1;
}