#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"
#include "../Usac/Estimator/Estimator.h"
#include "../Usac/Estimator/HomographyEstimator.h"
#include "../Usac/EssentialMatrixEstimation.h"

#include "../Usac/Homographies/DLT.h"
#include "../Usac/Homographies/GetNormalizingTransformation.h"
#include "../Usac/Homographies/NormalizedDLT.h"
#include "../Usac/FundamentalMatrixEstimation.h"

void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2);
void testFundamentalMatrixEstimation (cv::InputArray points1, cv::InputArray points2);
void testDLT (cv::InputArray points1, cv::InputArray points2);
void testGetNormalizingTransformation (cv::InputArray points);
void testNormalizedDLT (cv::InputArray points1, cv::InputArray points2);

void Tests::testHomographyFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2);

    testEssentialMatrixEstimation(points1, points2);
//    testFundamentalMatrixEstimation(points1, points2);

//    testDLT(points1, points2);
//    testGetNormalizingTransformation(points1);
//    testGetNormalizingTransformation(points2);
//    testNormalizedDLT(points1, points2);
}


void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2) {
    EssentialMatrixEstimation * ess_mat_est = new EssentialMatrixEstimation;
    cv::Mat E;
    ess_mat_est->fivePointsAlg(points1, points2, E);

    std::cout << "Essential Matrix =\n " << E << "\n\n";

    cv::Mat R1, R2, T;
    cv::decomposeEssentialMat(E, R1, R2, T);

    std::cout << "R1 = \n" << R1 << "\n\nR2 = \n" << R2 << "\n\nT = \n" << T << "\n\n";
}

void testFundamentalMatrixEstimation (cv::InputArray points1, cv::InputArray points2) {
    FundamentalMatrixEstimation * fun_mat_est = new FundamentalMatrixEstimation;
    cv::Mat F, F_opencv;

    fun_mat_est->sevenPointsAlg(points1, points2, F);
    fun_mat_est->sevenPointsOpencv(points1, points2, F_opencv);

    std::cout << "Fundamental Matrix =\n " << F << "\n\n";
    std::cout << "Fundamental Matrix OpenCV =\n " << F_opencv << "\n\n";

    cv::Mat img1 = cv::imread("../images/img1.png"), img2 = cv::imread("../images/img2.png");

    cv::Mat pts1 = points1.getMat(), pts2 = points2.getMat();
    cv::hconcat(pts1, cv::Mat_<float>::ones (pts1.rows, 1), pts1);
    cv::hconcat(pts2, cv::Mat_<float>::ones (pts2.rows, 1), pts2);

    cv::Mat lines1, lines2;
    cv::computeCorrespondEpilines(pts1, 1, F, lines1);
    cv::computeCorrespondEpilines(pts2, 2, F, lines2);

    int c = img1.cols, r = img1.rows;
    float x0, y0_img1, x1, y1_img1, r0_img1, r1_img1, r2_img1;
    float  y0_img2, y1_img2, r0_img2, r1_img2, r2_img2;
    for (int i = 0; i < points1.rows(); i++) {
        r0_img1 = lines1.at<float>(i, 0);
        r1_img1 = lines1.at<float>(i, 1);
        r2_img1 = lines1.at<float>(i, 2);

        r0_img2 = lines2.at<float>(i, 0);
        r1_img2 = lines2.at<float>(i, 1);
        r2_img2 = lines2.at<float>(i, 2);

        x0 = 0;
        y0_img1 = -r2_img1/r1_img1;
        y0_img2 = -r2_img2/r1_img2;
        x1 = c;
        y1_img1 = -(r2_img1 + r0_img1*c)/r1_img1;
        y1_img2 = -(r2_img2 + r0_img2*c)/r1_img2;

        cv::Scalar color = cv::Scalar(rand()%255, rand ()%255, rand()%255);
        cv::line(img1, cv::Point_<float> (x0, y0_img1), cv::Point_<float> (x1, y1_img1), color);
        cv::line(img2, cv::Point_<float> (x0, y0_img2), cv::Point_<float> (x1, y1_img2), color);
        circle (img1, cv::Point_<float> (pts1.at<float>(i, 0), pts1.at<float>(i,1)), 3, color, -1);
        circle (img2, cv::Point_<float> (pts2.at<float>(i, 0), pts2.at<float>(i,1)), 3, color, -1);
    }

    imshow("Epipolar lines using Fundamental matrix 1", img1);
    imshow("Epipolar lines using Fundamental matrix 2", img2);
    cv::waitKey (0);

}


void testDLT (cv::InputArray points1, cv::InputArray points2) {
    std::cout << "---------------- DLT ------------------------\n";

    cv::Mat H_DLT;
    DLT (points1, points2, H_DLT);
    std::cout << "H_DLT = \n" << H_DLT << "\n\n";

}

void testGetNormalizingTransformation (cv::InputArray points) {
    std::cout << "---------------- GetNormalizingTransformation ------------------------\n";

    float s, s1, s2;
    cv::Mat T, offset;
    GetNormalizingTransformation(points, T, offset, &s, &s1, &s2);
    std::cout << "offset =\n " << offset << "\n\n";
    std::cout << "T =\n " << T << "\n\n";
    std::cout << "s = " << s << "; s1 = " << s1 << "; s2 = " << s2 << '\n';

}

void testNormalizedDLT (cv::InputArray points1, cv::InputArray points2) {
    std::cout << "---------------- NormalizedDLT ------------------------\n";
    cv::Mat H_NDLT;
    NormalizedDLT(points1, points2, H_NDLT);
    std::cout << "H_NDLT = \n" << H_NDLT << "\n\n";
}