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

void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2);
void testDLT (cv::InputArray points1, cv::InputArray points2);
void testGetNormalizingTransformation (cv::InputArray points);
void testNormalizedDLT (cv::InputArray points1, cv::InputArray points2);

void Tests::testHomographyFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2);

//    testEssentialMatrixEstimation(points1, points2);

//    testDLT(points1, points2);
//    testGetNormalizingTransformation(points1);
//    testGetNormalizingTransformation(points2);
//    testNormalizedDLT(points1, points2);
}


void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2) {
    EssentialMatrixEstimation * ess_mat_est = new EssentialMatrixEstimation;
    cv::Mat ess_mat;
    ess_mat_est->fivePointsAlg(points1, points2, ess_mat);

    std::cout << "Essential Matrix =\n " << ess_mat << "\n\n";
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