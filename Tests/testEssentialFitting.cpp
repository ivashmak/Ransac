#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"
#include "../Usac/EssentialMatrixEstimation.h"

void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2);

void Tests::testEssentialFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2);

    testEssentialMatrixEstimation(points1, points2);
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