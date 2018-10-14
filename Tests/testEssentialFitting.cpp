#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <opencv2/core/types.hpp>

#include "../Detector/ReadPoints.h"

void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2);

void Tests::testEssentialFitting() {
    cv::Mat points1, points2;
    read_points (points1, points2, "../images/homograpy/graf_pts.txt");

    testEssentialMatrixEstimation(points1, points2);
}

void testEssentialMatrixEstimation (cv::InputArray points1, cv::InputArray points2) {

}