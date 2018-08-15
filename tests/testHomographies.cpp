#include "Tests.h"

#include <cstdio>
#include <iostream>
#include <chrono>
#include <vector>
#include <opencv2/core/types.hpp>

#include "../Homographies/ReadPoints/ReadPoints.h"
#include "../Homographies/Estimator.h"
#include "../Homographies/HomographyEstimator.h"



void Tests::testHomographies() {
    cv::Mat points1, points2;
    read_points (points1, points2);

    Estimator *estimator = new HomographyEstimator;

    std::cout << "---------------- DLT ------------------------\n";

    cv::Mat H_DLT;
    estimator->DLT (points1, points2, H_DLT);
    std::cout << "H_DLT = \n" << H_DLT << "\n\n";

    std::cout << "---------------- GetNormalizingTransformation ------------------------\n";

    float s, s1, s2;
    cv::Mat T, offset;
    estimator->GetNormalizingTransformation(points1, T, offset, &s, &s1, &s2);
    std::cout << "offset =\n " << offset << "\n\n";
    std::cout << "T =\n " << T << "\n\n";
    std::cout << "s = " << s << "; s1 = " << s1 << "; s2 = " << s2 << '\n';

    std::cout << "---------------- NormalizedDLT ------------------------\n";

    cv::Mat H_NDLT;
    estimator->NormalizedDLT(points1, points2, H_NDLT);
    std::cout << "H_NDLT = \n" << H_NDLT << "\n\n";

}