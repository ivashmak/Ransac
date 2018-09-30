#ifndef RANSAC_NDLT_H
#define RANSAC_NDLT_H

#include <opencv2/core.hpp>
#include "GetNormalizingTransformation.h"
#include "DLT.h"

void NormalizedDLT (const float * const points, const int * const sample, int sample_number, cv::Mat &H) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(points, norm_points, sample, sample_number, T1, T2);

    const float * const newpoints = (float *) norm_points.data;
    DLT(newpoints, sample_number, H);

    H = T2.inv()*H*T1;
}

#endif // RANSAC_NDLT_H