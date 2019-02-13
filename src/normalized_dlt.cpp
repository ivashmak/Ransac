// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/dlt.hpp"

bool cv::usac::DLt::NormalizedDLT (const int * const sample, unsigned int sample_number, cv::Mat &H) {
    cv::Mat T1, T2, norm_points;
    cv::usac::GetNormalizingTransformation(points, norm_points, sample, sample_number, T1, T2);

    const float * const norm_points_ptr = (float *) norm_points.data;

    if (! DLT(norm_points_ptr, sample_number, H)) {
            return false;
    }

    H = T2.inv()*H*T1;
    // normalize H by last h33
    H = H / H.at<float>(2,2);

    return true;
}