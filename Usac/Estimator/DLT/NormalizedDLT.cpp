#include "DLT.h"

bool NormalizedDLT (const float * const points, const int * const sample, unsigned int sample_number, cv::Mat &H) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(points, norm_points, sample, sample_number, T1, T2);

    // std::cout << "GetNormalizingTransformation finished \n";

    const float * const norm_points_ptr = (float *) norm_points.data;

    // solution not found
    if (! DLT(norm_points_ptr, sample_number, H)) {
        return false;
    }

    H = T2.inv()*H*T1;
    // normalize H by last h33
    H = H / H.at<float>(2,2);

    return true;
}

bool NormalizedDLT (const float * const points, const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &H) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(points, norm_points, sample, sample_number, weights, T1, T2);

//     std::cout << "GetNormalizingTransformation finished \n";

    const float * const norm_points_ptr = (float *) norm_points.data;

    // solution not found
    if (! DLT(norm_points_ptr, sample_number, H)) {
        return false;
    }

    H = T2.inv()*H*T1;
    // normalize H by last h33
    H = H / H.at<float>(2,2);

    return true;
}


bool NormalizedDLTLeastSquares (const float * const points, const int * const sample, unsigned int sample_number, cv::Mat &H) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(points, norm_points, sample, sample_number, T1, T2);

    // std::cout << "GetNormalizingTransformation finished \n";

    const float * const norm_points_ptr = (float *) norm_points.data;

    // solution not found
    if (! DLTLeastSquares(norm_points_ptr, sample_number, H)) {
        return false;
    }

    H = T2.inv()*H*T1;

    return true;
}
