#include "DLT.h"

bool NormalizedDLT (const float * const points, const int * const sample, int sample_number, cv::Mat &H) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(points, norm_points, sample, sample_number, T1, T2);

    // std::cout << "GetNormalizingTransformation finished \n";

    const float * const newpoints = (float *) norm_points.data;

    // solution not found
    if (DLT(newpoints, sample_number, H) == false) {
        return false;
    }

    H = T2.inv()*H*T1;

    return true;
}
