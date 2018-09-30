#ifndef USAC_EIGHTPOINTSALGORITHM_H
#define USAC_EIGHTPOINTSALGORITHM_H

#include <opencv2/core/mat.hpp>

void EightPointsAlgorithm (const float * const pts, const int * const sample, int sample_number, cv::OutputArray F_out) {
    // form a linear system: i-th row of A(=a) represents
    // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
    int smpl;
    float x1, y1, x2, y2;
    cv::Mat_<float> A(sample_number, 9);
    float *A_ptr = (float *) A.data;

    for (int i = 0; i < sample_number; i++) {
        smpl = 4*sample[i];
        x1 = pts[smpl];
        y1 = pts[smpl+1];
        x2 = pts[smpl+2];
        y2 = pts[smpl+3];

        (*A_ptr++) = x2*x1;
        (*A_ptr++) = x2*y1;
        (*A_ptr++) = x2;
        (*A_ptr++) = y2*x1;
        (*A_ptr++) = y2*y1;
        (*A_ptr++) = y2;
        (*A_ptr++) = x1;
        (*A_ptr++) = y1;
        (*A_ptr++) = 1;
    }

    // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
    // the solution is linear subspace of dimensionality 2.
    // => use the last two singular vectors as a basis of the space
    // (according to SVD properties)
    cv::Mat cov = A.t() * A;

    float f[9];
    cv::Mat_<float> F(3, 3, f), evals(1, 9), evecs(9, 9);

    cv::eigen(cov, evals, evecs);

    for (int i = 0; i < 9; i++) {
        f[i] = evecs.at<float>(8, i);
    }

    F.copyTo(F_out);
}

#endif //USAC_EIGHTPOINTSALGORITHM_H
