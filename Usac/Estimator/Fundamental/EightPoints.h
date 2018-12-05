#ifndef USAC_EIGHTPOINTSALGORITHM_H
#define USAC_EIGHTPOINTSALGORITHM_H

#include <opencv2/core/mat.hpp>
#include "../DLT/DLT.h"

bool EightPointsAlgorithm (const float * const pts, const int * const sample, int sample_number, cv::Mat &F) {

    cv::Mat_<float> T1, T2, norm_points;
    GetNormalizingTransformation(pts, norm_points, sample, sample_number, T1, T2);

    const float * const norm_points_ptr = (float *) norm_points.data;
    cv::Mat_ <float> A(9, 9, float (0));
    float * A_ptr = (float *) A.data;

    // form a linear system Ax=0: for each selected pair of points m1 & m2,
    // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
    // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
    float x1, x2, y1, y2;
    unsigned int norm_points_idx;
    float a[9];

    for (int i = 0; i < sample_number; i++) {
        norm_points_idx = 4*i;
        x1 = norm_points_ptr[norm_points_idx];
        y1 = norm_points_ptr[norm_points_idx+1];
        x2 = norm_points_ptr[norm_points_idx+2];
        y2 = norm_points_ptr[norm_points_idx+3];
        a[0] = x2*x1;
        a[1] = x2*y1;
        a[2] = x2;
        a[3] = y2*x1;
        a[4] = y2*y1;
        a[5] = y2;
        a[6] = x1;
        a[7] = y1;
        a[8] = 1;

        /*
         * A += a * a^t, where aa^t is symmetric
         * (a1) * (a1 a2 ... a9)     (a1^2 a1a2 ... a1a9)
         * (a2)                   =  (a2a1 a2^2 ... a2a9)
         * ...                       ...
         * (a9)                      (a9a1 a9a2 ... a9^2)
         * It is enough to calculate only upper triangle of A.
         * A =
         */
        for (int row = 0; row < 9; row++) {
            for (int col = row; col < 9; col++) {
                A_ptr[row*9+col] += a[row]*a[col];
            }
        }
    }

    /*
     * [L,U,D] = split (A), L = U
     * Copy upper triangle of A to lower triangle of A.
     */
    for (int row = 1; row < 9; row++) {
        for (int col = 0; col < row; col++) {
            A_ptr[row*9+col] = A_ptr[col*9+row];
        }
    }

    cv::Mat_<float> eigen_values, eigen_vectors;
    cv::eigen(A, eigen_values, eigen_vectors);

    float * W_ptr = (float *) eigen_values.data;

    int i;
    for (i = 0; i < 9; i++) {
        if (fabs(W_ptr[i]) < DBL_EPSILON) break;
    }

    if (i < 8) {
        std::cout << "Eigen values are too small\n";
//        return false;
    }


    F = cv::Mat_<float> (eigen_vectors.row(8).reshape (3,3));

    // make F0 singular (of rank 2) by decomposing it with SVD,
    // zeroing the last diagonal element of W and then composing the matrices back.
    cv::Mat_<float> w, U, Vt;
    cv::SVD::compute (F, w, U, Vt);

    // create from w diagonal matrix with w33 = 0
    w = (cv::Mat_<float> (3, 3) << w.at<float>(0), 0, 0,
                                    0, w.at<float>(1), 0,
                                    0, 0, 0);

    F = U*w*Vt;

    /*
     * T1 = (t11 0   t13)
     *      (0   t22 t23)
     *      (0   0   1)
     *
     * T2^t = (t'11 0    0)
     *        (0    t'22 0)
     *        (t'13 t'23 1)
     * F = T2^t * F * T1
     *
     */

//    // Transpose T2
    float * t2 = (float *) T2.data;

//    std::cout << T2 << "\n\n";

   // Something crazy is going on here, if I uncomment simple transpose.

//    t2[6] = t2[2];
//    t2[7] = t2[5];
//    t2[2] = 0;
//    t2[5] = 0;

     T2 = T2.t();
//    std::cout << T2 << "\n\n";

    F = T2 * F * T1;

    // normalize f33
    if (fabs(F.at<float>(2,2)) > FLT_EPSILON) {
        F = F / F.at<float>(2, 2);
    }

    return true;
}


#endif //USAC_EIGHTPOINTSALGORITHM_H
