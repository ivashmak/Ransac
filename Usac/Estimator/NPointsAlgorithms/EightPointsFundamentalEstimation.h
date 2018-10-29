#ifndef USAC_EIGHTPOINTSALGORITHM_H
#define USAC_EIGHTPOINTSALGORITHM_H

#include <opencv2/core/mat.hpp>
#include "../DLT/DLT.h"

bool EightPointsAlgorithm (const float * const pts, const int * const sample, int sample_number, cv::Mat &F) {

    cv::Mat_<double> T1, T2, norm_points;
    GetNormalizingTransformation(pts, norm_points, sample, sample_number, T1, T2);

    const float * const norm_points_ptr = (float *) norm_points.data;
    cv::Mat_ <double> A(9, 9, double (0));
    double * A_ptr = (double *) A.data;

    // form a linear system Ax=0: for each selected pair of points m1 & m2,
    // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
    // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
    double x1, x2, y1, y2;
    unsigned int norm_points_idx;

    for (int i = 0; i < sample_number; i++) {
        norm_points_idx = 4*i;
        x1 = norm_points_ptr[norm_points_idx];
        y1 = norm_points_ptr[norm_points_idx+1];
        x2 = norm_points_ptr[norm_points_idx+2];
        y2 = norm_points_ptr[norm_points_idx+3];
        double a [] = {x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1};

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

    cv::Mat_<double> eigen_values, eigen_vectors;
    cv::eigen(A, eigen_values, eigen_vectors);

    double * W_ptr = (double *) eigen_values.data;

    int i;
    for (i = 0; i < 9; i++) {
        if (fabs(W_ptr[i]) < DBL_EPSILON) break;
    }

    if (i < 8) {
        std::cout << "Eigen values are too small\n";
//        return false;
    }


    F = cv::Mat_<double> (eigen_vectors.row(8).reshape (3,3));

    // make F0 singular (of rank 2) by decomposing it with SVD,
    // zeroing the last diagonal element of W and then composing the matrices back.
    cv::Mat_<double> w, U, Vt;
    cv::SVD::compute (F, w, U, Vt);

    // create from w diagonal matrix with w33 = 0
    w = (cv::Mat_<double> (3, 3) << w.at<double>(0), 0, 0,
                                    0, w.at<double>(1), 0,
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

    T1 = (cv::Mat_<double> (3,3) <<
            T1.at<float>(0,0), 0, T1.at<float>(0,2),
            0, T1.at<float>(1,1), T1.at<float>(1,2),
            0, 0, 1);

    // Transpose T2
    T2 = (cv::Mat_<double> (3,3) <<
            T2.at<float>(0,0), 0, 0,
            0, T2.at<float>(1,1), 0,
            T2.at<float>(0,2), T2.at<float>(1,2), 1);

    F = T2 * F * T1;

    // make F(3,3) = 1
    if (fabs(F.at<double>(2,2)) > FLT_EPSILON) {
        F = F / F.at<double>(2, 2);
    }

    /* reassign to float (not necessary).
     * OpenCV can't apply function on matrix different type.
     * So we should be consistent if we use only float or only double.
     * In this function better to use double, otherwise result is significantly
     * different. But for other function I use float.
     */
    F = cv::Mat_<float> (F);

    return true;
}


/*
 *
        F =
        [-1.240528e-06, -2.4659263e-05, 0.0043561663;
         2.3357645e-05, 4.1720361e-07, -0.013281886;
         -0.0044564442, 0.010273063, 1]

        avg error is    2.0770e-09

        Ff =
        [-4.7419455e-09, -3.8766225e-06, 0.0011870193;
         -3.7750447e-06, 2.8917873e-06, -0.02240199;
         0.0013357102, 0.021094287, 1]

        avg error is    2.1823e-09

 */

bool EightPointsAlgorithm_float (const float * const pts, const int * const sample, int sample_number, cv::Mat &F) {

    cv::Mat_<float> T1, T2, norm_points;
    GetNormalizingTransformation(pts, norm_points, sample, sample_number, T1, T2);

    const float * const norm_points_ptr = (float *) norm_points.data;
    cv::Mat_ <float> A(9, 9, float (0));
    float * A_ptr = (float *) A.data;

    float x1, x2, y1, y2;
    unsigned int norm_points_idx;

    for (int i = 0; i < sample_number; i++) {
        norm_points_idx = 4*i;
        x1 = norm_points_ptr[norm_points_idx];
        y1 = norm_points_ptr[norm_points_idx+1];
        x2 = norm_points_ptr[norm_points_idx+2];
        y2 = norm_points_ptr[norm_points_idx+3];
        float a [] = {x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1};

        for (int row = 0; row < 9; row++) {
            for (int col = row; col < 9; col++) {
                A_ptr[row*9+col] += a[row]*a[col];
            }
        }
    }

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


    T1 = (cv::Mat_<float> (3,3) <<
                                 T1.at<float>(0,0), 0, T1.at<float>(0,2),
            0, T1.at<float>(1,1), T1.at<float>(1,2),
            0, 0, 1);

    T2 = (cv::Mat_<float> (3,3) <<
                                 T2.at<float>(0,0), 0, 0,
            0, T2.at<float>(1,1), 0,
            T2.at<float>(0,2), T2.at<float>(1,2), 1);

    F = T2 * F * T1;

    if (fabs(F.at<float>(2,2)) > FLT_EPSILON) {
        F = F / F.at<float>(2, 2);
    }

    F = cv::Mat_<float> (F);
    return true;
}
#endif //USAC_EIGHTPOINTSALGORITHM_H
