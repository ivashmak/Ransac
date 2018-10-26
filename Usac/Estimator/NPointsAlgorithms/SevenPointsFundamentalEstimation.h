#ifndef USAC_SEVENPOINTSALGORITHM_H
#define USAC_SEVENPOINTSALGORITHM_H

// 7,8-point algortihm
// R. I. Hartley and  A. Zisserman, Multiple  View  Geometry  in  Computer Vision. Cambridge University Press, 2
// http://cvrs.whu.edu.cn/downloads/ebooks/Multiple%20View%20Geometry%20in%20Computer%20Vision%20(Second%20Edition).pdf
// page 279

#include <opencv2/core/mat.hpp>
#include <iostream>

int SevenPointsAlgorithm (const float * const pts, const int * const sample, cv::OutputArray F_out) {
    float a[7*9], w[7], u[9*9], v[9*9], c[4], r[3];
    float* f1, *f2;
    float t0, t1, t2;
    cv::Mat_<float> A (7, 9, a);
    cv::Mat_<float> U (7, 9, u);
    cv::Mat_<float> Vt (9, 9, v);
    cv::Mat_<float> W (7, 1, w);
    cv::Mat_<float> coeffs (1, 4, c);
    cv::Mat_<float> roots (1, 3, r);

    // form a linear system: i-th row of A(=a) represents
    // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0

    unsigned int smpl;
    for (int i = 0; i < 7; i++ ) {
        smpl = 4*sample[i];
        float x1 = pts[smpl];
        float y1 = pts[smpl+1];
        float x2 = pts[smpl+2];
        float y2 = pts[smpl+3];

        a[i*9+0] = x2*x1;
        a[i*9+1] = x2*y1;
        a[i*9+2] = x2;
        a[i*9+3] = y2*x1;
        a[i*9+4] = y2*y1;
        a[i*9+5] = y2;
        a[i*9+6] = x1;
        a[i*9+7] = y1;
        a[i*9+8] = 1;
    }

    // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
    // the solution is linear subspace of dimensionality 2.
    // => use the last two singular vectors as a basis of the space
    // (according to SVD properties)
    cv::SVDecomp (A, W, U, Vt, cv::SVD::MODIFY_A + cv::SVD::FULL_UV);
    f1 = v + 7*9;
    f2 = v + 8*9;

    // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary f. matrix.
    // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
    // so f ~ lambda*f1 + (1 - lambda)*f2.
    // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to find lambda.
    // it will be a cubic equation.
    // find c - polynomial coefficients.
    for (int i = 0; i < 9; i++) {
        f1[i] -= f2[i];
    }

    t0 = f2[4]*f2[8] - f2[5]*f2[7];
    t1 = f2[3]*f2[8] - f2[5]*f2[6];
    t2 = f2[3]*f2[7] - f2[4]*f2[6];

    c[3] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2;

    c[2] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2 -
           f1[3]*(f2[1]*f2[8] - f2[2]*f2[7]) +
           f1[4]*(f2[0]*f2[8] - f2[2]*f2[6]) -
           f1[5]*(f2[0]*f2[7] - f2[1]*f2[6]) +
           f1[6]*(f2[1]*f2[5] - f2[2]*f2[4]) -
           f1[7]*(f2[0]*f2[5] - f2[2]*f2[3]) +
           f1[8]*(f2[0]*f2[4] - f2[1]*f2[3]);

    t0 = f1[4]*f1[8] - f1[5]*f1[7];
    t1 = f1[3]*f1[8] - f1[5]*f1[6];
    t2 = f1[3]*f1[7] - f1[4]*f1[6];

    c[1] = f2[0]*t0 - f2[1]*t1 + f2[2]*t2 -
           f2[3]*(f1[1]*f1[8] - f1[2]*f1[7]) +
           f2[4]*(f1[0]*f1[8] - f1[2]*f1[6]) -
           f2[5]*(f1[0]*f1[7] - f1[1]*f1[6]) +
           f2[6]*(f1[1]*f1[5] - f1[2]*f1[4]) -
           f2[7]*(f1[0]*f1[5] - f1[2]*f1[3]) +
           f2[8]*(f1[0]*f1[4] - f1[1]*f1[3]);

    c[0] = f1[0]*t0 - f1[1]*t1 + f1[2]*t2;

    // solve the cubic equation; there can be 1 to 3 roots ...
    int n = cv::solveCubic (coeffs, roots);

    if (n < 1) return n;
    cv::Mat F  = cv::Mat_<float>(n*3,3); // todo add n*3,3

    float* F_ptr = (float *) F.data;

    for (int k = 0; k < n; k++ , F_ptr += 9) {
        // for each root form the fundamental matrix
        float lambda = r[k], mu = 1;
        float s = f1[8]*r[k] + f2[8];

        // normalize each matrix, so that F(3,3) (~F[8]) == 1
        if (fabsf (s) > DBL_EPSILON) {
            mu = 1/s;
            lambda *= mu;
            F_ptr[8] = 1;
        } else {
            F_ptr[8] = 0;
        }
        for (int i = 0; i < 8; i++) {
            F_ptr[i] = f1[i] * lambda + f2[i] * mu;
        }
    }
    F.copyTo(F_out);
    return n;
}


#endif //USAC_SEVENPOINTSALGORITHM_H
