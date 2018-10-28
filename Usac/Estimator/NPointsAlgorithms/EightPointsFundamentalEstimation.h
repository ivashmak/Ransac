#ifndef USAC_EIGHTPOINTSALGORITHM_H
#define USAC_EIGHTPOINTSALGORITHM_H
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//M*/

#include <opencv2/core/mat.hpp>
#include "../DLT/DLT.h"

//void EightPointsAlgorithm (const float * const pts, const int * const sample, int sample_number, cv::OutputArray F_out) {
//    // form a linear system: i-th row of A(=a) represents
//    // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
//    int smpl;
//    float x1, y1, x2, y2;
//    cv::Mat_<float> A(sample_number, 9);
//    float *A_ptr = (float *) A.data;
//
//    for (int i = 0; i < sample_number; i++) {
//        smpl = 4*sample[i];
//        x1 = pts[smpl];
//        y1 = pts[smpl+1];
//        x2 = pts[smpl+2];
//        y2 = pts[smpl+3];
//
//        (*A_ptr++) = x2*x1;
//        (*A_ptr++) = x2*y1;
//        (*A_ptr++) = x2;
//        (*A_ptr++) = y2*x1;
//        (*A_ptr++) = y2*y1;
//        (*A_ptr++) = y2;
//        (*A_ptr++) = x1;
//        (*A_ptr++) = y1;
//        (*A_ptr++) = 1;
//    }
//
//    // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
//    // the solution is linear subspace of dimensionality 2.
//    // => use the last two singular vectors as a basis of the space
//    // (according to SVD properties)
//    cv::Mat cov = A.t() * A;
//
//    cv::Mat_<float> evals(1, 9), evecs(9, 9), F;
//    cv::eigen(cov, evals, evecs);
//
//    evecs.row (8).copyTo(F);
//    F = cv::Mat(evecs.row(8).reshape(3,3));
//
////    F = F / F.at<float>(2,2);
//
//    F.copyTo(F_out);
//}


void EightPointsAlgorithm (const float * const pts, const int * const sample, int sample_number, cv::OutputArray F_out) {

    cv::Mat T1, T2, norm_points;
    GetNormalizingTransformation(pts, norm_points, sample, sample_number, T1, T2);

    const float * const norm_points_ptr = (float *) norm_points.data;
    cv::Matx<double, 9, 9> A;

    // form a linear system Ax=0: for each selected pair of points m1 & m2,
    // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
    // to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.
    float x1, x2, y1, y2;
    unsigned int smpl;
    for (int i = 0; i < sample_number; i++) {
        x1 = norm_points_ptr[4*i];
        y1 = norm_points_ptr[4*i+1];
        x2 = norm_points_ptr[4*i+2];
        y2 = norm_points_ptr[4*i+3];
        cv::Vec<double, 9> r( x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1 );
        A += r*r.t();
    }

    cv::Vec<double, 9> W;
    cv::Matx<double, 9, 9> V;

    cv::eigen(A, W, V);

    int i;
    for( i = 0; i < 9; i++ ) {
        if( fabs(W[i]) < DBL_EPSILON )
            break;
    }

    if( i < 8 ) {
        std::cout << "smth wrong\n";
        exit (0);
    }

    cv::Matx33d F0( V.val + 9*8 ); // take the last column of v as a solution of Af = 0

    // make F0 singular (of rank 2) by decomposing it with SVD,
    // zeroing the last diagonal element of W and then composing the matrices back.

    cv::Vec3d w;
    cv::Matx33d U;
    cv::Matx33d Vt;

    cv::SVD::compute( F0, w, U, Vt);
    w[2] = 0.;

    F0 = U*cv::Matx33d::diag(w)*Vt;

    // apply the transformation that is inverse
    // to what we used to normalize the point coordinates

    cv::Matx33d T11 (T1.at<float>(0,0), T1.at<float>(0,1), T1.at<float>(0,2),
            T1.at<float>(1,0), T1.at<float>(1,1), T1.at<float>(1,2),
            T1.at<float>(2,0), T1.at<float>(2,1), T1.at<float>(2,2));

    cv::Matx33d T22 (T2.at<float>(0,0), T2.at<float>(0,1), T2.at<float>(0,2),
                     T2.at<float>(1,0), T2.at<float>(1,1), T2.at<float>(1,2),
                     T2.at<float>(2,0), T2.at<float>(2,1), T2.at<float>(2,2));
    F0 = T22.t()*F0*T11;

    // make F(3,3) = 1
    if( fabs(F0(2,2)) > FLT_EPSILON )
        F0 *= 1./F0(2,2);

    cv::Mat(F0).copyTo(F_out);
}
#endif //USAC_EIGHTPOINTSALGORITHM_H
