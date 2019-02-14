// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/dlt.hpp"

bool cv::usac::DLt::DLT4p (const int * const sample, cv::Mat &H) {
    float x1, y1, x2, y2;
    int smpl;

    cv::Mat_<float> A (8, 9), w, u, vt;
    auto * A_ptr = (float *) A.data;

    for (int i = 0; i < 4; i++) {
        smpl = 4*sample[i];
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = x2*x1;
        (*A_ptr++) = x2*y1;
        (*A_ptr++) = x2;

        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = y2*x1;
        (*A_ptr++) = y2*y1;
        (*A_ptr++) = y2;
    }

    cv::SVD::compute(A, w, u, vt);
    if (vt.empty()) {
        return false;
    }

    H = cv::Mat_<float>(vt.row(vt.rows-1).reshape (3,3));
    // normalize H by last h33
    H = H / H.at<float>(2,2);

    return true;
}

bool cv::usac::DLT (const float * const points, unsigned int sample_number, cv::Mat &H) {

    float x1, y1, x2, y2;
    unsigned int smpl;

    cv::Mat_<float> A (2*sample_number, 9), S, U, Vt;
    auto * A_ptr = (float *) A.data;

    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4*i;
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = x2*x1;
        (*A_ptr++) = x2*y1;
        (*A_ptr++) = x2;

        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = -x1;
        (*A_ptr++) = -y1;
        (*A_ptr++) = -1;
        (*A_ptr++) = y2*x1;
        (*A_ptr++) = y2*y1;
        (*A_ptr++) = y2;
    }

    cv::SVD::compute(A, S, U, Vt);

    if (Vt.empty ()) {
        return false;
    }

    H = cv::Mat_<float>(Vt.row(Vt.rows-1).reshape (3,3));

    return true;
}

bool cv::usac::DLTEigen (const float * const points, unsigned int sample_number, cv::Mat &H) {
    float x1, y1, x2, y2;
    unsigned int smpl;
    cv::Mat_<float> AtA (9, 9, float (0)), Vt, D;
    float a1[9] = {0, 0, -1, 0, 0, 0, 0, 0, 0}, a2[9] = {0, 0, 0, 0, 0, -1, 0, 0, 0};
    float * AtA_ptr = (float *) AtA.data;

    for (int i = 0; i < sample_number; i++) {
        smpl = 4*i;
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        a1[0] = -x1;
        a1[1] = -y1;
        a1[6] = x2*x1;
        a1[7] = x2*y1;
        a1[8] = x2;

        a2[3] = -x1;
        a2[4] = -y1;
        a2[6] = y2*x1;
        a2[7] = y2*y1;
        a2[8] = y2;

        for (unsigned int j = 0; j < 9; j++) {
            for (unsigned int z = j; z < 9; z++) {
                AtA_ptr[j*9+z] += a1[j]*a1[z] + a2[j]*a2[z];
            }
        }
    }

    for (unsigned int row = 1; row < 9; row++) {
        for (unsigned int col = 0; col < row; col++) {
            AtA_ptr[row*9+col] = AtA_ptr[col*9+row];
        }
    }

    cv::eigen(AtA, D, Vt);

    if (Vt.empty ()) {
        return false;
    }

    H = cv::Mat_<float>(Vt.row(Vt.rows-1).reshape (3,3));

    return true;
}

bool cv::usac::DLTLeastSquares (const float * const points, unsigned int sample_number, cv::Mat &H) {
    /*
     * A is 2N x 8
     * b is 2N x 1
     *
     * A h = b
     * h = A^(+) b
     * h = (A^T A) ^(-1) * A^T * b
     */
    cv::Mat_<float> b (2*sample_number, 1);
    cv::Mat_<float> A (2*sample_number, 8);
    auto * A_ptr = (float *) A.data;
    auto * b_ptr = (float *) b.data;
    unsigned int smpl;
    float x1, y1, x2, y2;

    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4*i;
        x1 = points[smpl];
        y1 = points[smpl+1];

        x2 = points[smpl+2];
        y2 = points[smpl+3];

        (*A_ptr++) = x1;
        (*A_ptr++) = y1;
        (*A_ptr++) = 1;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = -x2*x1;
        (*A_ptr++) = -x2*y1;

        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = 0;
        (*A_ptr++) = x1;
        (*A_ptr++) = y1;
        (*A_ptr++) = 1;
        (*A_ptr++) = -y2*x1;
        (*A_ptr++) = -y2*y1;

        (*b_ptr++) = x2;
        (*b_ptr++) = y2;
    }

    // 8 x 1
    H = (A.t() * A).inv() * A.t() * b;
    H = (cv::Mat_<float>(3,3) <<
            H.at<float>(0), H.at<float>(1), H.at<float>(2),
            H.at<float>(3), H.at<float>(4), H.at<float>(5),
            H.at<float>(6), H.at<float>(7), 1);
    return true;
}
