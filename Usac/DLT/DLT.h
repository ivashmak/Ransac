#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Direct Linear Transformation
void DLT (const float * const points, const int * const sample, int sample_number, cv::Mat &H) {
    float x1, y1, x2, y2;

    cv::Mat_<float> A (2*sample_number, 9), w, u, vt;
    float * A_ptr = (float *) A.data;

    for (int i = 0; i < sample_number; i++) {
        x1 = points[4*sample[i]];
        y1 = points[4*sample[i]+1];

        x2 = points[4*sample[i]+2];
        y2 = points[4*sample[i]+3];

//        std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << '\n';

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

    /*
     * src	decomposed matrix
     * w 	calculated singular values
     * u	calculated left singular vectors
     * vt	transposed matrix of right singular values
     */
    cv::SVD::compute(A, w, u, vt);

    H = cv::Mat_<float>(vt.row(vt.rows-1).reshape (3,3));
}



#endif // RANSAC_DLT_H