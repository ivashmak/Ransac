#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Direct Linear Transformation
void DLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
//    assert(!pts1.empty());
//    assert(!pts2.empty());

    cv::Mat points1 = pts1.getMat();
    cv::Mat points2 = pts2.getMat();

    int NUMP = points1.rows;

    float x1, y1, x2, y2;

//    std::cout << "NUMP = "<< points1.size << "\n";

    cv::Mat_<float> A (2*NUMP, 9), w, u, vt;

    // cvmSet
    for (int i = 1; i <= NUMP; i++) {
        x1 = points1.at<float>(i-1,0);
        y1 = points1.at<float>(i-1,1);

        x2 = points2.at<float>(i-1,0);
        y2 = points2.at<float>(i-1,1);

        A.at<float>(2*i-2, 0) = -x1;
        A.at<float>(2*i-2, 1) = -y1;
        A.at<float>(2*i-2, 2) = -1;
        A.at<float>(2*i-2, 3) = 0;
        A.at<float>(2*i-2, 4) = 0;
        A.at<float>(2*i-2, 5) = 0;
        A.at<float>(2*i-2, 6) = x2*x1;
        A.at<float>(2*i-2, 7) = x2*y1;
        A.at<float>(2*i-2, 8) = x2;

        A.at<float>(2*i-1, 0) = 0;
        A.at<float>(2*i-1, 1) = 0;
        A.at<float>(2*i-1, 2) = 0;
        A.at<float>(2*i-1, 3) = -x1;
        A.at<float>(2*i-1, 4) = -y1;
        A.at<float>(2*i-1, 5) = -1;
        A.at<float>(2*i-1, 6) = y2*x1;
        A.at<float>(2*i-1, 7) = y2*y1;
        A.at<float>(2*i-1, 8) = y2;
    }

    /*
     * src	decomposed matrix
     * w 	calculated singular values
     * u	calculated left singular vectors
     * vt	transposed matrix of right singular values
     */

    cv::SVD::compute(A, w, u, vt);

    H = cv::Mat_<float>(vt.row(vt.rows-1).reshape (3,3));

//     /* or */ cv::transpose(vt, vt); H = cv::Mat_<float>(vt.col(vt.cols-1).reshape (3,3));

}

#endif // RANSAC_DLT_H