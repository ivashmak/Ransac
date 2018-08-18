#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "HomographyMethods.h"

// Direct Linear Transformation
void HomographyMethods::DLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
    CV_Assert(!pts1.empty());
    CV_Assert(!pts2.empty());

    cv::Mat points1 = pts1.getMat();
    cv::Mat points2 = pts2.getMat();

    int NUMP = points1.rows;

    float x1, y1, x2, y2;

    std::cout << "NUMP = "<< points1.size << "\n";

    cv::Mat A = cv::Mat(2*NUMP, 9, CV_32FC1), tmp1, tmp2, vt;

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

    cv::SVD::compute(A, tmp1, tmp2, vt);
    cv::transpose(vt, vt);

    cv::Mat h;
    vt.col(8).copyTo(h);

    H = (cv::Mat_<float>(3,3) <<
            h.at<float>(0), h.at<float>(1), h.at<float>(2),
            h.at<float>(3), h.at<float>(4), h.at<float>(5),
            h.at<float>(6), h.at<float>(7), h.at<float>(8));

}