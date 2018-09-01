#include <cmath>
#include <opencv2/core.hpp>
#include <iostream>

void GetNormalizingTransformation (cv::InputArray pts, cv::Mat &T, cv::Mat &offset, float * s, float *s1, float * s2) {
    CV_Assert(!pts.empty());

    cv::Mat points = pts.getMat();
    int NUMP = points.rows;

    std::cout << "NUMP = " << points.size << '\n';

    cv::Scalar mean1 = cv::mean(points.col(0));
    cv::Scalar mean2 = cv::mean(points.col(1));

    offset = (cv::Mat_<float> (1,2) << mean1.val[0], mean2.val[0]);
    cv::Mat ones = cv::Mat_<float>::ones(NUMP, 1);
    cv::Mat ptsOffseted = points - ones * offset;

    float summa1 = 0, summa2 = 0;

    for (int i = 0; i < NUMP; i++) {
        summa1 += pow(ptsOffseted.at<float>(i, 0), 2);
        summa2 += pow(ptsOffseted.at<float>(i, 1), 2);
    }

    *s1 = (float) sqrt(summa1 / NUMP);
    *s2 = (float) sqrt(summa2 / NUMP);
    *s = (float) (sqrt((summa1 + summa2) / NUMP) / sqrt(2));


    cv::Mat T1 = cv::Mat_<float>::eye(3, 3);
    cv::Mat T2 = cv::Mat_<float>::eye(3, 3);
    T1.at<float>(0,2) = (float) -mean1.val[0];
    T1.at<float>(1,2) = (float) -mean2.val[0];
    T2.at<float>(0,0) = 1/(*s);
    T2.at<float>(1,1) = 1/(*s);

    T = T2*T1;
    cv::transpose (offset, offset);
}