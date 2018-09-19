#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Direct Linear Transformation
void DLT (cv::InputArray pts1, cv::InputArray pts2, cv::Mat &H) {
    assert(!pts1.empty());
    assert(!pts2.empty());

    cv::Mat points1 = pts1.getMat();
    cv::Mat points2 = pts2.getMat();

    int NUMP = points1.rows;

    float x1, y1, x2, y2;

    std::cout << "NUMP = "<< points1.size << "\n";

    cv::Mat_<float> A (2*NUMP, 9), tmp1, tmp2, vt;

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

    std::cout << "here\n";

    std::cout << vt << "\n\n";
    std::cout << vt.col (8) << "\n\n";

    vt.col(8).copyTo(H);

    std::cout << H << '\n';

    H = (cv::Mat_<float>(3,3) <<
            H.at<float>(0), H.at<float>(1), H.at<float>(2),
            H.at<float>(3), H.at<float>(4), H.at<float>(5),
            H.at<float>(6), H.at<float>(7), H.at<float>(8));

    std::cout << "here\n";

}