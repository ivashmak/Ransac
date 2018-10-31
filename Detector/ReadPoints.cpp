#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "ReadPoints.h"

/*
 * Get correspondence points from file
 * Assume syntax as
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void read_points (cv::Mat &pts1, cv::Mat &pts2, const std::string &filename) {
    std::fstream file(filename, std::ios_base::in);

    float x1, y1, z1, x2, y2, z2, inl;
    cv::Mat tmp = cv::Mat_<float>(1, 2);
    float eps = 3;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        float * pts1_ptr = (float *) pts1.data;
        float * pts2_ptr = (float *) pts2.data;
        bool cont = false;

        /*
         * Skip repeated points
         */
//        for (int i = 0; i < pts1.rows; i++) {
//            if ((pts1_ptr[2*i] <= x1+eps) && (pts1_ptr[2*i] >= x1-eps)){
//                if ((pts1_ptr[2*i+1] <= y1+eps) && (pts1_ptr[2*i+1] >= y1-eps)){
//                    cont = true;
//                    break;
//                }
//            }
//
//            if ((pts2_ptr[2*i] <= x2+eps) && (pts2_ptr[2*i] >= x2-eps)){
//                if ((pts2_ptr[2*i+1] <= y2+eps) && (pts2_ptr[2*i+1] >= y2-eps)){
//                    cont = true;
//                    break;
//                }
//            }
//        }

        if (cont) continue;

        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);

    }
}

/*
 * Get inliers (isinlier1, ..., isinlierN) from file with syntax
 * x1 y1 z1 x2 y2 z2 isinlier1
 * ...
 * xN yN zN xN yN zN isinlierN
 */
void getInliers (const std::string &filename, std::vector<int> &inliers) {
    std::fstream file(filename, std::ios_base::in);

    cv::Mat tmp = cv::Mat_<float>(1, 2), pts1, pts2;
    float x1, y1, z1, x2, y2, z2;
    int inl;
    int p = 0;
    float eps = 3;
    while (file >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> inl) {
        float * pts1_ptr = (float *) pts1.data;
        float * pts2_ptr = (float *) pts2.data;
        bool cont = false;

        /*
         * Skip repeated points
         */
//        for (int i = 0; i < pts1.rows; i++) {
//            if ((pts1_ptr[2*i] <= x1+eps) && (pts1_ptr[2*i] >= x1-eps)){
//                if ((pts1_ptr[2*i+1] <= y1+eps) && (pts1_ptr[2*i+1] >= y1-eps)){
//                    cont = true;
//                    break;
//                }
//            }
//
//            if ((pts2_ptr[2*i] <= x2+eps) && (pts2_ptr[2*i] >= x2-eps)){
//                if ((pts2_ptr[2*i+1] <= y2+eps) && (pts2_ptr[2*i+1] >= y2-eps)){
//                    cont = true;
//                    break;
//                }
//            }
//        }

        if (cont) continue;
        if (inl) inliers.push_back(p);
        p++;

        tmp.at<float>(0) = x1;
        tmp.at<float>(1) = y1;

        pts1.push_back(tmp);

        tmp.at<float>(0) = x2;
        tmp.at<float>(1) = y2;

        pts2.push_back(tmp);
    }
}

/*
 * Get matrix from file.
 * Assume file syntax as
 * a11 a12 a13
 * a21 a22 a23
 * a31 a32 a33
 */
void getMatrix3x3 (const std::string &filename, cv::OutputArray H) {
    cv::Mat H_ = cv::Mat_<float>(3,3);
    std::fstream file(filename, std::ios_base::in);

    float val;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file >> val;
            H_.at<float>(i,j) = val;
        }
    }

    H_.copyTo (H);
}