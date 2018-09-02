#ifndef USAC_FUNDAMENTALMATRIXESTIMATION_H
#define USAC_FUNDAMENTALMATRIXESTIMATION_H

// 7,8-point algortihm
// R. I. Hartley and  A. Zisserman, Multiple  View  Geometry  in  Computer Vision. Cambridge University Press, 2
// http://cvrs.whu.edu.cn/downloads/ebooks/Multiple%20View%20Geometry%20in%20Computer%20Vision%20(Second%20Edition).pdf
// page 279

#include <opencv2/core/mat.hpp>

class FundamentalMatrixEstimation {
protected:

public:
    void eightPointsAlg(cv::InputArray input_points1, cv::InputArray input_points2, double focal = 1.0,
                        cv::Point2d pp = cv::Point2d(0, 0)) {

        int total_points;
        if (input_points1.isMat()) {
            total_points = input_points1.getMat().rows;
        } else {
            total_points = input_points1.size().width;
        }

        cv::Mat points1 = cv::Mat(total_points, 2, CV_32F, input_points1.getMat().data);
        cv::Mat points2 = cv::Mat(total_points, 2, CV_32F, input_points2.getMat().data);


    }

    void sevenPointsAlg(cv::InputArray input_points1, cv::InputArray input_points2, double focal = 1.0,
                        cv::Point2d pp = cv::Point2d(0, 0)) {

    }

};

#endif //USAC_FUNDAMENTALMATRIXESTIMATION_H
