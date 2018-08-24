#ifndef USAC_ESSENTIALMATRIXESTIMATION_H
#define USAC_ESSENTIALMATRIXESTIMATION_H

// 5-point algorithm
// An Efficient Solution to the Five-Point Relative Pose Problem
// David Nistér
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.86.8769&rep=rep1&type=pdf

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include "Sampler/UniformSampler.h"

class EssentialMatrixEstimation {
protected:

public:
    void fivePointsAlg (cv::InputArray input_points1, cv::InputArray input_points2, cv::Mat& ess_mat, double focal=1.0, cv::Point2d pp=cv::Point2d(0, 0)) {
        int npoints = 5;

        int total_points;
        if (input_points1.isMat()) {
            total_points = input_points1.getMat().rows;
        } else {
            total_points = input_points1.size().width;
        }

        cv::Mat points1 = cv::Mat(total_points, 2, CV_32F, input_points1.getMat().data);
        cv::Mat points2 = cv::Mat(total_points, 2, CV_32F, input_points2.getMat().data);

        Sampler *sampler = new UniformSampler;
        int * samples = new int[npoints];
        sampler->getSample(samples, npoints, total_points);

        cv::Mat_<float> q1(3, npoints), q2 (3, npoints);
        for (int i = 0; i < npoints; i++) {
            q1(0, i) = points1.at<float>(samples[i], 0);
            q1(1, i) = points1.at<float>(samples[i], 1);
            q1(2, i) = 1;

            q2(0, i) = points2.at<float>(samples[i], 0);
            q2(1, i) = points2.at<float>(samples[i], 1);
            q2(2, i) = 1;
        }

        // camera Matrix
        cv::Mat camera_matrix;
        camera_matrix = (cv::Mat_<float>(3,3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);

        //  In this case, we can always assume that the image points q and
        //q have been premultiplied by K1^−1 and K2^-1, respectively

        q1 = camera_matrix.inv()*q1;
        q2 = camera_matrix.inv()*q2;


        std::cout << "q1 = \n" << q1 << "\n\n";
        std::cout << "q2 = \n" << q2 << "\n\n";

        cv::Mat_<float> q (npoints, 9);
        cv::Mat_<float> qc1;

    }
};


#endif //USAC_ESSENTIALMATRIXESTIMATION_H
