#ifndef USAC_ESSENTIALMATRIXESTIMATION_H
#define USAC_ESSENTIALMATRIXESTIMATION_H

// 5-point algorithm
// An Efficient Solution to the Five-Point Relative Pose Problem
// David Nistér
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.86.8769&rep=rep1&type=pdf


#include <theia/theia.h>
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
        sampler->setRange(0, total_points-1);
        sampler->setSampleSize(npoints); 
        int * samples = new int[npoints];
        sampler->generateSample(samples);
        samples[0] = 6;
        samples[1] = 32;
        samples[2] = 34;
        samples[3] = 43;
        samples[4] = 45;

        cv::Mat_<float> q1(2, npoints), q2 (2, npoints);
        for (int i = 0; i < npoints; i++) {
            q1(0, i) = points1.at<float>(samples[i], 0);
            q1(1, i) = points1.at<float>(samples[i], 1);
//            q1(2, i) = 1;

            q2(0, i) = points2.at<float>(samples[i], 0);
            q2(1, i) = points2.at<float>(samples[i], 1);
//            q2(2, i) = 1;
        }

        cv::transpose(q1, q1); cv::transpose(q2, q2);
        cv::Mat E = cv::findEssentialMat(q1, q2);

        for (int i = 0; i < E.rows; i+=3) {
            std::cout << "E"<<((i/3)+1) <<" = \n" <<E.rowRange(i, i+3) << "\n\n";
        }
        exit (0);

        // camera Matrix
        cv::Mat camera_matrix;
        camera_matrix = (cv::Mat_<float>(3,3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);

        // In this case, we can always assume that the image points q and
        // q' have been premultiplied by K1^-1 and K2^-1, respectively

        q1 = camera_matrix.inv()*q1; // q
        q2 = camera_matrix.inv()*q2; // q'


        std::cout << "q1 = \n" << q1 << "\n\n";
        std::cout << "q2 = \n" << q2 << "\n\n";

        cv::Mat_<float> q (npoints, 9);
        std::vector<cv::Mat_<float>> qcolumns (9);

        // q (estimation)  = [q1q1'  q2q1'  q3q1'  q1q2'  q2q2'  q3q2'  q1q3'  q2q3'  q3q3']^T

        qcolumns[0] = q1.row(0).mul(q2.row(0));
        qcolumns[1] = q1.row(1).mul(q2.row(0));
        qcolumns[2] = q1.row(2).mul(q2.row(0));
        qcolumns[3] = q1.row(0).mul(q2.row(1));
        qcolumns[4] = q1.row(1).mul(q2.row(1));
        qcolumns[5] = q1.row(2).mul(q2.row(1));
        qcolumns[6] = q1.row(0).mul(q2.row(2));
        qcolumns[7] = q1.row(1).mul(q2.row(2));
        qcolumns[8] = q1.row(2).mul(q2.row(2));

        for (int i = 0; i < 9; i++) {
            cv::transpose(qcolumns[i], qcolumns[i]);
            qcolumns[i].copyTo(q.col(i));
        }

        std::cout << q << "\n\n";

        Eigen::MatrixXd Q (5, 9);
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 9; j++) {
                Q(i, j) = q.at<float>(i,j);
            }
        }

        // Four vectors X, Y, Z, W that span the right nullspace of this matrix are now computed.

        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp (Q);
        Eigen::MatrixXd null_space = lu_decomp.kernel();
        std::cout << null_space << "\n\n";

        // The four vectors correspond directly to four 3×3 matrices X, Y, Z, W
        //and the essential matrix must be of the form E = xX + yY + zZ + wW, w = 1;

        cv::Mat_<float> X(3, 3), Y (3, 3), Z (3, 3), W (3, 3);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                X.at<float> (i, j) = (float) null_space(i*3+j, 0);
                Y.at<float> (i, j) = (float) null_space(i*3+j, 1);
                Z.at<float> (i, j) = (float) null_space(i*3+j, 2);
                W.at<float> (i, j) = (float) null_space(i*3+j, 3);
            }
        }

        std::cout << X << "\n\n" << Y << "\n\n" << Z << "\n\n" << W << "\n\n";


    }
};


#endif //USAC_ESSENTIALMATRIXESTIMATION_H
