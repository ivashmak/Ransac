#ifndef RANSAC_GETNORM_TRAN_H
#define RANSAC_GETNORM_TRAN_H

#include <cmath>
#include <opencv2/core.hpp>
#include <iostream>

void GetNormalizingTransformation (const float * const pts, cv::OutputArray norm_points, const int * const sample, int sample_number, cv::Mat &T1, cv::Mat &T2) {

    cv::Mat_<float> points1 (sample_number, 2);
    cv::Mat_<float> points2 (sample_number, 2);

    float *points1_ptr = (float *) points1.data;
    float *points2_ptr = (float *) points2.data;

    float mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0;

    for (int i = 0; i < sample_number; i++) {
        points1_ptr[2*i] = pts[4*sample[i]];
        points1_ptr[2*i+1] = pts[4*sample[i]+1];
        points2_ptr[2*i] = pts[4*sample[i]+2];
        points2_ptr[2*i+1] = pts[4*sample[i]+3];

        mean_pts1_x += points1_ptr[2*i];
        mean_pts1_y += points1_ptr[2*i+1];

        mean_pts2_x += points2_ptr[2*i];
        mean_pts2_y += points2_ptr[2*i+1];
    }
    mean_pts1_x /= sample_number;
    mean_pts1_y /= sample_number;
    mean_pts2_x /= sample_number;
    mean_pts2_y /= sample_number;

    cv::Mat offset1 = (cv::Mat_<float> (1,2) << mean_pts1_x, mean_pts1_y);
    cv::Mat offset2 = (cv::Mat_<float> (1,2) << mean_pts2_x, mean_pts2_y);

    cv::Mat ones = cv::Mat_<float>::ones(sample_number, 1);

    cv::Mat pts1_offseted = points1;
    cv::Mat pts2_offseted = points2;

    pts1_offseted.col(0) -= mean_pts1_x;
    pts1_offseted.col(1) -= mean_pts1_y;

    pts2_offseted.col(0) -= mean_pts1_x;
    pts2_offseted.col(1) -= mean_pts1_y;

    float * pts1_offseted_ptr = (float *) pts1_offseted.data;
    float * pts2_offseted_ptr = (float *) pts2_offseted.data;

    float summa_pts1_x = 0, summa_pts1_y = 0, summa_pts2_x = 0, summa_pts2_y = 0;

    for (int i = 0; i < sample_number; i++) {
        summa_pts1_x += pts1_offseted_ptr[2*i] * pts1_offseted_ptr[2*i]; // xi * xi
        summa_pts1_y += pts1_offseted_ptr[2*i+1] * pts1_offseted_ptr[2*i+1]; // yi * yi

        summa_pts2_x += pts2_offseted_ptr[2*i] * pts2_offseted_ptr[2*i]; // xi * xi
        summa_pts2_y += pts2_offseted_ptr[2*i+1] * pts2_offseted_ptr[2*i+1]; // yi * yi
    }

    float pts1_s_x = (float) sqrt(summa_pts1_x / sample_number);
    float pts1_s_y = (float) sqrt(summa_pts1_y / sample_number);
    float pts1_s = (float) (sqrt((summa_pts1_x + summa_pts1_y) / sample_number) / sqrt(2));

    float pts2_s_x = (float) sqrt(summa_pts2_x / sample_number);
    float pts2_s_y = (float) sqrt(summa_pts2_y / sample_number);
    float pts2_s = (float) (sqrt((summa_pts2_x + summa_pts2_y) / sample_number) / sqrt(2));


    cv::Mat pts1_T1 = cv::Mat_<float>::eye(3, 3);
    cv::Mat pts1_T2 = cv::Mat_<float>::eye(3, 3);

    cv::Mat pts2_T1 = cv::Mat_<float>::eye(3, 3);
    cv::Mat pts2_T2 = cv::Mat_<float>::eye(3, 3);

    pts1_T1.at<float>(0,2) = -mean_pts1_x;
    pts1_T1.at<float>(1,2) = -mean_pts1_y;
    pts1_T2.at<float>(0,0) = 1/pts1_s;
    pts1_T2.at<float>(1,1) = 1/pts1_s;

    pts2_T1.at<float>(0,2) = -mean_pts2_x;
    pts2_T1.at<float>(1,2) = -mean_pts2_y;
    pts2_T2.at<float>(0,0) = 1/pts2_s;
    pts2_T2.at<float>(1,1) = 1/pts2_s;

    T1 = pts1_T2*pts1_T1;
    T2 = pts2_T2*pts2_T1;

    cv::hconcat(points1, ones, points1);
    cv::hconcat(points2, ones, points2);

    cv::transpose(points1, points1);
    cv::transpose(points2, points2);

    points1 = T1*points1;
    points2 = T2*points2;

    cv::Mat norm;
    cv::vconcat(points1.rowRange(0,2), points2.rowRange(0,2), norm);
    cv::transpose(norm, norm);
    norm.copyTo(norm_points);
}

#endif // RANSAC_GETNORM_TRAN_H