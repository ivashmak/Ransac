#ifndef RANSAC_GETNORM_TRAN_H
#define RANSAC_GETNORM_TRAN_H

#include <cmath>
#include <opencv2/core.hpp>
#include <iostream>

void GetNormalizingTransformation (const float * const pts, cv::OutputArray norm_points, const int * const sample, int sample_number, cv::Mat &T1, cv::Mat &T2) {

    float mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0;

    int smpl;
    for (int i = 0; i < sample_number; i++) {
        smpl = 4*sample[i];
        mean_pts1_x += pts[smpl];
        mean_pts1_y += pts[smpl+1];
        mean_pts2_x += pts[smpl+2];
        mean_pts2_y += pts[smpl+3];
    }
    mean_pts1_x /= sample_number;
    mean_pts1_y /= sample_number;
    mean_pts2_x /= sample_number;
    mean_pts2_y /= sample_number;

    float summa_pts1_x = 0, summa_pts1_y = 0, summa_pts2_x = 0, summa_pts2_y = 0;

    float x1i, y1i, x2i, y2i;
    for (int i = 0; i < sample_number; i++) {
        smpl = 4*sample[i];
        x1i = pts[smpl] - mean_pts1_x;
        y1i = pts[smpl+1] - mean_pts1_y;
        x2i = pts[smpl+2] - mean_pts2_x;
        y2i = pts[smpl+3] - mean_pts2_y;

        summa_pts1_x += x1i * x1i;
        summa_pts1_y += y1i * y1i;

        summa_pts2_x += x2i * x2i;
        summa_pts2_y += y2i * y2i;
    }

    /*
     * s1 = sqrt(summa1/NUMP);
     * s2 = sqrt(summa2/NUMP);
     * s=sqrt((summa1+summa2)/NUMP)/sqrt(2);
     */

    float pts1_s = sqrt((summa_pts1_x + summa_pts1_y)/ sample_number) / sqrt(2);
    float pts2_s = sqrt((summa_pts2_x + summa_pts2_y)/ sample_number) / sqrt(2);

    /*
     * T1 = eye(3);
     * T2 = eye(3);
     * T1(1:2,3) = -offset;
     * T2(1,1) = 1.0/s;
     * T2(2,2) = 1.0/s;
     * T=T2*T1;
     *
     * pts1_T1 = [ 1 0 -mean_pts1_x;
     *             0 1 -mean_pts1_y
     *             0 0 1]
     *
     * pts1_T2 = [ 1/pts1_s  0          0
     *             0         1/pts1_s   0
     *             0         0          1]
     *
     * T1 = [1/pts1_s  0          -mean_pts1_x/pts1_s
     *       0         1/pts1_s   -mean_pts1_y/pts1_s
     *       0         0          1]
     *
     */

    T1 = (cv::Mat_<float>(3,3) << 1/pts1_s, 0, -mean_pts1_x/pts1_s, 0, 1/pts1_s, -mean_pts1_y/pts1_s, 0, 0, 1);
    T2 = (cv::Mat_<float>(3,3) << 1/pts2_s, 0, -mean_pts2_x/pts2_s, 0, 1/pts2_s, -mean_pts2_y/pts2_s, 0, 0, 1);

    float *T1_ptr = (float *)T1.data;
    float *T2_ptr = (float *)T2.data;

    cv::Mat normalized_points = cv::Mat_<float> (sample_number, 4);
    float *normalized_points_ptr = (float *) normalized_points.data;

    /*
     * Normalized points
     * Norm_img1_x1 Norm_img1_y1 Norm_img2_x1 Norm_img2_y1
     * Norm_img1_x2 Norm_img1_y2 Norm_img2_x2 Norm_img2_y2
     * ...
     * Norm_img1_xn Norm_img1_yn Norm_img2_xn Norm_img2_yn
     *
     * Npts1 = T1*pts1    3x3 * 3xN
     * Npts2 = T2*pts2    3x3 * 3xN
     *
     * Npts = [Npts1; Npts2]
     *
     * Fast T*pts multiplication below
     * We don't need third coordinate for points and third row for T,
     * because third column for output points is z(i) = 1
     *
     * N_x1 = T(1,1) * x1 + T(1,3)
     * N_y1 = T(2,2) * y1 + T(2,3)
     *
     * We don't need T(1,2) * y1 and T(2,2) * x1 because T(1,2) = T(2,1) = 0
     */
    for (int i = 0; i < sample_number; i++) {
        smpl = 4*i;
        normalized_points_ptr[smpl]   = T1_ptr[0] * pts[smpl]   + T1_ptr[2]; // Norm_img1_xi
        normalized_points_ptr[smpl+1] = T1_ptr[4] * pts[smpl+1] + T1_ptr[5]; // Norm_img1_yi

        normalized_points_ptr[smpl+2] = T1_ptr[0] * pts[smpl+2] + T1_ptr[2]; // Norm_img2_xi
        normalized_points_ptr[smpl+3] = T1_ptr[4] * pts[smpl+3] + T1_ptr[5]; // Norm_img2_yi
    }

    normalized_points.copyTo(norm_points);
}

#endif // RANSAC_GETNORM_TRAN_H