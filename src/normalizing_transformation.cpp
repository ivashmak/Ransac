// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "precomp.hpp"
#include "../include/opencv2/usac/dlt.hpp"

void cv::usac::GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, cv::Mat &T1, cv::Mat &T2) {

    float mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0;

    unsigned int smpl;
    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4 * sample[i];

        mean_pts1_x += pts[smpl];
        mean_pts1_y += pts[smpl + 1];
        mean_pts2_x += pts[smpl + 2];
        mean_pts2_y += pts[smpl + 3];
    }

    mean_pts1_x /= sample_number;
    mean_pts1_y /= sample_number;
    mean_pts2_x /= sample_number;
    mean_pts2_y /= sample_number;

    float avg_dist1 = 0, avg_dist2 = 0, x1_m, y1_m, x2_m, y2_m;
    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4 * sample[i];

        x1_m = pts[smpl    ] - mean_pts1_x;
        y1_m = pts[smpl + 1] - mean_pts1_y;
        x2_m = pts[smpl + 2] - mean_pts2_x;
        y2_m = pts[smpl + 3] - mean_pts2_y;

        avg_dist1 += sqrt (x1_m * x1_m + y1_m * y1_m);
        avg_dist2 += sqrt (x2_m * x2_m + y2_m * y2_m);
    }

    // scale
    avg_dist1 = M_SQRT2 / (avg_dist1 / sample_number);
    avg_dist2 = M_SQRT2 / (avg_dist2 / sample_number);


    T1 = (cv::Mat_<float>(3, 3) << avg_dist1, 0, -mean_pts1_x * avg_dist1,
                                    0, avg_dist1, -mean_pts1_y * avg_dist1,
                                    0, 0, 1);
    T2 = (cv::Mat_<float>(3, 3) << avg_dist2, 0, -mean_pts2_x * avg_dist2,
                                    0, avg_dist2, -mean_pts2_y * avg_dist2,
                                    0, 0, 1);

    auto *T1_ptr = (float *) T1.data;
    auto *T2_ptr = (float *) T2.data;

    norm_points = cv::Mat_<float>(sample_number, 4);

    auto *norm_points_ptr = (float *) norm_points.data;

    unsigned int norm_pts_idx;
    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4 * sample[i];
        norm_pts_idx = 4 * i;
        norm_points_ptr[norm_pts_idx    ] = T1_ptr[0] * pts[smpl    ] + T1_ptr[2]; // Norm_img1_xi
        norm_points_ptr[norm_pts_idx + 1] = T1_ptr[4] * pts[smpl + 1] + T1_ptr[5]; // Norm_img1_yi

        norm_points_ptr[norm_pts_idx + 2] = T2_ptr[0] * pts[smpl + 2] + T2_ptr[2]; // Norm_img2_xi
        norm_points_ptr[norm_pts_idx + 3] = T2_ptr[4] * pts[smpl + 3] + T2_ptr[5]; // Norm_img2_yi
    }
}
