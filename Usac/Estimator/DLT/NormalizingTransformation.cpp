// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "DLT.h"

void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, cv::Mat &T1, cv::Mat &T2) {

    float mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0,
          avg_dist1 = 0, avg_dist2 = 0, x1, y1, x2, y2;
    
    unsigned int smpl;
    for (unsigned int i = 0; i < sample_number; i++) {
        smpl = 4 * sample[i];
        x1 = pts[smpl];
        y1 = pts[smpl+1];
        x2 = pts[smpl+2];
        y2 = pts[smpl+3];
        
        mean_pts1_x += x1;
        mean_pts1_y += y1;
        mean_pts2_x += x2;
        mean_pts2_y += y2;

        /*
         * Compute a similarity transform T that takes points xi
         * to a new set of points x̃i such that the centroid of
         * the points x̃i is the coordinate origin and their
         * average distance from the origin is √2
         *
         * origin O(0,0)
         * sqrt(x̃*x̃ + ỹ*ỹ) = sqrt(2)
         * ax*ax + by*by = 2
         */
        avg_dist1 += sqrt (x1 * x1 + y1 * y1);
        avg_dist2 += sqrt (x2 * x2 + y2 * y2);
    }

    mean_pts1_x /= sample_number;
    mean_pts1_y /= sample_number;
    mean_pts2_x /= sample_number;
    mean_pts2_y /= sample_number;

    avg_dist1 = M_SQRT2 / (avg_dist1 / sample_number);
    avg_dist2 = M_SQRT2 / (avg_dist2 / sample_number);

    /*
     * pts1_T1 = [ 1 0 -mean_pts1_x;
     *             0 1 -mean_pts1_y
     *             0 0 1]
     *
     * pts1_T2 = [ avg_dist1  0         0
     *             0         avg_dist1  0
     *             0          0         1]
     *
     * T1 = [avg_dist1  0          -mean_pts1_x*avg_dist1
     *       0         avg_dist1   -mean_pts1_y*avg_dist1
     *       0         0          1]
     *
     */

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



// Weighted Normalizing Transformation
void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &T1, cv::Mat &T2) {

    float mean_pts1_x = 0, mean_pts1_y = 0, mean_pts2_x = 0, mean_pts2_y = 0,
            avg_dist1 = 0, avg_dist2 = 0, x1, y1, x2, y2;

    unsigned int smpl;
    unsigned int wsmpl;
    for (unsigned int i = 0; i < sample_number; i++) {
        wsmpl = sample[i];
        smpl = 4 * wsmpl;
//        std::cout << "weight = " << weights[wsmpl] << "\n";
        x1 = weights[wsmpl] * pts[smpl];
        y1 = weights[wsmpl] * pts[smpl+1];
        x2 = weights[wsmpl] * pts[smpl+2];
        y2 = weights[wsmpl] * pts[smpl+3];

        mean_pts1_x += x1;
        mean_pts1_y += y1;
        mean_pts2_x += x2;
        mean_pts2_y += y2;

        avg_dist1 += sqrt (x1 * x1 + y1 * y1);
        avg_dist2 += sqrt (x2 * x2 + y2 * y2);
    }

    mean_pts1_x /= sample_number;
    mean_pts1_y /= sample_number;
    mean_pts2_x /= sample_number;
    mean_pts2_y /= sample_number;

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
