// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

// Direct Linear Transformation
class DLt {
private:
    const float * const points;
public:
    DLt (const float * const points_) : points(points_) {}

    // minimal
    bool DLT4p (const int * const sample, cv::Mat &H);

    bool NormalizedDLT (const int * const sample, unsigned int sample_number, cv::Mat &H);
    bool NormalizedDLT (const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &H);

};
// non minimal
bool DLT (const float * const points, unsigned int sample_number, cv::Mat &H);
bool DLTEigen (const float * const points, unsigned int sample_number, cv::Mat &H);
bool DLTLeastSquares (const float * const points, unsigned int sample_number, cv::Mat &H);

void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, cv::Mat &T1, cv::Mat &T2);

void GetNormalizingTransformation (const float * const pts, cv::Mat& norm_points,
                                   const int * const sample, unsigned int sample_number, const float * const weights, cv::Mat &T1, cv::Mat &T2);


class DLTCov {
private:
    cv::Mat covA = cv::Mat_<float>(9, 9, float(0));
    float * covA_ptr;
    float a1[9] = {0, 0, -1, 0, 0, 0, 0, 0, 0};
    float a2[9] = {0, 0, 0, 0, 0, -1, 0, 0, 0};
    unsigned int points_size;
    bool * inlier_flags;
public:
    DLTCov (unsigned int points_size_) {
        points_size = points_size_;
        inlier_flags = (bool *) calloc (points_size, sizeof(bool));
    }
    bool computeH (const float * const points, const bool * const inlier_flags_, cv::Mat &H) {
        float x1, y1, x2, y2;
        unsigned int smpl;
        covA_ptr = (float *) covA.data;
        for (unsigned int i = 0; i < points_size; i++) {
            if (inlier_flags[i] != inlier_flags_[i]) {
                smpl = 4 * i;
                x1 = points[smpl];
                y1 = points[smpl + 1];

                x2 = points[smpl + 2];
                y2 = points[smpl + 3];

                a1[0] = -x1;
                a1[1] = -y1;
                a1[6] = x2 * x1;
                a1[7] = x2 * y1;
                a1[8] = x2;

                a2[3] = -x1;
                a2[4] = -y1;
                a2[6] = y2 * x1;
                a2[7] = y2 * y1;
                a2[8] = y2;

                /*
                 * if previous iteration point was inlier, but now it is not, so subtract
                 */
                if (inlier_flags[i] < inlier_flags_[i]) {
                    for (unsigned int j = 0; j < 9; j++) {
                        for (unsigned int z = j; z < 9; z++) {
                            covA_ptr[j * 9 + z] -= a1[j] * a1[z] + a2[j] * a2[z];
                        }
                    }
                } else {
                    // otherwise add to covariance
                    for (unsigned int j = 0; j < 9; j++) {
                        for (unsigned int z = j; z < 9; z++) {
                            covA_ptr[j * 9 + z] += a1[j] * a1[z] + a2[j] * a2[z];
                        }
                    }
                }

                // copy new inlier flags
                inlier_flags[i] = inlier_flags_[i];
            }
        }

        for (unsigned int row = 1; row < 9; row++) {
            for (unsigned int col = 0; col < row; col++) {
                covA_ptr[row*9+col] = covA_ptr[col*9+row];
            }
        }

        cv::Mat_<float> D, Vt;
        cv::eigen(covA, D, Vt);

        if (Vt.empty ()) {
            return false;
        }

        H = cv::Mat_<float>(Vt.row(Vt.rows-1).reshape (3,3));

        return true;
    }
};

#endif // RANSAC_DLT_H