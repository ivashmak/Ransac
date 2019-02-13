// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include "../../precomp.hpp"
#include "../estimator.hpp"

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


/*
 * Assume small difference between covariance matrices of
 * previous model and current estimation. Or inliers didn't
 * change so much.
 *
 * So in current estimation we can use previous covariance
 * matrix with modifications:
 * 1. Substract if current point was inlier but now is not.
 * 2. Add if current point is inlier but previously was not.
 */
class DLTCov {
private:
    cv::Mat covA = cv::Mat_<float>(9, 9, float(0));
    float * covA_ptr;
    float a1[9] = {0, 0, -1, 0, 0, 0, 0, 0, 0};
    float a2[9] = {0, 0, 0, 0, 0, -1, 0, 0, 0};
    unsigned int points_size;
    bool * inlier_flags;
    const float * const points;
    Estimator * estimator;
    float threshold;
public:
    DLTCov (unsigned int points_size_, const float * const points_, Estimator * estimator_, float threshold_) : points (points_) {
        points_size = points_size_;
        estimator = estimator_;
        // allocate inliers flags in the beginning as all zeros
        inlier_flags = (bool *) calloc (points_size, sizeof(bool));
        threshold = threshold_;
    }

    bool computeH (const cv::Mat &H_, cv::Mat &H) {
        estimator->setModelParameters(H_);

        float x1, y1, x2, y2;
        unsigned int smpl;
        covA_ptr = (float *) covA.data;
        bool is_inlier_i;
        for (unsigned int i = 0; i < points_size; i++) {
            is_inlier_i = estimator->GetError(i) < threshold;

            if (inlier_flags[i] != is_inlier_i) {
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
                if (is_inlier_i) { // so inliers_flags[i] is not inlier
                    // otherwise add to covariance
                    for (unsigned int j = 0; j < 9; j++) {
                        for (unsigned int z = j; z < 9; z++) {
                            covA_ptr[j * 9 + z] += a1[j] * a1[z] + a2[j] * a2[z];
                        }
                    }
                } else {
                    // otherwise inliers_flags[i] is inliers
                    for (unsigned int j = 0; j < 9; j++) {
                        for (unsigned int z = j; z < 9; z++) {
                            covA_ptr[j * 9 + z] -= a1[j] * a1[z] + a2[j] * a2[z];
                        }
                    }
                }

                // copy new inlier flag
                inlier_flags[i] = is_inlier_i;
            }
        }

        // copy symmetric part
        for (unsigned int row = 1; row < 9; row++) {
            for (unsigned int col = 0; col < row; col++) {
                covA_ptr[row*9+col] = covA_ptr[col*9+row];
            }
        }
        //

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