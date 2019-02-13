// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_DLT_H
#define RANSAC_DLT_H

#include "estimator.hpp"
#include <opencv2/core/mat.hpp>

namespace cv { namespace usac {
// Direct Linear Transformation
    class DLt {
    private:
        const float *const points;
    public:
        DLt(const float *const points_) : points(points_) {}

        // minimal
        bool DLT4p(const int *const sample, cv::Mat &H);

        bool NormalizedDLT(const int *const sample, unsigned int sample_number, cv::Mat &H);

    };

// non minimal
    bool DLT(const float *const points, unsigned int sample_number, cv::Mat &H);

    bool DLTEigen(const float *const points, unsigned int sample_number, cv::Mat &H);

    bool DLTLeastSquares(const float *const points, unsigned int sample_number, cv::Mat &H);

    void GetNormalizingTransformation(const float *const pts, cv::Mat &norm_points,
                                      const int *const sample, unsigned int sample_number, cv::Mat &T1, cv::Mat &T2);

}}

#endif // RANSAC_DLT_H