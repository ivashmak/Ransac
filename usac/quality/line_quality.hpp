// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_LINEQUALITY_H
#define USAC_LINEQUALITY_H

#include "quality.hpp"
#include "../utils/math.hpp"

class LineQuality : Quality {
public:
    /*
     * Find number of inliers and calculate coefficient of determination as score of model
     * https://en.wikipedia.org/wiki/Coefficient_of_determination
     *
     */
    virtual void getScore (const float * const points, Score * score, const cv::Mat& model, int * inliers) override {

        estimator->setModelParameters(model);
        score->inlier_number = 0;

        // calculate coefficient of determination r^2
        float a = model.at<float>(0), b = model.at<float>(1), c = model.at<float>(2);
        float y, mean_y = 0, SS_tot = 0, SS_res = 0;
        unsigned int pt;

        for (unsigned int point = 0; point < points_size; point++) {
            if (estimator->GetError(point) < threshold) {
                inliers[score->inlier_number] = point;
                pt = 2*point;
                // second column in data points
                y = points[pt+1];
                mean_y += y;

                // The sum of squares of residuals
                // y - y_est = y - (-c -ax)/b = y + (c + ax)/b
                SS_res += fast_pow (y + (c + a*points[pt])/b, 2);
                score->inlier_number++;
            }
        }

        mean_y /= score->inlier_number;

//         The total sum of squares
        for (int i = 0; i < score->inlier_number; i++) {
            SS_tot += fast_pow(points[2*inliers[i]+1] - mean_y, 2) ;
        }

        score->score = score->inlier_number;

        // store coefficient of determination
        score->score = 1 - SS_res/SS_tot;
    }
};

#endif //USAC_LINEQUALITY_H
