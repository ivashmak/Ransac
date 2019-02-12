// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "estimator.hpp"

class Line2DEstimator : public Estimator {
protected:
    float a,b,c;
    const float * const points;
public:

    /*
     * @points: is matrix of size: number of points x 2
     * x1 y1
     * ...
     * xN yN
     */
    Line2DEstimator (cv::InputArray input_points) : points ((float *)input_points.getMat().data) {
        assert(!input_points.empty());
    }

    /*
     * (y1 - y2)x + (x2 - x1)y + (x1y2 - x2y1) = 0
     *                 x2 - x1                                      y1 - y2
     * b = --------------------------------        a = ----------------------------------
     *     sqrt ((x1 - x2)^2 + (y2 - y1)^2)            sqrt ((x1 - x2)^2 + (y2 - y1)^2)
     *
     *              x2 y1 - y2 x1
     * c = --------------------------------
     *     sqrt ((x1 - x2)^2 + (y2 - y1)^2)
     */
    unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        const int idx1 = sample[0];
        const int idx2 = sample[1];
        
        float a, b, c; // use global

        // Estimate the model parameters from the sample
        a = points[2*idx1+1] - points[2*idx2+1]; // tangent_y
        b = points[2*idx2] - points[2*idx1]; // tangent_x

        float mag = sqrt(a * a + b * b);
        a /= mag;
        b /= mag;
        c = (points[2*idx1] * points[2*idx2+1] - points[2*idx2] * points[2*idx1+1])/mag;

        // Set the model descriptor
        models[0]->setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
        return 1;
    }

    /*
     * Principal Component Analysis
     */
    bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, Model &model) override {

        /*
         * cov (i, j) = sum ((xi - mean_x) * (yi - mean_y)) =
         * sum (xi*yi - xi * mean_y - yi * mean_x + mean_x * mean_y) =
         * sum (xi*yi) - mean_y * sum (xi) - mean_x * sum (yi) + N * mean_x * mean_y
         *
         * Symmetric cov (i, j) = cov (j, i)
         * It is enough to calculate sum_x, sum_y, sum_xy, sum_x_sq and sum_y_sq, and means after.
         */

        float x, y, sum_x = 0, sum_y = 0, sum_xy, sum_x2 = 0, sum_y2 = 0, mean_x, mean_y;
        unsigned int smpl;
        for (unsigned int i = 0; i < sample_size; i++) {
            smpl = 2*sample[i];
            x = points[smpl];
            y = points[smpl+1];

            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
            sum_y2 += y * y;
        }
        mean_x = sum_x / sample_size;
        mean_y = sum_y / sample_size;

        cv::Mat_<float> cov (2,2);
        cov.at<float>(0, 0) = sum_x2 - 2 * sum_x * mean_x              + sample_size * mean_x * mean_x;
        cov.at<float>(0, 1) = sum_xy - sum_x * mean_y - sum_y * mean_x + sample_size * mean_x * mean_y;
        cov.at<float>(1, 1) = sum_y2 - 2 * sum_y * mean_y              + sample_size * mean_y * mean_y;
        cov.at<float>(1, 0) = cov.at<float>(0, 1);

        cv::Mat eigenvecs, eigenvals;
        cv::eigen (cov, eigenvals, eigenvecs);

        // in opencv eigen values has decreased order, so
        // optimal subspace is first eigen vector,
        // the equation of line is second eigen vector

        float a, b, c;
        // eigen vectors are normalized
        a = -eigenvecs.at<float>(1,0);
        b = -eigenvecs.at<float>(1,1);
        c = -a * mean_x - b * mean_y;

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
        return true;
    }

    /* Least Square Fitting
     * https://ch.mathworks.com/help/curvefit/least-squares-fitting.html
     * TODO:
     *     Make optinions of:
     *     1) Least Square Fitting.
     *     2) Weighted Least Square Fitting.
     */
    bool LeastSquaresFitting (const int * const sample, unsigned int sample_size, Model &model) override {
        float a = 0, b = 0, c;
        float x, y, x_mean = 0, y_mean = 0;
        unsigned int smpl;
        
        for (unsigned int i = 0; i < sample_size; i++) {
            smpl = 2*sample[i];
            x = points[smpl];
            y = points[smpl+1];
            x_mean += x;
            y_mean += y;
        }

        x_mean /= sample_size; y_mean /= sample_size;
        for (unsigned int i = 0; i < sample_size; i++) {
            smpl = 2*sample[i];
            x = points[smpl];
            y = points[smpl+1];
            
            a += (x-x_mean) * (y-y_mean);
            b += (x-x_mean) * (x-x_mean);
        }
        a = -a;
        c = -b*y_mean - a*x_mean;

        // normalize
        float mag = sqrt(a * a + b * b);
        a /= mag;
        b /= mag;
        c /= mag;

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
        return true;
    }

    /*
     * |ax + by + c|, where ||(a b)|| = 1
     */
    inline float GetError(unsigned int pidx) override {
        return fabsf (a * points[2*pidx] + b * points[2*pidx+1] + c);
    }

    int SampleNumber() override {
        return 2;
    }

    void setModelParameters (const cv::Mat& model) override {
        a = model.at<float>(0); b =  model.at<float>(1); c =  model.at<float>(2);
    }
};



#endif //RANSA  C_LINE2DESTIMATOR_H