#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"

class Line2DEstimator : public Estimator {
protected:
    float a,b,c;
    const float * const input_points;
public:
    Line2DEstimator (cv::InputArray input_array_points) : input_points ((float *)input_array_points.getMat().data) {
        assert(!input_array_points.empty());
    }

    /*
     *                 x1 - x2                                      y2 - y1
     * a = --------------------------------        b = ----------------------------------
     *     sqrt ((x1 - x2)^2 + (y2 - y1)^2)            sqrt ((x1 - x2)^2 + (y2 - y1)^2)
     *
     *              x2 y1 - y2 x1
     * c = --------------------------------
     *     sqrt ((x1 - x2)^2 + (y2 - y1)^2)
     */
    int EstimateModel(const int * const sample, std::vector<Model*>& models) override {
        const int idx1 = sample[0];
        const int idx2 = sample[1];
        
        float a, b, c; // use global

        // Estimate the model parameters from the sample
        b = -(input_points[2*idx2] - input_points[2*idx1]); // tangent_x
        a = input_points[2*idx2+1] - input_points[2*idx1+1]; // tangent_y

        float mag = sqrt(a * a + b * b);
        a /= mag;
        b /= mag;
        c = (input_points[2*idx2] * input_points[2*idx1+1] - input_points[2*idx2+1] * input_points[2*idx1])/mag;

        // Set the model descriptor
        models[0]->setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
        return 1;
    }

    /*
     * Principal Component Analysis
     */
    bool EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat_<float> points (sample_size, 2);
        float *points_arr = (float *) points.data;
        float a, b, c;

       for (int i = 0; i < sample_size; i++) {
           *points_arr++ = input_points[2*sample[i]];
           *points_arr++ = input_points[2*sample[i]+1];
       }

        cv::Mat covar, eigenvecs, means, eigenvals;
        // Find the covariance matrix
        cv::calcCovarMatrix(points, covar, means, 1 | 8); //CV_COVAR_NORMAL | CV_COVAR_ROWS
        // Find the eigenvectors and eigenvalues of the covariance matrix
        cv::eigen (covar, eigenvals, eigenvecs);

        std::cout << covar << "\n\n";
        std::cout << eigenvals << "\n\n";
        std::cout << eigenvecs << "\n\n";

        a = (float) -eigenvecs.at<double>(1,0);
        b = (float) -eigenvecs.at<double>(1,1);
        c = (float) (-a*means.at<double>(0) - b*means.at<double>(1));

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
        return true;
    }

    /* Least Square Fitting
     * https://ch.mathworks.com/help/curvefit/least-squares-fitting.html
     * TODO:
     *     Make optinions of:
     *     1) Least Square Fitting.
     *     2) Weighted Least Square Fitting.
     *     3) Robust Least Squre Fitting.
     */
    void LeastSquaresFitting (const int * const sample, int sample_size, Model &model) override {
        float a = 0, b = 0, c;
        float x, y, x_mean = 0, y_mean = 0;
        // float xx_mean = 0, yy_mean = 0, varx, vary;
        unsigned int smpl;
        for (unsigned int i = 0; i < sample_size; i++) {
            smpl = 2*sample[i];
            x = input_points[smpl];
            y = input_points[smpl+1];
            x_mean += x;
            y_mean += y;
            // xx_mean += x*x;
            // yy_mean += y*y;
        }

        // float MADx = 0, MADy;
        // for (int i = 0; i < sample_size; i++) {
        //     x = input_points[2*sample[i]];
        //     y = input_points[2*sample[i]+1];
        //     MADx += fabsf (x - x_mean);
        //     MADy += fabsf (y - y_mean);   
        // }

        x_mean /= sample_size; y_mean /= sample_size;

        // xx_mean /= sample_size; yy_mean /= sample_size;

        // MADy /= sample_size;

        // varx = xx_mean - x_mean * x_mean;
        // vary = yy_mean - y_mean * y_mean;
        // float w = 1/(sqrt (varx*varx + vary*vary));
        
        // float robust_w, residual_adj, residual_adj_x, residual_adj_y, y_pred, K = 4.685;
        // float s = MADy/0.6745;
        
        for (unsigned int i = 0; i < sample_size; i++) {
            smpl = 2*sample[i];
            x = input_points[smpl];
            y = input_points[smpl+1];
            
            // residual_adj_x = residuals_x[i]/sqrt(1-residuals_x[i]/x);
            // residual_adj_y = (y - y_mean)/sqrt(1-(y-y_mean)/y);

            // float u = residual_adj_y/(K*s);
            // if (u >= 1) {
            //     robust_w = 0;
            // } else {
            //     robust_w = (1 - u*u) * (1 - u*u);
            // }   

            // w = 1/(y - (-x*this->a - this->c)/this->b);
            a += (x-x_mean) * (y-y_mean);
            b += (x-x_mean) * (x-x_mean);
        }
        a = -a;
        c = -b*y_mean - a*x_mean;

        float mag = (float) (sqrt(a * a + b * b));
        a /= mag;
        b /= mag;
        c /= mag;

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
    }

    /*
     * |ax + by + c|
     */
    inline float GetError(int pidx) override {
        return fabsf (a * input_points[2*pidx] + b * input_points[2*pidx+1] + c);
    }

    int SampleNumber() override {
        return 2;
    }

    void setModelParameters (Model * const model) override {
        auto *params = (float *) model->returnDescriptor().data;
        a = params[0]; b = params[1]; c = params[2];

    }


};



#endif //RANSA  C_LINE2DESTIMATOR_H