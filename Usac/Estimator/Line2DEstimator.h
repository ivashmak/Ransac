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

    int EstimateModel(const int * const sample, Model **& models) override {
        const int idx1 = sample[0];
        const int idx2 = sample[1];

        // Estimate the model parameters from the sample
        float b = -(input_points[2*idx2] - input_points[2*idx1]); // tangent_x
        float a = input_points[2*idx2+1] - input_points[2*idx1+1]; // tangent_y

        float mag = (float) (sqrt(a * a + b * b));
        a /= mag;
        b /= mag;
        float c = (input_points[2*idx2] * input_points[2*idx1+1] - input_points[2*idx2+1] * input_points[2*idx1])/mag;

        // Set the model descriptor
        models[0]->setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));

        return 1;
    }

    /*
     * Least Square Fitting
     * https://ch.mathworks.com/help/curvefit/least-squares-fitting.html
     *
     * Principal Component Analysis
     */
    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
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

        a = (float) -eigenvecs.at<double>(1,0);
        b = (float) -eigenvecs.at<double>(1,1);
        c = (float) (-a*means.at<double>(0) - b*means.at<double>(1));

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
    }

    void LeastSquaresFitting (const int * const sample, int sample_size, Model &model) override {
        float x, y, x_mean = 0, y_mean = 0;
        float xx_mean = 0, yy_mean = 0, varx, vary;
        float *residuals_x = new float[sample_size];
        float *residuals_y = new float[sample_size];
        
        for (int i = 0; i < sample_size; i++) {
            x = input_points[2*sample[i]];
            y = input_points[2*sample[i]+1];
            x_mean += x;
            y_mean += y;
            xx_mean += x*x;
            yy_mean += y*y;
            // residuals_y[i] = y - (-x*this->a - this->c)/this->b;
            // residuals_x[i] = x - (-y*this->b - this->c)/this->a;
        }

        float MADx = 0, MADy;
        for (int i = 0; i < sample_size; i++) {
            x = input_points[2*sample[i]];
            y = input_points[2*sample[i]+1];
            MADx += fabsf (x - x_mean);
            MADy += fabsf (y - y_mean);   
        }

        x_mean /= sample_size;
        y_mean /= sample_size;

        xx_mean /= sample_size;
        yy_mean /= sample_size;

        MADx /= sample_size;
        MADy /= sample_size;

        varx = xx_mean - x_mean * x_mean;
        vary = yy_mean - y_mean * y_mean;
        float w = 1/(sqrt (varx*varx + vary*vary));
        
        // std::cout << "weight " << w << '\n';

        a = 0;
        b = 0;
        float robust_w;
        float residual_adj, residual_adj_x, residual_adj_y, y_pred, K = 4.685;
        
        // float s = MAD/0.6745;
        // float s = MADx/0.6745;
        float s = MADy/0.6745;
        
        for (int i = 0; i < sample_size; i++) {
            x = input_points[2*sample[i]];
            y = input_points[2*sample[i]+1];
            
            // residual_adj_x = residuals_x[i]/sqrt(1-residuals_x[i]/x);
            residual_adj_y = (y - y_mean)/sqrt(1-(y-y_mean)/y);
            // residual_adj = sqrt (residual_adj_x * residual_adj_x + residual_adj_y * residual_adj_y);
            
            float u = residual_adj_y/(K*s);
            if (u >= 1) {
                robust_w = 0;
            } else {
                robust_w = (1 - u*u) * (1 - u*u);
            }   
            // std::cout << u << " " << robust_w << '\n';

            // w = 1/(y - (-x*this->a - this->c)/this->b);
            w = 1;
            a += w*(x-x_mean) * (y-y_mean);
            b += w*(x-x_mean) * (x-x_mean);
        }
        a = -a;
        c = -b*y_mean - a*x_mean;

        float mag = (float) (sqrt(a * a + b * b));
        a /= mag;
        b /= mag;
        c /= mag;

        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
    }
    
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