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
     * Principal Component Analysis
     */
    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat_<float> points (sample_size, 2);
        float *points_arr = (float *) points.data;
        float a, b, c;


        for (int i = 0; i < sample_size; i++) {
            float residualy = input_points[2*sample[i]+1] - (- this->a * input_points[2*sample[i]] - this->c)/(this->b+0.001);
            float residualx = input_points[2*sample[i]] - (- this->b * input_points[2*sample[i]+1] - this->c)/(this->a+0.001);

            float residual = sqrt (residualx * residualx + residualy * residualy);

            float w;
            if (residual <= 1) w = 1;
            else w = 1/residual;

//            std::cout << residualx << " " << residualy << '\n';
            std::cout << w << '\n';
            *points_arr++ = input_points[2*sample[i]] *w;
            *points_arr++ = input_points[2*sample[i]+1] *w;
        }



        return;

        cv::Mat covar, eigenvecs, means, eigenvals;
        // Find the covariance matrix
        cv::calcCovarMatrix(points, covar, means, CV_COVAR_NORMAL | CV_COVAR_ROWS);
        // Find the eigenvectors and eigenvalues of the covariance matrix
        cv::eigen (covar, eigenvals, eigenvecs);

        a = (float) -eigenvecs.at<double>(1,0);
        b = (float) -eigenvecs.at<double>(1,1);
        c = (float) (-a*means.at<double>(0) - b*means.at<double>(1));

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