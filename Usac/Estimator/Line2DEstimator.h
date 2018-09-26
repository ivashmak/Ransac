#ifndef RANSAC_LINE2DESTIMATOR_H
#define RANSAC_LINE2DESTIMATOR_H

#include "Estimator.h"

class Line2DEstimator : public Estimator {
protected:
    float a,b,c;
    cv::Point_<float> *input_points;

    // new
//    cv::Mat points;
//    cv::Mat mdl;
public:

    void EstimateModel(const int * const sample, Model &model) override {
        const int idx1 = sample[0];
        const int idx2 = sample[1];

        // Estimate the model parameters from the sample
        float b = -(input_points[idx2].x - input_points[idx1].x); // tangent_x
        float a = input_points[idx2].y - input_points[idx1].y; // tangent_y

        float mag = (float) (sqrt(a * a + b * b));
        a /= mag;
        b /= mag;
        float c = (input_points[idx2].x * input_points[idx1].y - input_points[idx2].y * input_points[idx1].x)/mag;

        // Set the model descriptor
        model.setDescriptor((cv::Mat_<float>(1,3) <<  a, b, c));
    }

    /*
     * Least Square Fitting
     * Principal Component Analysis
     */
    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat_<float> points (sample_size, 2);
        float a, b, c;

        for (int i = 0; i < sample_size; i++) {
            points.at<float>(i,0) = input_points[sample[i]].x;
            points.at<float>(i,1) = input_points[sample[i]].y;
        }

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
        std::cout << input_points[pidx] << '\n';

        return fabsf (a * input_points[pidx].x + b * input_points[pidx].y + c);
    }

    int SampleNumber() override {
        return 2;
    }

    void setModelParameters (Model * const model) override {
        auto *params = (float *) model->returnDescriptor().data;
        a = params[0]; b = params[1]; c = params[2];

//        mdl = (cv::Mat_<float> (2,1) << a,b);
    }

    void setPoints (cv::InputArray input_pts) override {
        input_points = (cv::Point_<float> *) input_pts.getMat().data;

//        points = cv::Mat (input_pts.size().width, 2, CV_32FC1, input_pts.getMat().data);
    }

    /*int getNumberOfInliers (const Model * const model) override {

        cv::Mat pts_ = abs(points * mdl + c) - model->threshold;
        int inl_num = 0;
        for (int i = 0; i < pts_.rows; i++) {
            if (pts_.at<float>(i) < 0) inl_num++;
        }

        return inl_num;
    }*/

};



#endif //RANSA  C_LINE2DESTIMATOR_H