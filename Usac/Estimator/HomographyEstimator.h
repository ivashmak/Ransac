#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H

#include "Estimator.h"
#include "../DLT/NormalizedDLT.h"

class HomographyEstimator : public Estimator{
private:
    cv::Mat H;
    cv::Mat H_inv;
    // K images x (N x 3) points
//    std::vector<cv::Mat> set_points;
    float * points;
//    float *samples1_ptr, *samples2_ptr;

public:

    void setPoints (cv::InputArray input_points) override {
        assert(!input_points.empty());

        cv::Mat_<float> pts;
        cv::hconcat(input_points.getMat(0), input_points.getMat(1), pts);

        // Easy way to point Mat to float array does not work properly
//        points = (float *) pts.data;

//        std::cout << pts << "\n\n";

        points = new float [4*pts.rows];
        for (int i = 0; i < pts.rows; i++) {
            points[4*i] = pts.at<float>(i, 0);
            points[4*i+1] = pts.at<float>(i, 1);
            points[4*i+2] = pts.at<float>(i, 2);
            points[4*i+3] = pts.at<float>(i, 3);
        }

    }

    void setModelParametres (Model * const model) override {
        H = cv::Mat_<float>(model->returnDescriptor());
        H_inv = H.inv();
    }

    void EstimateModel(const int * const sample, Model &model) override {
        cv::Mat H;
        cv::Mat_<float>  samples1(4,2), samples2(4,2);

        float * samples1_ptr = (float *) samples1.data;
        float * samples2_ptr = (float *) samples2.data;

        for (int i = 0; i < 4; i++) {
            (*samples1_ptr++) = points[4*sample[i]];
            (*samples1_ptr++) = points[4*sample[i]+1];

            (*samples2_ptr++) = points[4*sample[i]+2];
            (*samples2_ptr++) = points[4*sample[i]+3];
        }

        DLT(samples1, samples2, H);

        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) override {
        cv::Mat H;
        cv::Mat_<float>  samples1(sample_size,2), samples2(sample_size,2);
        float * samples1_ptr = (float *) samples1.data;
        float * samples2_ptr = (float *) samples2.data;

        for (int i = 0; i < sample_size; i++) {
            (*samples1_ptr++) = points[4*sample[i]];
            (*samples1_ptr++) = points[4*sample[i]+1];

            (*samples2_ptr++) = points[4*sample[i]+2];
            (*samples2_ptr++) = points[4*sample[i]+3];
        }

        NormalizedDLT(samples1, samples2, H);

        this->H = H;
        this->H_inv = H.inv();

        model.setDescriptor(H);
    }

    float GetError(int pidx) override {
        float error = 0;
        cv::Mat_<float> corr1_pt;
        corr1_pt = (cv::Mat_<float>(3,1) << points[4*pidx], points[4*pidx+1], 1);
        cv::Mat_<float> corr2_pt;
        corr2_pt = (cv::Mat_<float>(3,1) << points[4*pidx+2], points[4*pidx+3], 1);

//        std::cout << corr1_pt << '\n';
//        std::cout << corr2_pt << '\n';

        // 3x3 * 3x1
        cv::Mat est_pt_on_corr2 = H * corr1_pt;
        cv::Mat est_pt_on_corr1 = H_inv * corr2_pt;

        est_pt_on_corr1 /= est_pt_on_corr1.at<float>(2);
        est_pt_on_corr2 /= est_pt_on_corr2.at<float>(2);

        // error = d(p(i)H, p'(i)) + d(p(i), p'(i)H^-1)
        error += sqrt ((corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) * (corr2_pt.at<float>(0) - est_pt_on_corr2.at<float>(0)) +
                       (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)) * (corr2_pt.at<float>(1) - est_pt_on_corr2.at<float>(1)));

        error += sqrt ((corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) * (corr1_pt.at<float>(0) - est_pt_on_corr1.at<float>(0)) +
                       (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)) * (corr1_pt.at<float>(1) - est_pt_on_corr1.at<float>(1)));

        return error/2;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H