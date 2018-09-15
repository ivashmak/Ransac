#ifndef RANSAC_HOMOGRAPHYESTIMATOR_H
#define RANSAC_HOMOGRAPHYESTIMATOR_H


#include "Estimator.h"

class HomographyEstimator : public Estimator{
public:
    void EstimateModel(cv::InputArray input_points, const int * const sample, Model &model) override {
        const int idx1 = sample[0];
        const int idx2 = sample[1];
        const int idx3 = sample[2];
        const int idx4 = sample[3];
    }

    void EstimateModelNonMinimalSample(cv::InputArray input_points, const int * const sample, int sample_size, Model &model) override {
        cv::Mat points = cv::Mat(sample_size, 2, CV_32FC1);
    }

    float GetError(int pidx) override {
        return 0;
    }

    int SampleNumber() override {
        return 4;
    }
};


#endif //RANSAC_HOMOGRAPHYESTIMATOR_H
