#ifndef HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H
#define HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H


#include "Estimator.h"

class HomographyEstimator : public Estimator{
public:
    void EstimateModel(cv::InputArray input_points, int *sample, Model &model) {}
    void EstimateModelNonMinimalSample(cv::InputArray input_points, int *sample, int sample_size, Model &model) {}
    float GetError(cv::InputArray input_points, int pidx, Model * model) {
        return 0;
    }
    int SampleNumber() {
        return 4;
    }
};


#endif //HOMOGRAPHYFITTING_HOMOGRAPHYESTIMATOR_H
