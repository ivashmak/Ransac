#ifndef RANSAC_ESTIMATOR_H
#define RANSAC_ESTIMATOR_H

#include <iostream>
#include <cstdio>
#include <string>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

#include "../Sampler/Sampler.h"
#include "../Model.h"
#include "../TerminationCriteria.h"

class Estimator {
protected:
    float * params;

public:
    virtual void EstimateModel(cv::InputArray input_points, int *sample, Model &model) = 0;
    virtual void EstimateModelNonMinimalSample(cv::InputArray input_points, int *sample, int sample_size, Model &model) = 0;
    virtual float GetError(cv::InputArray input_points, int pidx, Model * const model) = 0;
    virtual int SampleNumber()  = 0;

    // speedups 1.5 times
    void setModelParametres (Model * const model) {
        params = (float *) model->returnDescriptor().data;
    }

};

#endif //RANSAC_ESTIMATOR_H