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
    float a,b,c;
public:
    virtual void EstimateModel(const int * const sample, Model &model) = 0;
    virtual void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) = 0;
    virtual float GetError(int pidx) = 0;
    virtual int SampleNumber()  = 0;

    // speedups 1.5 times
    void setModelParametres (Model * const model) {
        params = (float *) model->returnDescriptor().data;
        a = params[0]; b = params[1]; c = params[2];
    }

};

#endif //RANSAC_ESTIMATOR_H