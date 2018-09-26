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
public:
    // Pure virtuals functions
    virtual void EstimateModel(const int * const sample, Model &model) = 0;
    virtual void EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) = 0;
    virtual float GetError(int pidx) = 0;
    virtual int SampleNumber()  = 0;

    // Setters of points set and model's parameters sufficiently sped up code
    // functions are virtual, they can be overwritten but not necessarily.
    virtual void setPoints (cv::InputArray input_points) {}
    virtual void setModelParameters (Model * const model) {}

    virtual int getNumberOfInliers (const Model * const model) {return 0;}

};

#endif //RANSAC_ESTIMATOR_H