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
//#include <opencv2/xfeatures2D.hpp>
#include <opencv2/highgui.hpp>

#include "../Sampler/Sampler.h"
#include "../Model.h"
#include "../TerminationCriteria.h"

class Estimator {
public:
    // Pure virtuals functions
    // return number of models
    // or Model **& models
    virtual int EstimateModel(const int * const sample, std::vector<Model*>& models) = 0;
    
    virtual bool EstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) = 0;
    virtual void LeastSquaresFitting (const int * const sample, int sample_size, Model &model) {
        EstimateModelNonMinimalSample(sample, sample_size, model);
    }
    
    virtual float GetError(int pidx) = 0;
    virtual int SampleNumber()  = 0;

    // Setters of points set and model's parameters sufficiently sped up code
    // functions are virtual, they can be overwritten but not necessarily.
    /*
     * These function should be avoided in Ransac. Efficiently to use global 
     * input points and descriptors as private class members.
     */
    virtual void setPoints (cv::InputArray input_points) {}
    virtual void setModelParameters (Model * const model) {}

    virtual int getNumberOfInliers (const Model * const model) {return 0;}

    /*
     * Especially for Fundamental Estimator
     */
    virtual bool isValid (const Model * const model) {
        return true;
    }
};

#endif //RANSAC_ESTIMATOR_H