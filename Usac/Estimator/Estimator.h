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
#include "../TerminationCriteria/StandardTerminationCriteria.h"

class Estimator {
public:
    // Pure virtuals functions
    // return number of models
    // or Model **& models
    virtual unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) = 0;
    
    virtual bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, Model &model) = 0;
    virtual bool LeastSquaresFitting (const int * const sample, unsigned int sample_size, Model &model) {
        return EstimateModelNonMinimalSample(sample, sample_size, model);
    }

    virtual bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, const float * const weights, Model &model) {
        std::cout << "NOT IMPLEMENTED EstimateModelNonMinimalSample in estimator\n";
    }

//    virtual bool WeightedEstimateModelNonMinimalSample(const int * const sample, int sample_size, Model &model) = 0;
//    virtual bool WeightedLeastSquaresFitting (const int * const sample, int sample_size, Model &model) {
//        return WeightedEstimateModelNonMinimalSample(sample, sample_size, model);
//    }

    virtual float GetError(unsigned int pidx) = 0;
    virtual void GetError(float * weights, float threshold, int * inliers, unsigned int * inliers_size) {
        std::cout << "NOT IMPLEMENTED GetError (float * weights) in estimator\n";
    };
    virtual int SampleNumber() = 0;

    // Setters of points set and model's parameters sufficiently sped up code
    // functions are virtual, they can be overwritten but not necessarily.
    /*
     * These function should be avoided in Ransac. Efficiently to use global 
     * input points and descriptors as private class members.
     */
    virtual void setModelParameters (const cv::Mat& model) {
        std::cout << "NOT IMPLEMENTED FUNCTION setModelParameters!\n";
    }

    virtual void getModelbyCameraMatrix (const cv::Mat &K1, const cv::Mat &K2, const cv::Mat &InModel, cv::Mat &OutModel) {
        std::cout << "NOT IMPLEMENTED FUNCTION getModelbyCameraMatrix!\n";
    }

    /*
     * Fundamental Estimator
     */
    virtual bool isValid (const Model * const model) {
        return true;
    }
};

#endif //RANSAC_ESTIMATOR_H