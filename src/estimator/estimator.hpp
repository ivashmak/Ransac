// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_ESTIMATOR_H
#define RANSAC_ESTIMATOR_H

#include "../precomp.hpp"

#include "../sampler/sampler.hpp"
#include "../model.hpp"
#include "../termination_criteria/standard_termination_criteria.hpp"

class Estimator {
public:
    virtual ~Estimator () = default;

    // minimal model estimation
    virtual unsigned int EstimateModel(const int * const sample, std::vector<Model*>& models) = 0;
    
    virtual bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, Model &model) = 0;
    virtual bool LeastSquaresFitting (const int * const sample, unsigned int sample_size, Model &model) {
        return EstimateModelNonMinimalSample(sample, sample_size, model);
    }

    virtual bool EstimateModelNonMinimalSample(const int * const sample, unsigned int sample_size, const float * const weights, Model &model) {
        std::cout << "NOT IMPLEMENTED EstimateModelNonMinimalSample in estimator\n";
    }

    virtual float GetError(unsigned int pidx) = 0;
    virtual void GetError(float * weights, float threshold, int * inliers, unsigned int * inliers_size) {
        std::cout << "NOT IMPLEMENTED GetError (float * weights) in estimator\n";
    };
    virtual int SampleNumber() = 0;

    virtual void setModelParameters (const cv::Mat& model) = 0;

    virtual bool isModelValid (const cv::Mat &model, const int * const sample) {
        return true;
    }
};

#endif //RANSAC_ESTIMATOR_H