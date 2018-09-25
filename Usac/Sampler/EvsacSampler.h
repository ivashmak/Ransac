#ifndef USAC_EVSACSAMPLER_H
#define USAC_EVSACSAMPLER_H

#include "../../RandomGenerator/UniformRandomGenerator.h"

#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>

#include "Sampler.h"
#include <theia/theia.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/flann/flann.hpp>

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

// The matrix containing k L2 sorted distances in ascending order. The matrix has
// num. of query features as rows and k columns.

class EvsacSampler : public Sampler {
protected:
    std::unique_ptr<std::discrete_distribution<>> correspondence_sampler_;
    std::mt19937 rng_;
public:

    //  The idea of EVSAC is to model the statistics of the minimum distances
    // computed when using the Nearest-Neighbor feature matcher.
    // find knn and their sorted distances.
    EvsacSampler (cv::InputArray input_points, unsigned int num_queries, unsigned int knn, unsigned int sample_size, bool reset_time = true) {
        assert (!input_points.empty());

        // init random generator
        RandomGenerator * randomGenerator = new UniformRandomGenerator;
        if (reset_time) randomGenerator->resetTime();

        this->sample_size = sample_size;

        std::random_device rand_dev;
        rng_.seed(rand_dev());

        int points_size = input_points.size().width;

        // fitting_method:  The fitting method MLE or QUANTILE_NLS (see statx doc).
        //   The recommended fitting method is the MLE estimation.
        theia::FittingMethod fitting_method = theia::QUANTILE_NLS;

        // The threshold used to decide correct or incorrect
        // matches/correspondences. The recommended value is 0.65.
        double predictor_threshold = 0.65;

        // Parameters of the mixture of distributions (Gamma + GEV).
        theia::EvsacSampler<Eigen::Vector2d>::MixtureModelParams mixture_model_params;


        /*
         * Create matrix that for each num_queries points has k nearest neughbors sorted distances
         */
        Eigen::MatrixXd sorted_distances_(num_queries, knn-1);

        int *random_samples = new int[num_queries];
        randomGenerator->generateUniqueRandomSet(random_samples, num_queries, 0, points_size-1);

        cv::Mat sorted_dists, indicies, points = cv::Mat(points_size, 2, CV_32F, input_points.getMat().data);
        cv::flann::LinearIndexParams flannIndexParams;
        cv::flann::Index flannIndex (cv::Mat(points).reshape(1), flannIndexParams);

        for (int i = 0; i < num_queries; i++) {
            flannIndex.knnSearch(cv::Mat(1, 2, CV_32F, &points.at<cv::Point_<float>>(random_samples[i])),
                                 indicies, sorted_dists, knn);

            for (int j = 0; j < knn-1; j++) {
                sorted_distances_(i, j) = sorted_dists.at<float>(j+1);
            }
        }

        std::vector<float> probabilities, sampling_weights;

        // calculates weight for each query point
        if (!theia::EvsacSampler<Eigen::Vector2d>::CalculateMixtureModel(
                sorted_distances_,
                predictor_threshold,
                fitting_method,
                &mixture_model_params,
                &probabilities,
                &sampling_weights)){

            std::cout << "Calculation of Mixture Model failed" << '\n';
            exit (0);
        }

        initializeSampler(sampling_weights);
    }


    // Initialize correspondence sampler.
    void initializeSampler (std::vector<float> sampling_weights) {
        //std::discrete_distribution produces random integers on the interval [0, n), where the
        // probability of each individual integer i is defined as w(i)/S
        // that is the weight of the ith integer divided by the sum of all n weights.

        // MSVC has one of the constructors missing. See
        // http://stackoverflow.com/questions/21959404/initialising-stddiscrete-distribution-in-vs2013

        #if defined(_MSC_VER) & _MSC_VER < 1900
            std::size_t i(0);
            correspondence_sampler_.reset(new std::discrete_distribution<int>(sampling_weights.size(),
                                          0.0,  // dummy!
                                          0.0,  // dummy!
                                          [&sampling_weights, &i](double) {
                                            auto w = sampling_weights[i];
                                            ++i;
                                            return w;
                                          }));
        #else
            correspondence_sampler_.reset(new std::discrete_distribution<int>(
                sampling_weights.begin(), sampling_weights.end()));
        #endif
    }


    void generateSample (int *sample) override {
        for (int i = 0; i < sample_size; i++) {
            sample[i] = (*correspondence_sampler_)(rng_);
            for (int j = i - 1; j >= 0; j--) {
                if (sample[i] == sample[j]) {
                    i--;
                }
            }
        }
    }

    bool isInit () override {
        return true;
    }
};

#endif //USAC_EVSACSAMPLER_H
