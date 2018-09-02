#ifndef USAC_EVSACSAMPLER_H
#define USAC_EVSACSAMPLER_H

#include "../RandomGenerator/UniformRandomGenerator.h"

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
    EvsacSampler (cv::InputArray input_points, int num_q, int knn, int sample_size, bool reset_time = true) {
        // init random generator
        if (reset_time) randomGenerator->resetTime();

        this->sample_size = sample_size;

        std::random_device rand_dev;
        rng_.seed(rand_dev());

        int total_points = input_points.size().width;

        // fitting_method:  The fitting method MLE or QUANTILE_NLS (see statx doc).
        //   The recommended fitting method is the MLE estimation.
        theia::FittingMethod fitting_method = theia::QUANTILE_NLS;

        // The threshold used to decide correct or incorrect
        // matches/correspondences. The recommended value is 0.65.
        double predictor_threshold = 0.65;

        // Parameters of the mixture of distributions (Gamma + GEV).
        theia::EvsacSampler<Eigen::Vector2d>::MixtureModelParams mixture_model_params;

        Eigen::MatrixXd sorted_distances_(num_q, knn-1);

        int *r_samples = new int[num_q];
        randomGenerator = new UniformRandomGenerator ;
        randomGenerator->resetGenerator(0, total_points-1);
        randomGenerator->generateUniqueRandomSample(r_samples, num_q);

        cv::Mat sorted_dists, query, indicies, points = cv::Mat(total_points, 2, CV_32F, input_points.getMat().data);
        cv::flann::LinearIndexParams flannIndexParams;
        cv::flann::Index flannIndex (cv::Mat(points).reshape(1), flannIndexParams);

        cv::Point_<float> r_point;
        for (int i = 0; i < num_q; i++) {
            r_point = points.at<cv::Point_<float>>(r_samples[i]);
            query = cv::Mat(1, 2, CV_32F, &r_point);
            flannIndex.knnSearch(query, indicies, sorted_dists, knn);

            for (int j = 0; j < knn-1; j++) {
                sorted_distances_(i, j) = sorted_dists.at<float>(j+1);
            }
        }

        std::vector<float> probabilities;
        std::vector<float> sampling_weights;

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
        std::vector<int> random_numbers;
        for (int i = 0; i < sample_size; i++) {
            int rand_number;
            // Generate a random number that has not already been used.
            while (std::find(random_numbers.begin(),
                             random_numbers.end(),
                             (rand_number = (*correspondence_sampler_)(rng_))) !=
                   random_numbers.end()) {
            }

            random_numbers.push_back(rand_number);
            sample[i] = rand_number;
        }

    }
};

#endif //USAC_EVSACSAMPLER_H
