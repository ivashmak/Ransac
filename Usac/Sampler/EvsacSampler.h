#ifndef USAC_EVSACSAMPLER_H
#define USAC_EVSACSAMPLER_H

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

struct MixtureModelParams {
    // Gamma Parameters (for correspondences modeled to be correct).
    double k;      // shape.
    double theta;  // scale.

    // GEV Parameters (for correspondences estimated to be incorrect).
    double xi;     // tail.
    double sigma;  // scale.
    double mu;     // location.

    // Inlier ratio (Mixture parameter).
    double inlier_ratio;  // Estimated inlier ratio.
};

class EvsacSampler : public Sampler {
protected:
    std::unique_ptr<std::discrete_distribution<>> correspondence_sampler_;
    std::mt19937 rng_;
    MixtureModelParams mixture_model_params;
    theia::FittingMethod fitting_method;
    double predictor_threshold;
public:
    EvsacSampler (cv::InputArray input_points, int knn) {
        // init random generator
        srand (time(NULL));
        std::random_device rand_dev;
        rng_.seed(rand_dev());


        // get initial point
        int total_points = input_points.size().width;
        cv::Mat points = cv::Mat(total_points, 2, CV_32F, input_points.getMat().data);
        int * init_point = new int[1];
        init_point[0] = rand() % total_points;
        cv::Point_<float> initial_point = points.at<cv::Point_<float>>(init_point[0]);


        //  The idea of EVSAC is to model the statistics of the minimum distances
        // computed when using the Nearest-Neighbor feature matcher.
        // find knn and their sorted distances.
        cv::Mat query, sorted_distances, indicies;
        cv::flann::LinearIndexParams flannIndexParams;
        cv::flann::Index flannIndex (cv::Mat(points).reshape(1), flannIndexParams);
        query = cv::Mat(1, 2, CV_32F, &initial_point);
        flannIndex.knnSearch(query, indicies, sorted_distances, knn);
        std::cout << "indicies = " << indicies << '\n';
        std::cout << "sorted_distances = " << sorted_distances << "\n\n";


        // fitting_method:  The fitting method MLE or QUANTILE_NLS (see statx doc).
        //   The recommended fitting method is the MLE estimation.
        this->fitting_method = theia::MLE;


        // The threshold used to decide correct or incorrect
        // matches/correspondences. The recommended value is 0.65.
        this->predictor_threshold = 0.65;


        std::vector<float> probabilities;
        std::vector<float> sampling_weights;

        Eigen::MatrixXd sorted_distances_(2,2);

//        theia::EvsacSampler::CalculateMixtureModel(sorted_distances_,
//                                                   this->predictor_threshold,
//                                                   this->fitting_method,
//                                                   &this->mixture_model_params,
//                                                   &probabilities,
//                                                   &sampling_weights);


        initializeSampler(sampling_weights);
    }



    // Initialize correspondence sampler.
    void initializeSampler (std::vector<float> sampling_weights) {
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



//    bool EvsacSampler<Datum>::FitGamma(
//            const std::vector<double>& predicted_correct_correspondences_distances,
//            MixtureModelParams* mixture_model_parameters) {
//        const bool gam_success = statx::distributions::gammafit(
//                predicted_correct_correspondences_distances,
//                &CHECK_NOTNULL(mixture_model_parameters)->k,
//                &mixture_model_parameters->theta);
//        VLOG(2) << "Gamma distribution: k=" << mixture_model_parameters->k
//                << " theta=" << mixture_model_parameters->theta
//                << " flag: " << gam_success;
//        return gam_success;
//    }

    void getSample (int *sample, int npoints, int total_points) {
        std::vector<int> random_numbers;
        for (int i = 0; i < npoints; i++) {
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
