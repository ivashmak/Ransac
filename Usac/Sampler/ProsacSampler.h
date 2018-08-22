#ifndef USAC_PROSACSAMPLER_H
#define USAC_PROSACSAMPLER_H

#include <cmath>
#include <vector>
#include "Sampler.h"
#include "../Model.h"

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

class ProsacSampler : public Sampler {
protected:
    Model * model;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;
    int kth_sample_number_;
    int ransac_convergence_iterations_;

public:
    ProsacSampler () {
        srand (time(NULL));
        std::random_device rand_dev;
        generator = std::mt19937(rand_dev());

        kth_sample_number_ = 1;
        ransac_convergence_iterations_ = 10000;
    }

    // Samples the input variable data and fills the vector subset with the prosac
    // samples.
    // NOTE: This assumes that data is in sorted order by quality where data[i] is
    // of higher quality than data[j] for all i < j.
    void getSample (int *sample, int npoints, int total_points) {

        double t_n = ransac_convergence_iterations_;
        int n = npoints;
        // From Equations leading up to Eq 3 in Chum et al.
        for (int i = 0; i < npoints; i++) {
            t_n *= (double) (n - i) / (total_points - i);
        }

        double t_n_prime = 1.0;
        // Choose min n such that T_n_prime >= t (Eq. 5).
        for (int t = 1; t <= kth_sample_number_; t++) {
            if (t > t_n_prime && n < total_points) {
                double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - npoints);
                t_n_prime += ceil(t_n_plus1 - t_n);
                t_n = t_n_plus1;
                n++;
            }
        }


        if (t_n_prime < kth_sample_number_) {
            // Randomly sample m data points from the top n data points.
            std::vector<int> random_numbers;
            for (int i = 0; i < npoints; i++) {
                // Generate a random number that has not already been used.
                int rand_number;
                distribution = std::uniform_int_distribution<int>(0, n-1);

                while (std::find(random_numbers.begin(),
                                 random_numbers.end(),
                                 (rand_number = distribution(generator))) !=
                       random_numbers.end()) {
                }

                random_numbers.push_back(rand_number);

                // Push the *unique* random index back.
                sample[i] = rand_number;

            }

        } else {
            std::vector<int> random_numbers;
            // Randomly sample m-1 data points from the top n-1 data points.
            for (int i = 0; i < npoints - 1; i++) {
                // Generate a random number that has not already been used.
                int rand_number;
                distribution = std::uniform_int_distribution<int>(0, n-2);

                while (std::find(random_numbers.begin(),
                                 random_numbers.end(),
                                 (rand_number = distribution(generator))) !=
                       random_numbers.end()) {
                }
                random_numbers.push_back(rand_number);

                // Push the *unique* random index back.
                sample[i] = rand_number;
            }
            // Make the last point from the nth position.
            sample[npoints-1] = n;
        }

        this->kth_sample_number_++;

    }
};

#endif //USAC_PROSACSAMPLER_H
