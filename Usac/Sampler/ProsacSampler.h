#ifndef USAC_PROSACSAMPLER_H
#define USAC_PROSACSAMPLER_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Sampler.h"
#include "../Model.h"
#include "../RandomGenerator/UniformRandomGenerator.h"

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

class ProsacSampler : public Sampler {
protected:
    int kth_sample_number_;
    double t_n = 10000;
    int t = 1;
    int n;
    double t_n_prime = 1.0;
    unsigned int N_points;
public:
    ProsacSampler (int sample_size, int N_points, bool reset_time = true) {
        if (reset_time) randomGenerator->resetTime();

        this->sample_size = sample_size;
        this->N_points = N_points;

        randomGenerator = new UniformRandomGenerator;

        n = sample_size;
        // From Equations leading up to Eq 3 in Chum et al.
        for (int i = 0; i < sample_size; i++) {
            t_n *= (double) (n - i) / (N_points - i);
        }
    }

    void generateSample (int *sample) override {

        // Choose min n such that T_n_prime >= t (Eq. 5).
        if (t > t_n_prime && n < N_points) {
            double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - sample_size);
            t_n_prime += (t_n_plus1 - t_n);
            t_n = t_n_plus1;
            n++;
        }

        if (t_n_prime < kth_sample_number_) {
            // Randomly sample m data points from the top n data points.
            randomGenerator->resetGenerator(0, n-1);
            randomGenerator->generateUniqueRandomSet(sample, sample_size);

        } else {
            // Randomly sample m-1 data points from the top n-1 data points.

            randomGenerator->resetGenerator(0, n-2);
            randomGenerator->generateUniqueRandomSet(sample, sample_size-1);


            // Make the last point from the nth position.
            sample[sample_size-1] = n;
        }

        this->kth_sample_number_++;

    }
};

#endif //USAC_PROSACSAMPLER_H
