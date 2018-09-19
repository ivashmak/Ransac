#ifndef USAC_PROSACSAMPLER_H
#define USAC_PROSACSAMPLER_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Sampler.h"
#include "../Model.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"
#include "../../RandomGenerator/ArrayRandomGenerator.h"

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

class ProsacSampler : public Sampler {
protected:
    int kth_sample_number_;
    double t_n;
    int n;
    double t_n_prime;
    RandomGenerator *array_rand_gen;
public:
    ProsacSampler (unsigned int sample_size, unsigned int N_points, bool reset_time = true) {
        array_rand_gen = new ArrayRandomGenerator;
        initSample(sample_size, N_points);
    }

    void initSample (unsigned int sample_size, unsigned int points_size, bool reset_time = true) {
        if (reset_time) array_rand_gen->resetTime();

        this->sample_size = sample_size;
        this->points_size = points_size;

        n = sample_size;
        t_n = 1000;
        t_n_prime = 1.0;

        // From Equations leading up to Eq 3 in Chum et al.
        //t_n samples containing only data points from U_n and
        // t_n+1 samples containing only data points from U_n+1
        for (int i = 0; i < sample_size; i++) {
            t_n *= (double) (n - i) / (points_size - i);
        }

        kth_sample_number_ = 1;
    }


    void generateSample (int * sample) override {

        // Choice of the hypothesis generation set
        if (kth_sample_number_ > t_n_prime && n < points_size) {
            double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - sample_size);
            t_n_prime += ceil(t_n_plus1 - t_n);
            t_n = t_n_plus1;
            n++;
        }

        // Semi-random sample Mt of size m
        if (t_n_prime < kth_sample_number_) {
            array_rand_gen->resetGenerator(0, n-1);
            array_rand_gen->generateUniqueRandomSet(sample, sample_size);
        } else {
            array_rand_gen->resetGenerator(0, n-2);
            array_rand_gen->generateUniqueRandomSet(sample, sample_size-1);
            sample[sample_size-1] = n; // Make the last point from the nth position.
        }

        kth_sample_number_++;
    }
};

#endif //USAC_PROSACSAMPLER_H