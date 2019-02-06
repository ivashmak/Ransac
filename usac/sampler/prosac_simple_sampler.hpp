// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_PROSAC_SIMPLE_SAMPLER_H
#define USAC_PROSAC_SIMPLE_SAMPLER_H

#include <cmath>
#include <vector>
#include <iostream>
#include "sampler.hpp"
#include "../model.hpp"
#include "../random_generator/array_random_generator.hpp"

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

class ProsacSimpleSampler : public Sampler {
protected:
    int kth_sample_number;
    double t_n;
    int n;
    double t_n_prime;
    UniformRandomGenerator * uniformRandomGenerator;
public:
    ~ProsacSimpleSampler() override {
        delete (uniformRandomGenerator);
    }

    ProsacSimpleSampler (unsigned int sample_size_, unsigned int points_size_, bool reset_time) {
        uniformRandomGenerator = new UniformRandomGenerator;
        if (reset_time) uniformRandomGenerator->resetTime();

        sample_size = sample_size_;
        points_size = points_size_;

        n = sample_size;
        t_n = 1000;
        t_n_prime = 1.0;

        // From Equations leading up to Eq 3 in Chum et al.
        // t_n samples containing only data points from U_n and
        // t_n+1 samples containing only data points from U_n+1
        for (int i = 0; i < sample_size; i++) {
            t_n *= (double) (n - i) / (points_size - i);
        }

        kth_sample_number = 1;
    }


    void generateSample (int * sample) override {

        // Choice of the hypothesis generation set
        if (kth_sample_number > t_n_prime && n < points_size) {
            double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - sample_size);
            t_n_prime += ceil(t_n_plus1 - t_n);
            t_n = t_n_plus1;
            n++;
        }

        // Semi-random sample Mt of size m
        if (t_n_prime < kth_sample_number) {
            uniformRandomGenerator->resetGenerator(0, n-1);
            uniformRandomGenerator->setSubsetSize(sample_size);
            uniformRandomGenerator->generateUniqueRandomSet(sample);
        } else {
            uniformRandomGenerator->resetGenerator(0, n-2);
            uniformRandomGenerator->setSubsetSize(sample_size-1);
            uniformRandomGenerator->generateUniqueRandomSet(sample);
            sample[sample_size-1] = n; // Make the last point from the nth position.
        }

        kth_sample_number++;

        // debug
//        std::vector<int> s;
//        Sample (s);
//        for (int i = 0; i < sample_size; i++) {
//            std::cout << sample[i] << " " << s[i] << "\n";
//        }
    }


    // Original theia (Just for compare)
    int kth_sample_number_ = 1;
    bool Sample(std::vector<int>& subset) {
        // Set t_n according to the PROSAC paper's recommendation.
        double t_n = 1000;
        int n = sample_size;
        // From Equations leading up to Eq 3 in Chum et al.
        for (int i = 0; i < sample_size; i++) {
            t_n *= static_cast<double>(n - i) / (points_size - i);
        }

        double t_n_prime = 1.0;
        // Choose min n such that T_n_prime >= t (Eq. 5).
        for (int t = 1; t <= kth_sample_number_; t++) {
            if (t > t_n_prime && n < points_size) {
                double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - sample_size);
                t_n_prime += ceil(t_n_plus1 - t_n);
                t_n = t_n_plus1;
                n++;
            }
        }
        subset.reserve(sample_size);
        if (t_n_prime < kth_sample_number_) {
            // Randomly sample m data points from the top n data points.
            std::vector<int> random_numbers;
            for (int i = 0; i < sample_size; i++) {
                // Generate a random number that has not already been used.
                int rand_number;
                while (std::find(random_numbers.begin(), random_numbers.end(),
                                 (rand_number = random () % n)) !=
                       random_numbers.end()) {}

                random_numbers.push_back(rand_number);

                // Push the *unique* random index back.
                subset.push_back(rand_number);
            }
        } else {
            std::vector<int> random_numbers;
            // Randomly sample m-1 data points from the top n-1 data points.
            for (int i = 0; i < sample_size - 1; i++) {
                // Generate a random number that has not already been used.
                int rand_number;
                while (std::find(random_numbers.begin(), random_numbers.end(),
                                 (rand_number = random() % (n-1))) !=
                       random_numbers.end()) {}

                random_numbers.push_back(rand_number);

                // Push the *unique* random index back.
                subset.push_back(rand_number);
            }
            // Make the last point from the nth position.
            subset.push_back(n);
        }
        if (subset.size() != sample_size)
            std::cout << "Prosac subset is of incorrect " << "size!"  << std::endl;
        kth_sample_number_++;
        return true;
    }

    bool isInit () override {
        return true;
    }
};

#endif //USAC_PROSAC_SIMPLE_SAMPLER_H