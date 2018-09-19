#ifndef USAC_PROSACSAMPLER2_H
#define USAC_PROSACSAMPLER2_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Sampler.h"
#include "../Model.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"

// theia implementation
// Copyright (C) 2014 The Regents of the University of California (Regents).

class ProsacSampler2 : public Sampler {
protected:
    bool initialized=false;

    std::vector<unsigned int> growth_function_prosac_;
    unsigned int hypCount = 1;
    unsigned int N_points;
    unsigned int prosac_growth_max_samples_ = 20000;
    unsigned int subset_size_prosac_;
    unsigned int largest_size_prosac_;
    unsigned int points_size;
    unsigned int sample_size = 2;

    unsigned int * stop_len_prosac_;

    RandomGenerator * randomGenerator;
public:
    bool isInitialized () { return initialized; }

    void initProsacSampler (unsigned int sample_size, int N_points, bool reset_time = true) {
        randomGenerator = new UniformRandomGenerator;
        if (reset_time) randomGenerator->resetTime();

        this->sample_size = sample_size;
        points_size = N_points;

        growth_function_prosac_.clear();
        growth_function_prosac_.resize(points_size);
        double T_n;
        unsigned int T_n_p = 1;
        // compute initial value for T_n
        T_n = prosac_growth_max_samples_;
        for (unsigned int i = 0; i < sample_size; ++i) {
            T_n *= (double)(sample_size-i)/(points_size-i);
        }
        // compute values using recurrent relation
        for (unsigned int i = 0; i < points_size; ++i) {
            if (i+1 <= sample_size) {
                growth_function_prosac_[i] = T_n_p;
                continue;
            }
            double temp = (double)(i+1)*T_n/(i+1-sample_size);
            growth_function_prosac_[i] = T_n_p + (unsigned int)ceil(temp - T_n);
            T_n = temp;
            T_n_p = growth_function_prosac_[i];
        }

        // other initializations
        largest_size_prosac_ = sample_size;       // largest set sampled in PROSAC
        subset_size_prosac_ = sample_size;		// size of the current sampling pool
        *stop_len_prosac_    = points_size;		// current stopping length
        hypCount=1;
    }




    void generateSample (int * sample) {
        // revert to RANSAC-style sampling if maximum number of PROSAC samples have been tested
        if (hypCount > prosac_growth_max_samples_) {
            randomGenerator->generateUniqueRandomSet(sample, points_size, sample_size);
            return;
        }

        // if current stopping length is less than size of current pool, use only points up to the stopping length
        if (subset_size_prosac_ > *stop_len_prosac_) {
            randomGenerator->generateUniqueRandomSet(sample, *stop_len_prosac_, sample_size);
        }

        // increment the size of the sampling pool if required
        if (hypCount > growth_function_prosac_[subset_size_prosac_-1]) {
            ++subset_size_prosac_;
            if (subset_size_prosac_ > points_size) {
                subset_size_prosac_ = points_size;
            }
            if (largest_size_prosac_ < subset_size_prosac_) {
                largest_size_prosac_ = subset_size_prosac_;
            }
        }

        // generate PROSAC sample
        randomGenerator->generateUniqueRandomSet(sample, subset_size_prosac_-1, sample_size-1);
        sample[sample_size-1] = subset_size_prosac_-1;

        hypCount++;
    }



};

#endif //USAC_PROSACSAMPLER2_H
