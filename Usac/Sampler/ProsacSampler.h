#ifndef RANSAC_PROSAC_SAMPLER_H
#define RANSAC_PROSAC_SAMPLER_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Sampler.h"
#include "../Model.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"

class ProsacSampler : public Sampler {
protected:
    bool initialized = false;

    unsigned int * growth_function;

    unsigned int subset_size;
    unsigned int largest_sample_size;

    unsigned int hypCount;

    unsigned int growth_max_samples;

    unsigned int sample_size;
    unsigned int points_size;
    
    RandomGenerator * randomGenerator;
public:
    ~ProsacSampler () {
        delete growth_function;
    }

    unsigned int * getGrowthFunction () {
        return growth_function;
    }

    unsigned int getLargestSampleSize () {
        return largest_sample_size;
    }

    void initProsacSampler (unsigned int sample_size_, unsigned int points_size_, bool reset_time = true) {
        sample_size = sample_size_;
        points_size = points_size_;

        // it is T_N
        // Imagine standard RANSAC drawing T_N samples of size m out of N data points
        // In our experiments, the parameter was set to T_N = 200000
        growth_max_samples = 200000; // model->max_iters

        randomGenerator = new UniformRandomGenerator;
        if (reset_time) randomGenerator->resetTime();
        
        growth_function = new unsigned int[points_size];
        // The data points in U_N are sorted in descending order w.r.t. the quality function q.
        // Let {Mi}i = 1...T_N denote the sequence of samples Mi c U_N that are uniformly drawn by Ransac.

        // Let T_n be an average number of samples from {Mi}i=1...T_N that contain data points from U_n only.
        // compute initial value for T_n
        //                                  n - i
        // T_n = T_N * Product i = 0...m-1 -------, n >= sample size, N = points size
        //                                  N - i
        double T_n = growth_max_samples;
        for (unsigned int i = 0; i < sample_size; ++i) {
            T_n *= (double)(sample_size-i)/(points_size-i);
        }

        unsigned int T_n_prime = 1;
        // compute values using recurrent relation
        //             n + 1
        // T(n+1) = --------- T(n), m is sample size.
        //           n + 1 - m

        // growth function is defined as
        // g(t) = min {n, T'_(n) >= t}
        // T'_(n+1) = T'_(n) + (T_(n+1) - T_(n))
        for (unsigned int i = 0; i < points_size; ++i) {
            if (i+1 <= sample_size) {
                growth_function[i] = T_n_prime;
                continue;
            }
            double Tn_plus1 = (double)(i+1)*T_n/(i+1-sample_size);
            growth_function[i] = T_n_prime + (unsigned int)ceil(Tn_plus1 - T_n);
            T_n = Tn_plus1;
            T_n_prime = growth_function[i];
        }

        // other initializations
        largest_sample_size = sample_size;       // largest set sampled in PROSAC
        subset_size = sample_size;		// size of the current sampling pool
        hypCount = 1;

        initialized = true;
    }


    void generateSample (int * sample) override {
    }
    
    void generateSampleProsac (int * sample, unsigned int stopping_length) {
//        std::cout << "stopping length " << stopping_length << "\n";

        // revert to RANSAC-style sampling if maximum number of PROSAC samples have been tested
        if (hypCount > growth_max_samples) {
            randomGenerator->generateUniqueRandomSet(sample, sample_size, points_size);
            return;
        }

        // if current stopping length is less than size of current pool, use only points up to the stopping length
        if (subset_size > stopping_length) {
            randomGenerator->generateUniqueRandomSet(sample, sample_size, stopping_length);
        }

        // increment the size of the sampling pool if required
        if (hypCount > growth_function[subset_size-1]) {
            ++subset_size;
            if (subset_size > points_size) {
                subset_size = points_size;
            }
            if (largest_sample_size < subset_size) {
                largest_sample_size = subset_size;
            }
        }

        // generate PROSAC sample
        randomGenerator->generateUniqueRandomSet(sample, sample_size-1, subset_size-1);
        sample[sample_size-1] = subset_size-1;

        hypCount++;
    }

    bool isInit () override {
        return initialized;
    }
};

#endif //RANSAC_PROSAC_SAMPLER_H
