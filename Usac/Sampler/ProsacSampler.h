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

    const unsigned int growth_max_samples = 20000;

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

    bool isInitialized () { return initialized; }

    void initProsacSampler (unsigned int sample_size_, unsigned int points_size_, bool reset_time = true) {
        sample_size = sample_size_;
        points_size = points_size_;

        randomGenerator = new UniformRandomGenerator;
        if (reset_time) randomGenerator->resetTime();
        
        growth_function = new unsigned int[points_size];
        
        unsigned int T_n_p = 1;
        // compute initial value for T_n
        double T_n = growth_max_samples;
        for (unsigned int i = 0; i < sample_size; ++i) {
            T_n *= (double)(sample_size-i)/(points_size-i);
        }
        // compute values using recurrent relation
        for (unsigned int i = 0; i < points_size; ++i) {
            if (i+1 <= sample_size) {
                growth_function[i] = T_n_p;
                continue;
            }
            double temp = (double)(i+1)*T_n/(i+1-sample_size);
            growth_function[i] = T_n_p + (unsigned int)ceil(temp - T_n);
            T_n = temp;
            T_n_p = growth_function[i];
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
        return true;
    }
};

#endif //RANSAC_PROSAC_SAMPLER_H
