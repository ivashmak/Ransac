#ifndef RANSAC_PROSAC_SAMPLER_H
#define RANSAC_PROSAC_SAMPLER_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Sampler.h"
#include "../Model.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"
#include "../TerminationCriteria/ProsacTerminationCriteria.h"

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
    
    UniformRandomGenerator * randomGenerator;

    unsigned int * termination_length;
public:
    // set stopping length as pointer to stopping length from prosac termination criteria
    void setTerminationLength (unsigned int * termination_length_) {
        termination_length = termination_length_;
    }

    ~ProsacSampler () override {
        if (initialized) {
            delete[] growth_function;
            delete (randomGenerator);
        }
    }

    unsigned int * getGrowthFunction () {
        return growth_function;
    }

    // return largest_sample_size as pointer to prosac termination criteria
    unsigned int * getLargestSampleSize () {
        return &largest_sample_size;
    }

    /*
     * Quality sort (Chum, Matas):
     * Matching based on SIFT descriptors [5] was used to obtain
     * tentative correspondences in PLANT and MUG experiments 2.
     * The similarity was defined as the ratio of the distances in
     * the SIFT space of the best and second match.
     */
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
        subset_size = sample_size;		// n,  size of the current sampling pool
        hypCount = 1; // t

        initialized = true;
    }


    void generateSample (int * sample) override {
//        std::cout << "subset size " << subset_size << "\n";
//        std::cout << "stopping length " << termination_length << "\n";

        // revert to RANSAC-style sampling if maximum number of PROSAC samples have been tested
        if (hypCount > growth_max_samples) {
            randomGenerator->generateUniqueRandomSet(sample, sample_size, points_size);
            return;
        }


/*
        //-----------------------------------------------------------------------
        // Choice of the hypothesis generation set
        // if (t = T'_n) & (n < n*) then n = n + 1 (eqn. 4)
        if (hypCount == growth_function[subset_size] && subset_size < termination_length) {
            subset_size++;
        }

        // Semi-random sample M_t of size m
        if (growth_function[subset_size] < hypCount) {
            // The sample contains m-1 points selected from U_(n-1) at random and u_n
            randomGenerator->generateUniqueRandomSet(sample, sample_size-1, subset_size-2);
            sample[sample_size-1] = subset_size-1;
        } else {
            // Select m points from U_n at random.
            randomGenerator->generateUniqueRandomSet(sample, sample_size, subset_size-1);
        }
        hypCount++;
        return;
        //-----------------------------------------------------------------------
*/

        // if current stopping length is less than size of current pool, use only points up to the stopping length
        if (subset_size > *termination_length) {
            randomGenerator->generateUniqueRandomSet(sample, sample_size, *termination_length);
            return;
        }

        // increment the size of the sampling pool if required
        if (hypCount > growth_function[subset_size-1]) {
            ++subset_size; // n = n + 1
            if (subset_size > points_size) {
                subset_size = points_size;
            }
            if (largest_sample_size < subset_size) {
                largest_sample_size = subset_size;
            }
        }

        // generate PROSAC sample in range <0, subset_size-2>
        randomGenerator->generateUniqueRandomSet(sample, sample_size-1, subset_size-2);
        sample[sample_size-1] = subset_size-1;

        hypCount++; // t = t + 1
    }

    bool isInit () override {
        return initialized;
    }
};

#endif //RANSAC_PROSAC_SAMPLER_H
