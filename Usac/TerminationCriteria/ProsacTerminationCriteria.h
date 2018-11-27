#ifndef USAC_PROSACTERMINATIONCRITERIA_H
#define USAC_PROSACTERMINATIONCRITERIA_H

#include <vector>
#include <opencv2/core/mat.hpp>
#include "StandardTerminationCriteria.h"

////////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2012 University of North Carolina at Chapel Hill
//  All Rights Reserved
//
//  Permission to use, copy, modify and distribute this software and its
//  documentation for educational, research and non-profit purposes, without
//  fee, and without a written agreement is hereby granted, provided that the
//  above copyright notice and the following paragraph appear in all copies.
//  
//  The University of North Carolina at Chapel Hill make no representations
//  about the suitability of this software for any purpose. It is provided
//  'as is' without express or implied warranty. 
//
//  Please send BUG REPORTS to rraguram@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////

class ProsacTerminationCriteria : public TerminationCriteria {
private:
    bool initialized = false;

    unsigned int * maximality_samples;
    unsigned int * non_random_inliers;
    unsigned int * inlier_flags;

    unsigned int * growth_function;

    unsigned int stopping_length;
    const unsigned int min_stopping_length = 20;

    unsigned int points_size;
    unsigned int sample_size;

    float desired_prob;
    StandardTerminationCriteria * standart_termination_criteria;
public:

    ~ProsacTerminationCriteria () {
        delete maximality_samples, non_random_inliers, inlier_flags;
    }

    /*
        All arrays have been initialized in Prosac Sampler,
        so termination criteria have only pointer to them.
    */
    void initProsacTerminationCriteria (unsigned int * growth_function_,
                                        const Model * const model,
                                        unsigned int points_size_) {

        standart_termination_criteria = new StandardTerminationCriteria;
        standart_termination_criteria->init (model, points_size_);

        growth_function = growth_function_;

        sample_size = model->sample_number;
        points_size = points_size_;

        desired_prob = model->desired_prob;

        stopping_length = points_size;
    
        // ------------------------------------------------------------------------
        // initialize the data structures that determine stopping
        const float non_randomness = 0.95;
        const float beta = 0.05;
        const unsigned int max_hypotheses = 10000; // max iterations
        
        // non-randomness constraint
        // i-th entry - inlier counts for termination up to i-th point (term length = i+1)
        non_random_inliers = (unsigned int *) calloc (points_size, sizeof (unsigned int));
        double pn_i = 1.0;    // prob(i inliers) with subset size n
        double * pn_i_vec;
        for (size_t n = sample_size+1; n <= points_size; ++n) {
            if (n-1 > 1000) {
                non_random_inliers[n-1] = non_random_inliers[n-2];
                continue;
            }

            pn_i_vec = (double *) calloc (points_size, sizeof (double));

            // initial value for i = m+1 inliers
            pn_i_vec[sample_size] = (beta)*std::pow((double)1-beta, (double)n-sample_size-1)*(n-sample_size);
            pn_i = pn_i_vec[sample_size];
            for (size_t i = sample_size+2; i <= n; ++i) {
                // use recurrent relation to fill in remaining values
                if (i == n) {
                    pn_i_vec[n-1] = std::pow((double)beta, (double)n-sample_size);
                    break;
                }
                pn_i_vec[i-1] = pn_i * ((beta)/(1-beta)) * ((double)(n-i)/(i-sample_size+1));
                pn_i = pn_i_vec[i-1];
            }
            // find minimum number of inliers satisfying the non-randomness constraint
            double acc = 0.0;
            unsigned int i_min = 0;
            for (size_t i = n; i >= sample_size+1; --i) {
                acc += pn_i_vec[i-1];
                if (acc < 1-non_randomness) {
                    i_min = i;
                } else {
                    break;
                }
            }
            non_random_inliers[n-1] = i_min;
        }

        // maximality constraint
        // i-th entry - number of samples for pool [0...i] (pool length = i+1)
        maximality_samples = new unsigned int[points_size];
        for (size_t i = 0; i < points_size; ++i) {
            maximality_samples[i] = max_hypotheses;
        }
        
    }

    unsigned int getStoppingLength () {
        return stopping_length;
    }

    inline unsigned int getUpBoundIterations (unsigned int inlier_size) override {
        return standart_termination_criteria->getUpBoundIterations(inlier_size);
    }
    inline unsigned int getUpBoundIterations (unsigned int inlier_size, unsigned int points_size) override {
        return standart_termination_criteria->getUpBoundIterations(inlier_size, points_size);
    }

    /*
            Returns predicted maximum iterations
         */
    unsigned int getUpBoundIterations(unsigned int hypCount, unsigned int largest_sample_size,
                                    const int * const inliers, unsigned int inliers_size) {

        inlier_flags = (unsigned int *) calloc (points_size, sizeof(unsigned int));
        for (int i = 0; i < inliers_size; i++) {
            inlier_flags[inliers[i]] = 1;
        }

        unsigned int max_samples = maximality_samples[stopping_length-1];

        // go through sorted points and track inlier counts
        unsigned int inlier_count = 0;

        // just accumulate the count for the first min_stopping_length points
        for (unsigned int i = 0; i < min_stopping_length; ++i) {
            inlier_count += inlier_flags[i];
        }

        // after this initial subset, try to update the stopping length if possible
        for (unsigned int i = min_stopping_length; i < points_size; ++i) {
            inlier_count += inlier_flags[i];

            if (non_random_inliers[i] < inlier_count) {
                non_random_inliers[i] = inlier_count;	// update the best inliers for the the subset [0...i]

                // update the number of samples based on this inlier count
                if ((i == points_size-1) || (inlier_flags[i] && !inlier_flags[i+1])) {
                    unsigned int new_samples = standart_termination_criteria->
                                            getUpBoundIterations(inlier_count, i+1);

                    if (i+1 < largest_sample_size) {
                        // correct for number of samples that have points in [i+1, largest_sample_size-1]
                        new_samples += hypCount - growth_function[i];
                    }

                    if (new_samples < maximality_samples[i]) {
                        // if number of samples can be lowered, store values and update stopping length
                        maximality_samples[i] = new_samples;
                        if ((new_samples < max_samples) || ((new_samples == max_samples) && (i+1 >= stopping_length))) {
                            stopping_length = i+1;
                            max_samples = new_samples;
                        }
                    }
                }
            }
        }

        return max_samples;
    }
};

#endif //USAC_PROSACTERMINATIONCRITERIA_H
