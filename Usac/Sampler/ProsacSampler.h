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
    UniformSampler * uniform_sampler;
    double t_n;
    int t;
    int n;
    double t_n_prime;
public:
    ProsacSampler (int sample_size, int N_points, bool reset_time = true) {
        if (reset_time) resetTime();

        this->sample_size = sample_size;
        this->N_points = N_points;

        std::random_device rand_dev;
        generator = std::mt19937(rand_dev());

        kth_sample_number_ = 1;
        int max_convergence_iterations_ = 10000;

        uniform_sampler = new UniformSampler (sample_size, N_points);

        t_n = max_convergence_iterations_;
        t_n_prime = 1.0;
        t = 1;
        n = sample_size;
        // From Equations leading up to Eq 3 in Chum et al.
        for (int i = 0; i < sample_size; i++) {
            t_n *= (double) (sample_size - i) / (N_points - i);
        }

    }

    // Samples the input variable data and fills the vector subset with the prosac
    // samples.
    // NOTE: This assumes that data is in sorted order by quality where data[i] is
    // of higher quality than data[j] for all i < j.


    int updateProsac (unsigned int hypCount)  {
        unsigned int max_samples = maximality_samples_prosac_[stop_len_prosac_-1];

        // go through sorted points and track inlier counts
        unsigned int inlier_count = 0;

        // just accumulate the count for the first prosac_min_stop_length_ points
        for (unsigned int i = 0; i < prosac_min_stop_length_; ++i)
        {
            inlier_count += usac_results_.inlier_flags_[prosac_sorted_point_indices_[i]];
        }

        // after this initial subset, try to update the stopping length if possible
        for (unsigned int i = prosac_min_stop_length_; i < usac_num_data_points_; ++i)
        {
            inlier_count += usac_results_.inlier_flags_[prosac_sorted_point_indices_[i]];

            if (non_random_inliers_prosac_[i] < inlier_count)
            {
                non_random_inliers_prosac_[i] = inlier_count;	// update the best inliers for the the subset [0...i]

                // update the number of samples based on this inlier count
                if ( (i == usac_num_data_points_-1) ||
                     (usac_results_.inlier_flags_[prosac_sorted_point_indices_[i]] && !usac_results_.inlier_flags_[prosac_sorted_point_indices_[i+1]]) )
                {
                    unsigned int new_samples = updateStandardStopping(inlier_count, i+1, usac_min_sample_size_);
                    if (i+1 < largest_size_prosac_)
                    {
                        // correct for number of samples that have points in [i+1, largest_size_prosac_-1]
                        new_samples += hypCount - growth_function_prosac_[i];
                    }

                    if (new_samples < maximality_samples_prosac_[i])
                    {
                        // if number of samples can be lowered, store values and update stopping length
                        maximality_samples_prosac_[i] = new_samples;
                        if ( (new_samples < max_samples) || ( (new_samples == max_samples) && (i+1 >= stop_len_prosac_) ) )
                        {
                            stop_len_prosac_ = i+1;
                            max_samples = new_samples;
                        }
                    }
                }
            }
        }
        return max_samples;
    }

    void generateSample (int *sample) override {

        // Choose min n such that T_n_prime >= t (Eq. 5).
        if (t > t_n_prime && n < N_points) {
            double t_n_plus1 = (t_n * (n + 1.0)) / (n + 1.0 - sample_size);
            t_n_prime += ceil(t_n_plus1 - t_n);
            t_n = t_n_plus1;
            n++;
        }

        if (t_n_prime < kth_sample_number_) {
            // Randomly sample m data points from the top n data points.
            uniform_sampler->setSampleSize(sample_size);
            uniform_sampler->resetGenerator(n); // n - 1
            uniform_sampler->generateSample(sample);

        } else {
            // Randomly sample m-1 data points from the top n-1 data points.
            uniform_sampler->setSampleSize(sample_size-1);
            uniform_sampler->resetGenerator(n-1); // n - 2
            uniform_sampler->generateSample(sample);

            // Make the last point from the nth position.
            sample[sample_size-1] = n;
        }

        this->kth_sample_number_++;

    }
};

#endif //USAC_PROSACSAMPLER_H
