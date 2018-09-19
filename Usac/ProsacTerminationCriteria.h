#ifndef USAC_PROSACTERMINATIONCRITERIA_H
#define USAC_PROSACTERMINATIONCRITERIA_H

#include <vector>
#include <opencv2/core/mat.hpp>
#include "TerminationCriteria.h"

class ProsacTerminationCriteria {
private:
    bool initialized=false;
    std::vector<unsigned int> maximality_samples_prosac_;
    std::vector<unsigned int> non_random_inliers_prosac_;
    std::vector<unsigned int> growth_function_prosac_;
    std::vector<unsigned int> inlier_flags_;

    unsigned int stop_len_prosac_;
    unsigned int prosac_min_stop_length_ = 20;
    unsigned int usac_num_data_points_;
    unsigned int usac_min_sample_size_ = 2;
    unsigned int largest_size_prosac_;
    unsigned int usac_max_hypotheses_ = 10000;
    unsigned int prosac_growth_max_samples_ = 20000;

    float prosac_non_rand_conf_ = 0.95;
    float prosac_beta_ = 0.05;

    TerminationCriteria standart_termination_criteria;
public:
    bool isInitialized () { return initialized; }

    void initProsacTerminationCriteria (cv::InputArray input_points) {
        initialized=true;

        growth_function_prosac_.clear(); growth_function_prosac_.resize(usac_num_data_points_);
        double T_n;
        unsigned int T_n_p = 1;
        // compute initial value for T_n
        T_n = prosac_growth_max_samples_;
        for (unsigned int i = 0; i < usac_min_sample_size_; ++i) {
            T_n *= (double)(usac_min_sample_size_-i)/(usac_num_data_points_-i);
        }
        // compute values using recurrent relation
        for (unsigned int i = 0; i < usac_num_data_points_; ++i) {
            if (i+1 <= usac_min_sample_size_) {
                growth_function_prosac_[i] = T_n_p;
                continue;
            }
            double temp = (double)(i+1)*T_n/(i+1-usac_min_sample_size_);
            growth_function_prosac_[i] = T_n_p + (unsigned int)ceil(temp - T_n);
            T_n = temp;
            T_n_p = growth_function_prosac_[i];
        }

        // ------------------------------------------------------------------------
        // initialize the data structures that determine stopping

        // non-randomness constraint
        // i-th entry - inlier counts for termination up to i-th point (term length = i+1)
        non_random_inliers_prosac_.clear();
        non_random_inliers_prosac_.resize(usac_num_data_points_, 0);
        double pn_i = 1.0;    // prob(i inliers) with subset size n
        for (size_t n = usac_min_sample_size_+1; n <= usac_num_data_points_; ++n) {
            if (n-1 > 1000) {
                non_random_inliers_prosac_[n-1] = non_random_inliers_prosac_[n-2];
                continue;
            }

            std::vector<double> pn_i_vec(usac_num_data_points_, 0);
            // initial value for i = m+1 inliers
            pn_i_vec[usac_min_sample_size_] = (prosac_beta_)*std::pow((double)1-prosac_beta_, (double)n-usac_min_sample_size_-1)*(n-usac_min_sample_size_);
            pn_i = pn_i_vec[usac_min_sample_size_];
            for (size_t i = usac_min_sample_size_+2; i <= n; ++i) {
                // use recurrent relation to fill in remaining values
                if (i == n) {
                    pn_i_vec[n-1] = std::pow((double)prosac_beta_, (double)n-usac_min_sample_size_);
                    break;
                }
                pn_i_vec[i-1] = pn_i * ((prosac_beta_)/(1-prosac_beta_)) * ((double)(n-i)/(i-usac_min_sample_size_+1));
                pn_i = pn_i_vec[i-1];
            }
            // find minimum number of inliers satisfying the non-randomness constraint
            double acc = 0.0;
            unsigned int i_min = 0;
            for (size_t i = n; i >= usac_min_sample_size_+1; --i) {
                acc += pn_i_vec[i-1];
                if (acc < 1-prosac_non_rand_conf_) {
                    i_min = i;
                } else {
                    break;
                }
            }
            non_random_inliers_prosac_[n-1] = i_min;
        }

        // maximality constraint
        // i-th entry - number of samples for pool [0...i] (pool length = i+1)
        maximality_samples_prosac_.clear();
        maximality_samples_prosac_.resize(usac_num_data_points_);
        for (size_t i = 0; i < usac_num_data_points_; ++i) {
            maximality_samples_prosac_[i] = usac_max_hypotheses_;
        }

        // other initializations
        largest_size_prosac_ = usac_min_sample_size_;       // largest set sampled in PROSAC
        stop_len_prosac_ = input_points.size().width;      // current stopping length

        inlier_flags_.resize(usac_num_data_points_, 0);
    }

    unsigned int updatePROSACStopping(unsigned int hypCount) {
        unsigned int max_samples = maximality_samples_prosac_[stop_len_prosac_-1];

        // go through sorted points and track inlier counts
        unsigned int inlier_count = 0;

        // just accumulate the count for the first prosac_min_stop_length_ points
        for (unsigned int i = 0; i < prosac_min_stop_length_; ++i) {
            inlier_count += inlier_flags_[i];
        }

        // after this initial subset, try to update the stopping length if possible
        for (unsigned int i = prosac_min_stop_length_; i < usac_num_data_points_; ++i) {
            inlier_count += inlier_flags_[i];

            if (non_random_inliers_prosac_[i] < inlier_count) {
                non_random_inliers_prosac_[i] = inlier_count;	// update the best inliers for the the subset [0...i]

                // update the number of samples based on this inlier count
                if ( (i == usac_num_data_points_-1) ||
                     (inlier_flags_[i] && !inlier_flags_[i+1]) )
                {
                    unsigned int new_samples = standart_termination_criteria.getUpBoundIterations(inlier_count, i+1, usac_min_sample_size_, 0.99);
                    if (i+1 < largest_size_prosac_) {
                        // correct for number of samples that have points in [i+1, largest_size_prosac_-1]
                        new_samples += hypCount - growth_function_prosac_[i];
                    }

                    if (new_samples < maximality_samples_prosac_[i]) {
                        // if number of samples can be lowered, store values and update stopping length
                        maximality_samples_prosac_[i] = new_samples;
                        if ( (new_samples < max_samples) || ( (new_samples == max_samples) && (i+1 >= stop_len_prosac_) ) ) {
                            stop_len_prosac_ = i+1;
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
