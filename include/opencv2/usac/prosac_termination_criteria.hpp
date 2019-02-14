// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_PROSACTERMINATIONCRITERIA_H
#define USAC_PROSACTERMINATIONCRITERIA_H

#include "standard_termination_criteria.hpp"

namespace cv { namespace usac {
class ProsacTerminationCriteria : public TerminationCriteria {
private:
    bool initialized = false;

    unsigned int *maximality_samples;
    unsigned int *non_random_inliers;
    unsigned int *growth_function;

    unsigned int *largest_sample_size;
    unsigned int termination_length;
    const unsigned int min_termination_length = 20;

    unsigned int points_size;
    unsigned int sample_size;

    StandardTerminationCriteria *standart_termination_criteria;
    Estimator *estimator;

    float threshold;
public:

    ~ProsacTerminationCriteria() {
        delete[] maximality_samples;
        delete[] non_random_inliers;
        delete (standart_termination_criteria);
    }

    void setLargestSampleSize(unsigned int *largest_sample_size_) {
        largest_sample_size = largest_sample_size_;
    }

    /*
        Remember to initialize estimator with sorted points and Prosac sampler before.
    */
    ProsacTerminationCriteria(unsigned int *growth_function_,
                              const Model *const model,
                              unsigned int points_size_,
                              Estimator *estimator_) {

        standart_termination_criteria = new StandardTerminationCriteria(model, points_size_);

        estimator = estimator_;
        growth_function = growth_function_;

        threshold = model->threshold;

        sample_size = model->sample_size;
        points_size = points_size_;

        termination_length = points_size;

        // ------------------------------------------------------------------------
        // initialize the data structures that determine stopping
        const float non_randomness = 0.95;
        const float beta = 0.05;
        const unsigned int max_hypotheses = 10000; // max iterations

        // non-randomness constraint
        // The non-randomness requirement prevents PROSAC
        // from selecting a solution supported by outliers that are
        // by chance consistent with it.  The constraint is typically
        // checked ex-post in standard approaches [1]. The distribution
        // of the cardinalities of sets of random ‘inliers’ is binomial
        // i-th entry - inlier counts for termination up to i-th point (term length = i+1)
        non_random_inliers = (unsigned int *) calloc(points_size, sizeof(unsigned int));
        double pn_i = 1.0;    // prob(i inliers) with subset size n
        double *pn_i_vec;
        for (size_t n = sample_size + 1; n <= points_size; ++n) {
            if (n - 1 > 1000) {
                non_random_inliers[n - 1] = non_random_inliers[n - 2];
                continue;
            }

            pn_i_vec = (double *) calloc(points_size, sizeof(double));

            // initial value for i = m+1 inliers
            pn_i_vec[sample_size] =
                    (beta) * std::pow((double) 1 - beta, (double) n - sample_size - 1) * (n - sample_size);
            pn_i = pn_i_vec[sample_size];
            for (size_t i = sample_size + 2; i <= n; ++i) {
                // use recurrent relation to fill in remaining values
                if (i == n) {
                    pn_i_vec[n - 1] = std::pow((double) beta, (double) n - sample_size);
                    break;
                }
                pn_i_vec[i - 1] = pn_i * ((beta) / (1 - beta)) * ((double) (n - i) / (i - sample_size + 1));
                pn_i = pn_i_vec[i - 1];
            }
            // find minimum number of inliers satisfying the non-randomness constraint
            double acc = 0.0;
            unsigned int i_min = 0;
            for (size_t i = n; i >= sample_size + 1; --i) {
                acc += pn_i_vec[i - 1];
                if (acc < 1 - non_randomness) {
                    i_min = i;
                } else {
                    break;
                }
            }
            non_random_inliers[n - 1] = i_min;
        }

        // maximality constraint defines how many samples are
        // needed to be drawn to ensure the confidence in the solution
        // and is the (only) termination criterion of RANSAC
        // i-th entry - number of samples for pool [0...i] (pool length = i+1)
        maximality_samples = new unsigned int[points_size];
        for (size_t i = 0; i < points_size; ++i) {
            maximality_samples[i] = max_hypotheses;
        }
    }

    // return stopping length as pointer to stopping length for prosac sampler
    unsigned int *getStoppingLength() {
        return &termination_length;
    }

    inline unsigned int getUpBoundIterations(unsigned int inlier_size) override {
        return standart_termination_criteria->getUpBoundIterations(inlier_size);
    }

    inline unsigned int getUpBoundIterations(unsigned int inlier_size, unsigned int points_size) override {
        return standart_termination_criteria->getUpBoundIterations(inlier_size, points_size);
    }


    /*
     * Returns predicted maximum iterations
     * The PROSAC algorithm terminates if the number of inliers In∗
     * within the set Un∗ satisfies the following conditions:
     *
     * • non-randomness – the probability that In∗ out of n∗ (termination_length)
     * data points are by chance inliers to an arbitrary incorrect model
     * is smaller than Ψ (typically set to 5%)
     *
     * •maximality – the probability that a solution with more than
     * In∗ inliers in Un∗ exists and was not found after k
     * samples is smaller than η0 (typically set to 5%).
     */
    inline unsigned int getUpBoundIterations(unsigned int hypCount, const cv::Mat &model) {

        unsigned int max_samples = maximality_samples[termination_length - 1];

        // go through sorted points and track inlier counts
        unsigned int inlier_count = 0;

        estimator->setModelParameters(model);
        // just accumulate the count for the first min_termination_length points
        for (unsigned int i = 0; i < min_termination_length; i++) {
            inlier_count += estimator->getError(i) < threshold;
        }

        bool is_inlier_iplus1;
        bool is_inlier_i = estimator->getError(min_termination_length) < threshold;

        // after this initial subset, try to update the stopping length if possible
        for (unsigned int i = min_termination_length; i < points_size; ++i) {
            if (i != points_size - 1) {
                is_inlier_iplus1 = estimator->getError(i + 1) < threshold;
            }

            inlier_count += is_inlier_i;

            if (non_random_inliers[i] < inlier_count) {
                non_random_inliers[i] = inlier_count;    // update the best inliers for the the subset [0...i]

                // update the number of samples based on this inlier count
                if ((i == points_size - 1) || (is_inlier_i && !is_inlier_iplus1)) {
                    unsigned int new_samples = standart_termination_criteria->
                            getUpBoundIterations(inlier_count, i + 1);
                    if (i + 1 < *largest_sample_size) {
                        // correct for number of samples that have points in [i+1, largest_sample_size-1]
                        new_samples += hypCount - growth_function[i];
                    }

                    if (new_samples < maximality_samples[i]) {
                        // if number of samples can be lowered, store values and update stopping length
                        maximality_samples[i] = new_samples;
                        if ((new_samples < max_samples) ||
                            ((new_samples == max_samples) && (i + 1 >= termination_length))) {
                            termination_length = i + 1;
                            max_samples = new_samples;
                        }
                    }
                }
            }

            is_inlier_i = is_inlier_iplus1;
        }

        return max_samples;
    }
};
}}
#endif //USAC_PROSACTERMINATIONCRITERIA_H
