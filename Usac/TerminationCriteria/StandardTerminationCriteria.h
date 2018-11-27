#ifndef RANSAC_STANDARD_TERMINATION_CRITERIA_H
#define RANSAC_STANDARD_TERMINATION_CRITERIA_H


#include "TerminationCriteria.h"

class StandardTerminationCriteria : public TerminationCriteria {
private:
    const Model * model;
    float log_1_p;
    unsigned int sample_size;
    unsigned int points_size;
    unsigned int max_iterations = 10000;
    const float EPSILON = 0.00001;
public:

    /*
     * Initialize termination criteria
     */
    void init (const Model * const model, unsigned int points_size_) override {
        assert (model != nullptr);
        this->model = model;
        log_1_p = (float) log (1-model->desired_prob);
        sample_size = model->sample_number;
        max_iterations = model->max_iterations;
        points_size = points_size_;
        isinit = true;
    }


	/*
	 * Get upper bound iterations for any sample number
	 * Can be used ceil.
	 * If inlier ratio is almost 0, so under logarithm is almost 1, so denominator is almost 0
	 * In this case just return max iterations from model parameters or 10000 if model is not defined.
	 * Otherwise upper bound iterations will not fit unsigned int type and will be (unsigned int) -inf = 0
     *
	 * n is points size, w is inlier ratio, p is desired probability, k is expceted number of iterations.
	 * 1 - p = (1 - w^n)^k,
	 * k = log_(1-w^n) (1-p)
	 * k = ln (1-p) / ln (1-w^n)
	 *
	 * w^n is probability that all N points are inliers.
	 * (1 - w^n) is probability that at least one point of N is outlier.
	 * 1 - p = (1-w^n)^k is probability that in K steps of getting at least one outlier is 1% (5%).
	 *
	 * Assume sample size is at least 2
	 */
    inline unsigned int getUpBoundIterations (unsigned int inlier_size) override {
        float inl_ratio = (float) inlier_size/points_size;
        float inl_prob = inl_ratio * inl_ratio;
        int k = sample_size;
        while (k > 2) {
            inl_prob *= inl_ratio;
            k--;
        }
        if (inl_prob < EPSILON) return max_iterations;
        return log_1_p/log(1 - inl_prob);
    }

    inline unsigned int getUpBoundIterations (unsigned int inlier_size, unsigned int points_size) {
        float inl_ratio = (float) inlier_size/points_size;
        float inl_prob = inl_ratio * inl_ratio;
        int k = sample_size;
        while (k > 2) {
            inl_prob *= inl_ratio;
            k--;
        }
        if (inl_prob < EPSILON) return max_iterations;
        return log_1_p/log(1 - inl_prob);
    }

    static inline unsigned int getUpBoundIterations (unsigned int inlier_size, unsigned int points_size, unsigned int sample_size, float desired_prob) {
        float inl_ratio = (float) inlier_size/points_size;
        float inl_prob = inl_ratio * inl_ratio;
        int k = sample_size;
        while (k > 2) {
            inl_prob *= inl_ratio;
            k--;
        }
        if (inl_prob < 0.00001) return 10000;
        return log (1 - desired_prob)/log(1 - inl_prob);
    }

};


#endif //RANSAC_STANDARD_TERMINATION_CRITERIA_H