#ifndef RANSAC_TERMINATION_CRITERIA_H
#define RANSAC_TERMINATION_CRITERIA_H


class TerminationCriteria {
private:
    const Model * model;
    float log_1_p;
    int sample_number;
    bool initialized = false;
    unsigned int max_iterations = 10000;
    const float EPSILON = 0.00001;
public:

    void init (const Model * const model) {
        assert (model != nullptr);
        setModel(model);
        initialized = true;
    }

    /*
     * Model setter
     * Saves time of accessing to model object and recomputing same values.
     */
    void setModel (const Model * const model) {
		this->model = model;
        log_1_p = (float) log (1-model->desired_prob);
        sample_number = model->sample_number;
        max_iterations = model->max_iterations;
    }

    /*
     * Can happen that getUpBoundIterations could be called on uninitialized values. Checker function.
     */
    bool isModelInitialized () { return initialized; }


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
	 */
    inline unsigned int getUpBoundIterations (float inlier_points, float total_points) {
        float inl_ratio = inlier_points/total_points;
        float inl_prob = inl_ratio * inl_ratio;
        int k = sample_number;
        while (k > 2) {
            inl_prob *= inl_ratio;
            k--;
        }
        if (inl_prob < EPSILON) return max_iterations;
        return log_1_p/log(1 - inl_prob);
    }
    
    inline unsigned int getUpBoundIterations (float inlier_points, float total_points, unsigned int sample_number) {
        float inl_prob = (inlier_points/total_points) * (inlier_points/total_points);
        int k = sample_number;
        while (k > 2) {
            inl_prob *= (inlier_points/total_points);
            k--;
        }
        if (inl_prob < EPSILON) return max_iterations;
        return log_1_p/log(1 - inl_prob);
    }

    inline unsigned int getUpBoundIterations (float inlier_points, float total_points, unsigned int sample_number, float desired_prob) {
        float inl_prob = (inlier_points/total_points) * (inlier_points/total_points);
        int k = sample_number;
        while (k > 2) {
            inl_prob *= (inlier_points/total_points);
            k--;
        }
        if (inl_prob < EPSILON) return max_iterations;
        return log (1 - desired_prob)/log(1 - inl_prob);
    }

};


#endif //RANSAC_TERMINATION_CRITERIA_H