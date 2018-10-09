#ifndef RANSAC_TERMINATION_CRITERIA_H
#define RANSAC_TERMINATION_CRITERIA_H


class TerminationCriteria {
private:
    const Model * model;
    float log_1_p;
    int sample_number;
    bool initialized = false;
    unsigned int max_iterations = 10000;
    /*
     * Declare my_pow function as pow is very slow
     * https://stackoverflow.com/questions/41072787/why-is-powint-int-so-slow/41072811
     * pow(x,y) = e^(y log(x))
     * Assume that my pow is called for power at least 2.
     */
    float my_pow (float n, int k) {
        float res = n * n;
        while (k > 2) {
            res *= n; k--;
        }
        return res;
    }
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
	 * If (inls/total_pts) almost 0, so under logarithm is almost 1, so denominator is almost 0
	 * In this case just return max iterations from model parameters or 10000 if model is not defined.
	 * Otherwise upper bound iterations will not fit unsigned int type and will be (unsigned int) -inf = 0
	 */
    inline unsigned int getUpBoundIterations (float inlier_points, float total_points) {
        float inl_prob = (inlier_points/total_points) * (inlier_points/total_points);
        int k = sample_number;
        while (k > 2) {
            inl_prob *= (inlier_points/total_points);
            k--;
        }
        if (inl_prob < 0.00001) return max_iterations;
        return log_1_p/log(1 - inl_prob);
    }

    inline unsigned int getUpBoundIterations (float inlier_points, float total_points, unsigned int sample_number, float desir_prob) {
        this->sample_number = sample_number;
        log_1_p = log(1-desir_prob);
        return getUpBoundIterations(inlier_points, total_points);
    }

};


#endif //RANSAC_TERMINATION_CRITERIA_H