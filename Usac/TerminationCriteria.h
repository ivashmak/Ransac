#ifndef RANSAC_TERMINATION_CRITERIA_H
#define RANSAC_TERMINATION_CRITERIA_H


class TerminationCriteria {
private:
    const Model * model;
    float log_1_p;
    int sample_number;
    bool initialized = false;

    /*
     * Declare my_pow function as pow is very slow
     * pow(x,y) = e^(y log(x))
     */
    float my_pow (float n, int k) {
        float res = n;
        while (k > 1) {
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
    }

    /*
     * Can happen that getUpBoundIterations could be called on uninitialized values. Checker function.
     */
    bool isModelInitialized () { return initialized; }


	/*
	 * Get upper bound iterations for any sample number
	 * Can be used ceil.
	 */
    inline unsigned int getUpBoundIterations (float inlier_points, float total_points) {
        return log_1_p/log(1 - my_pow((inlier_points/total_points), sample_number));
    }

    inline unsigned int getUpBoundIterations (float inlier_points, float total_points, unsigned int sample_number, float desir_prob) {
        return log(1-desir_prob)/log(1 - my_pow((inlier_points/total_points), sample_number));
    }

};


#endif //RANSAC_TERMINATION_CRITERIA_H