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
     * Can happen that getUpBoundIterations could be called on uninitialized values. Check function.
     */
    bool isModelInitialized () { return initialized; }


    /*
     * Faster way than calculation with pow
     */
    inline float getUpBoundIterations2Points (float inlier_points, float total_points) {
		return log_1_p/log(1 - ((inlier_points*inlier_points)/(total_points*total_points)));
	}

    inline float getUpBoundIterations (float inlier_points, float total_points) {
        return log_1_p/log(1 - my_pow((inlier_points/total_points), sample_number));
    }

};


#endif //RANSAC_TERMINATION_CRITERIA_H