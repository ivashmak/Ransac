#ifndef RANSAC_TERMINATION_CRITERIA_H
#define RANSAC_TERMINATION_CRITERIA_H

class TerminationCriteria {
public:
    const Model * model;
public:
	TerminationCriteria(const Model * const model)  {
        this->model = model;
	}

	void setModel (const Model * const model) {
		this->model = model;
	}

	float getUpBoundIterations (float inlier_points, float total_points) {
		return log(1-model->desired_prob)/log(1-pow(inlier_points/total_points, model->sample_number));
	}
};


#endif //RANSAC_TERMINATION_CRITERIA_H