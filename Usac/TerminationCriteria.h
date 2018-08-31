#ifndef RANSAC_TERMINATION_CRITERIA_H
#define RANSAC_TERMINATION_CRITERIA_H

class TerminationCriteria {
public:
    const Model * model;
public:
	TerminationCriteria(const Model * const model)  {
        this->model = model;
	}

	void setNewModel (const Model * const model) {
		this->model = model;
	}

	float getUpBoundIterations (float inlier_points, float total_points) {
//		CV_Assert((1-model->desired_prob) > 0);
//		CV_Assert((1-pow(inlier_points/total_points, model->Npoints)) > 0);
//		CV_Assert((log(1-model->desired_prob)/log(1-pow(inlier_points/total_points, model->Npoints))) > 0);
			
		return log(1-model->desired_prob)/log(1-pow(inlier_points/total_points, model->sample_number));
	}
};


#endif //RANSAC_TERMINATION_CRITERIA_H