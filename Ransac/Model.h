#ifndef RANSAC_MODEL_H
#define RANSAC_MODEL_H

class Model {
public:
	float threshold;
	float Npoints;
	float desired_prob;
	float max_iterations = 10000;
	std::string model_name;
	
public:
	Model () {
		threshold = 10.0;
		Npoints = 2;
		desired_prob = 0.99;
	} 

	Model (float threshold, float Npoints, float desired_prob, std::string model_name) {
		this->threshold = threshold;
		this->Npoints = Npoints;
		this->desired_prob = desired_prob;
		this->model_name = model_name;
	}

	void setThreshold (float threshold) {
		this->threshold = threshold;
	}

	void setNpoints (float Npoints) {
		this->Npoints = Npoints;
	}

	void setDesiredProbability (float desired_prob) {
		this->desired_prob = desired_prob;
	}

	void setModelName (std::string model_name) {
		this->model_name = model_name;
	}
};

#endif //RANSAC_MODEL_H