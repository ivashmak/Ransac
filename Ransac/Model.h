#ifndef RANSAC_MODEL_H
#define RANSAC_MODEL_H

class Model {
public:
	float threshold;
	int sample_number;
	float desired_prob;
	float max_iterations = 10000;
	std::string model_name;

protected:
    cv::Mat descriptor;
	
public:
	Model () {
		threshold = 10.0;
		sample_number = 2;
		desired_prob = 0.99;
	}

	Model (float threshold, int sample_number, float desired_prob, std::string model_name) {
		this->threshold = threshold;
		this->sample_number = sample_number;
		this->desired_prob = desired_prob;
		this->model_name = model_name;
	}

    void setDescriptor(cv::Mat _desc) { 
    	descriptor = _desc;
    }

    void getDescriptor(cv::Mat &_desc) { 
    	_desc = descriptor;
    }    

	void setThreshold (float threshold) {
		this->threshold = threshold;
	}

	void setsample_number (float sample_number) {
		this->sample_number = sample_number;
	}

	void setDesiredProbability (float desired_prob) {
		this->desired_prob = desired_prob;
	}

	void setModelName (std::string model_name) {
		this->model_name = model_name;
	}
};

#endif //RANSAC_MODEL_H