#ifndef RANSAC_MODEL_H
#define RANSAC_MODEL_H

class Model {
public:
	float threshold = 10.0;
	int sample_number = 2;
	float desired_prob = 0.95;
	float max_iterations = 10000;
	std::string model_name = "ransac";
	int k_nearest_neighbors = 2;
    int N_points = 0;

protected:
    cv::Mat descriptor;
	
public:
    Model () {}

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

    cv::Mat returnDescriptor () {
        return descriptor;
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

	void setKNearestNeighbors (int k_nearest_neighbors) {
	    this->k_nearest_neighbors = k_nearest_neighbors;
	}
};

#endif //RANSAC_MODEL_H