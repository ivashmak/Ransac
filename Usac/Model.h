#ifndef RANSAC_MODEL_H
#define RANSAC_MODEL_H

class Model {
public:
	float threshold = 10.0;
	int sample_number = 2;
	float desired_prob = 0.95;
	int max_iterations = 10000;
	std::string model_name = "ransac";
	int k_nearest_neighbors = 2;

	/*
	 * Local Optimization parameters
	 */
	unsigned int lo_sample_size = 14;
	unsigned int lo_iterative_iterations = 4;
    unsigned int lo_max_iterations = 10;
    float lo_threshold = 10.0;
    unsigned int lo_threshold_multiplier = 2;

protected:
    cv::Mat descriptor;
	
public:
	Model () {}
	Model (float threshold, int sample_number, float desired_prob, int knn, std::string model_name) {
		this->threshold = threshold;
		this->sample_number = sample_number;
		this->desired_prob = desired_prob;
		this->k_nearest_neighbors = knn;
		this->model_name = model_name;
	}

	void setLOParametres (unsigned int lo_iterative_iters, unsigned int lo_max_iters, float lo_thresh, unsigned int lo_thresh_mult) {
	    lo_iterative_iterations = lo_iterative_iters;
	    lo_max_iterations = lo_max_iters;
	    lo_threshold = lo_thresh;
	    lo_threshold_mutliplier = lo_thresh_mult;
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