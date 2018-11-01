#ifndef RANSAC_MODEL_H
#define RANSAC_MODEL_H

enum ESTIMATOR  { Line2d, Homography, Fundamental, Essential };
enum SAMPLER  { Uniform, GradualNapsac, Napsac, Prosac, Evsac };

class Model {
public:
	float threshold = 10.0;
	int sample_number = 2;
	float desired_prob = 0.95;
	int max_iterations = 10000;
	int k_nearest_neighbors = 2;

	/*
	 * |expected number of inliers|
	 * -----------------------------
	 *          |points size|
	 */
	float estimated_inliers_ratio = 1; // 0.8

	/*
	 * Local Optimization parameters
	 */
	unsigned int lo_sample_size = 14;
	unsigned int lo_iterative_iterations = 4;
    unsigned int lo_max_iterations = 5;
    float lo_threshold = 10.0;
    unsigned int lo_threshold_multiplier = 15;

    // Graph cut
    float lambda_graph_cut = 0.1; // range <0; 1>

    // name
    ESTIMATOR estimator;
    SAMPLER sampler;
private:
    cv::Mat descriptor;
	
public:
	~Model () {}
	Model () {}
	
	Model (float threshold_, int sample_number_, float desired_prob_, int knn,
            ESTIMATOR estimator_, SAMPLER sampler_) {
		threshold = threshold_;
		lo_threshold = threshold_;
		sample_number = sample_number_;
		desired_prob = desired_prob_;
		k_nearest_neighbors = knn;
		estimator = estimator_;
		sampler = sampler_;
	}

	void setLOParametres (unsigned int lo_iterative_iters, unsigned int lo_max_iters, float lo_thresh, unsigned int lo_thresh_mult) {
	    lo_iterative_iterations = lo_iterative_iters;
	    lo_max_iterations = lo_max_iters;
	    lo_threshold = lo_thresh;
        lo_threshold_multiplier = lo_thresh_mult;
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

	void setKNearestNeighbors (int k_nearest_neighbors) {
	    this->k_nearest_neighbors = k_nearest_neighbors;
	}
	void copyFrom (const Model * const model) {
        threshold = model->threshold;
        sample_number = model->sample_number;
        desired_prob = model->desired_prob;
        max_iterations = model->max_iterations;
        estimator = model->estimator;
        sampler = model->sampler;
        k_nearest_neighbors = model->k_nearest_neighbors;
        estimated_inliers_ratio = model->estimated_inliers_ratio;
        lo_sample_size = model->lo_sample_size;
        lo_iterative_iterations = model->lo_iterative_iterations;
        lo_max_iterations = model->lo_max_iterations;
        lo_threshold = model->lo_threshold;
        lo_threshold_multiplier = model->lo_threshold_multiplier;
//        descriptor = model->descriptor;
	}

	std::string getName () {
	    std::string name;
	    if (estimator == ESTIMATOR::Homography) {
	        name = "Homography";
	    } else if (estimator == ESTIMATOR::Essential) {
	        name = "Essential";
	    } else if (estimator == ESTIMATOR::Fundamental) {
	        name = "Fundamental";
	    } else if (estimator == ESTIMATOR::Line2d) {
	        name = "Line2d";
	    }
	    name += "_estimator_";
	    if (sampler == SAMPLER::Evsac) {
	        name += "Evsac";
	    } else if (sampler == SAMPLER::Uniform) {
	        name += "Uniform";
	    } else if (sampler == SAMPLER::Prosac) {
	        name += "Prosac";
	    } else if (sampler == SAMPLER ::Napsac) {
	        name += "Napsac";
	    } else if (sampler == SAMPLER ::GradualNapsac) {
	        name += "GradualNapsacSampler";
	    }
	    name += "_sampler";
        return name;
	}
};

#endif //RANSAC_MODEL_H