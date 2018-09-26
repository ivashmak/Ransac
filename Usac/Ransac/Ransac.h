#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "../Estimator/Estimator.h"
#include "../Estimator/Line2DEstimator.h"
#include "../Quality.h"
#include "../Sampler/Sampler.h"

#include "../Verbose.h"

class Ransac {
protected:
    /*
     * Initialize them to 0 to check if they are null
     */
    Model *model = 0;
    Quality *quality = 0;
    Sampler *sampler = 0;
    TerminationCriteria *termination_criteria = 0;
public:

    Model best_model;
    Model non_minimal_model;

    std::vector<int> most_inliers;

    Ransac (Model &model,
            Sampler& sampler,
            TerminationCriteria& termination_criteria) {


        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = new Quality;
    }

    void run (cv::InputArray input_points, Estimator * const estimator2d);

    void setSampler (Sampler &sampler) {
        this->sampler = &sampler;
    }

    Sampler& getUniformSamler () {
        return *sampler;
    }

    void setModel (Model &model) {
        this->model = &model;
    }

    Model getModel () {
        return *model;
    }

    void setTerminationCriteria (TerminationCriteria &termination_criteria) {
        this->termination_criteria = &termination_criteria;
    }

    TerminationCriteria getTerminationCriteria () {
        return *termination_criteria;
    }

    void setQuality (Quality& quality) {
        this->quality = &quality;
    }

    Quality* getQuality () {
        return quality;
    }

//    Model* getBestModel () {
//        return &best_model;
//    }
//
//    Model* getNonMinimalModel () {
//        return &non_minimal_model;
//    }
};



#endif //RANSAC_RANSAC_H
