#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "../Estimator/Estimator.h"
#include "../Estimator/Line2DEstimator.h"
#include "../Quality.h"
#include "../Sampler/Sampler.h"

class Ransac {
protected:
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;

public:
    std::vector<int> most_inliers;

    Model best_model;
    Model non_minimal_model;

    Ransac (Model &model,
            Sampler& sampler,
            TerminationCriteria& termination_criteria,
            Quality& quality) {

        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = &quality;
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

    Quality getQuality () {
        return *quality;
    }
};


#endif //RANSAC_RANSAC_H
