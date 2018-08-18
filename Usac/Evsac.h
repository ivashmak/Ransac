#ifndef EVSAC_USAC_H
#define EVSAC_USAC_H

#include "Estimator/Estimator.h"
#include "Estimator/Line2DEstimator.h"
#include "Quality.h"
#include "Sampler/UniformSampler.h"

class Evsac {
protected:
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;

public:
    cv::Point_<float> *points;
    int total_points;

    std::vector<int> most_inliers;

    Model best_model;
    Model non_minimal_model;

    Evsac (cv::InputArray points,
            Model& model,
            Sampler& sampler,
            TerminationCriteria& termination_criteria,
            Quality& quality) {

        CV_Assert(!points.empty());

        this->points = (cv::Point_<float> *) points.getMat().data;
        this->total_points = points.size().width;

        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = &quality;
    }

    void run (cv::InputArray input_points1, cv::InputArray input_points2, Estimator *estimator2d);

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


#endif //EVSAC_RANSAC_H
