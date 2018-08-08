#ifndef RANSAC_RANSAC_H
#define RANSAC_RANSAC_H

#include "Estimator.h"
#include "Line2DEstimator.h"

class Ransac {
public:
    Line2DEstimator *estimator;
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;

    cv::Point_<float> *points;
    int total_points;

    Ransac (cv::InputArray points,
            Model& model,
            Sampler& sampler,
            TerminationCriteria& termination_criteria,
            Quality& quality,
            Line2DEstimator &estimator) {

        CV_Assert(!points.empty());

        this->estimator = &estimator;

        this->points = (cv::Point_<float> *) points.getMat().data;
        this->total_points = points.size().width;

        this->model = &model;
        this->sampler = &sampler;
        this->termination_criteria = &termination_criteria;
        this->quality = &quality;
    }

    void run (cv::InputArray input_points, cv::OutputArray &line);
};


#endif //RANSAC_RANSAC_H
