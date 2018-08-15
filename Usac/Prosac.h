#ifndef RANSAC_PROSAC_H
#define RANSAC_PROSAC_H


#include "Estimator.h"
#include "Quality.h"

class Prosac {
public:
    Model *model;
    Quality *quality;
    Sampler *sampler;
    TerminationCriteria *termination_criteria;

    cv::Point_<float> *points;
    int total_points;

    std::vector<int> inliers;
    std::vector<int> most_inliers;

    Model best_model;
    Model non_minimal_model;

    Prosac (cv::InputArray points,
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

    void run (cv::InputArray input_points, Estimator *estimator2d );
};


#endif //RANSAC_PROSAC_H
