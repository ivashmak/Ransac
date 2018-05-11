#ifndef RANSAC_NAIVE_RANSAC_H
#define RANSAC_NAIVE_RANSAC_H

#include "Estimator.h"


bool fit_point_between_lines (float x, float y, float k, float b1, float b2);

class NaiveRansac : public Estimator {
    public:
        NaiveRansac (cv::InputArray points, 
                     Model& model, 
                     Sampler& sampler, 
                     TerminationCriteria& termination_criteria, 
                     Quality& quality) : Estimator(points, model, sampler, termination_criteria, quality) {}

        Line getBestLineFit (void) override;

};

#endif //RANSAC_NAIVE_RANSAC_H