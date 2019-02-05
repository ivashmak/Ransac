// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_INIT_H
#define RANSAC_INIT_H

#include "../estimator/line2d_estimator.hpp"
#include "../estimator/homography_estimator.hpp"
#include "../estimator/essential_estimator.hpp"
#include "../estimator/fundamental_estimator.hpp"

#include "../sampler/prosac_sampler.hpp"
#include "../sampler/napsac_sampler.hpp"
#include "../sampler/uniform_sampler.hpp"

#include "../termination_criteria/prosac_termination_criteria.hpp"



void InitEstimator (Estimator *& estimator, ESTIMATOR est, const cv::Mat& points);
// ----------------------------------------------------------------------------------------
void InitSampler (Sampler *& sampler, const Model * const model, cv::InputArray points, cv::InputArray neighbors);
// ----------------------------------------------------------------------------------------
void InitTerminationCriteria (TerminationCriteria *& termination_criteria, 
        const Model * const model, unsigned int points_size);
// ----------------------------------------------------------------------------------------
void InitProsacTerminationCriteria (TerminationCriteria *& termination_criteria, Sampler * prosac_sampler, 
            const Model * const model, Estimator * estimator, unsigned int points_size);
// ----------------------------------------------------------------------------------------




#endif //RANSAC_INIT_H
