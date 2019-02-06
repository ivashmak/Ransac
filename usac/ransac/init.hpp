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
#include "../sampler/progressive_sampler.hpp"
#include "../sampler/evsac_sampler.hpp"

#include "../termination_criteria/prosac_termination_criteria.hpp"

#include "../local_optimization/inner_local_optimization.hpp"
#include "../local_optimization/irls.hpp"
#include "../local_optimization/graphcut.hpp"



void initEstimator (Estimator *& estimator, ESTIMATOR est, const cv::Mat& points);
// ----------------------------------------------------------------------------------------
void initSampler (Sampler *& sampler, const Model * const model, const cv::Mat& points);
// ----------------------------------------------------------------------------------------
void initTerminationCriteria (TerminationCriteria *& termination_criteria,
        const Model * const model, unsigned int points_size);

void initProsacTerminationCriteria (TerminationCriteria *& termination_criteria, Sampler *& prosac_sampler,
            const Model * const model, Estimator * estimator, unsigned int points_size);
// ----------------------------------------------------------------------------------------
void initLocalOptimization (LocalOptimization *& local_optimization, Model * model, Estimator * estimator,
    Quality * quility, unsigned int points_size);




#endif //RANSAC_INIT_H
