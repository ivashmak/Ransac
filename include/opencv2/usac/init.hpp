// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef RANSAC_INIT_H
#define RANSAC_INIT_H

#include "line2d_estimator.hpp"
#include "homography_estimator.hpp"
#include "essential_estimator.hpp"
#include "fundamental_estimator.hpp"

#include "prosac_sampler.hpp"
#include "napsac_sampler.hpp"
#include "uniform_sampler.hpp"

#include "prosac_termination_criteria.hpp"

#include "inner_local_optimization.hpp"
#include "graphcut.hpp"


namespace cv { namespace usac {
void initEstimator(Estimator *&estimator, ESTIMATOR est, const cv::Mat &points);

// ----------------------------------------------------------------------------------------
void initSampler(Sampler *&sampler, const Model *const model, const cv::Mat &points);

// ----------------------------------------------------------------------------------------
void initTerminationCriteria(TerminationCriteria *&termination_criteria,
                             const Model *const model, unsigned int points_size);

void initProsacTerminationCriteria(TerminationCriteria *&termination_criteria, Sampler *&prosac_sampler,
                                   const Model *const model, Estimator *estimator, unsigned int points_size);

// ----------------------------------------------------------------------------------------
void initLocalOptimization(LocalOptimization *&local_optimization, Model *model, Estimator *estimator,
                           Quality *quility, unsigned int points_size);
}}



#endif //RANSAC_INIT_H
