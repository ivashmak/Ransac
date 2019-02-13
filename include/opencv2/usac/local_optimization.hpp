// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H

#include "estimator.hpp"
#include "quality.hpp"

namespace cv { namespace usac {
class LocalOptimization {
public:
    virtual ~LocalOptimization() = default;

    /*
     * Update best model and best score.
     */
    virtual void GetModelScore(Model *best_model, Score *best_score) = 0;

};
}}

#endif //USAC_LOCALOPTIMIZATION_H
