// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H


#include "../estimator/estimator.hpp"
#include "../quality/quality.hpp"

class LocalOptimization {
public:
	virtual ~LocalOptimization () = default;

	/*
	 * Returns as int number local optimization iterations including inner
	 * and iterative ransac.
	 * Returns also best lo model and best lo score.
	 */
    virtual void GetModelScore  (Model * best_model,
                                   Score * best_score) = 0;

};


#endif //USAC_LOCALOPTIMIZATION_H
