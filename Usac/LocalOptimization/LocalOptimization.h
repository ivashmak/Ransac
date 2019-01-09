#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H


#include "../Estimator/Estimator.h"
#include "../Quality/Quality.h"

class LocalOptimization {
public:
	virtual ~LocalOptimization () = default;

	/*
	 * Returns as int number local optimization iterations including inner
	 * and iterative ransac.
	 * Returns also best lo model and best lo score.
	 */
    virtual void GetLOModelScore  (Model * best_model,
                                   Score * best_score) {}

};


#endif //USAC_LOCALOPTIMIZATION_H
