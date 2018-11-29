#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H


#include "../Estimator/Estimator.h"
#include "../Quality/Quality.h"

class LocalOptimization {
public:
	virtual ~LocalOptimization () {}

	/*
	 * Returns as int number local optimization iterations including inner
	 * and iterative ransac.
	 * Returns also best lo model and best lo score.
	 */
    virtual int GetLOModelScore  (Model *best_lo_model,
                                  Score *lo_score,
                                  Score *kth_ransac_score,
                                  cv::InputArray input_points,
                                  unsigned int points_size,
                                  unsigned int kth_step,
                                  const int * const inliers,
                                  bool *can_finish) {
        return 0;
	}

};


#endif //USAC_LOCALOPTIMIZATION_H
