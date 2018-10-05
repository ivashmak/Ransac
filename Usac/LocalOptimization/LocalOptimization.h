#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H


#include "../Estimator/Estimator.h"
#include "../Quality.h"

class LocalOptimization {
public:
    virtual void GetLOModelScore (Model &best_lo_model,
                                  Score &lo_score,
                                  Score *kth_ransac_score,
                                  cv::InputArray input_points,
                                  unsigned int points_size,
                                  const int * const inliers) = 0;
};


#endif //USAC_LOCALOPTIMIZATION_H
