#ifndef USAC_LOCALOPTIMIZATION_H
#define USAC_LOCALOPTIMIZATION_H


#include "../Estimator/Estimator.h"
#include "../Quality.h"

class LocalOptimization {
public:
    virtual void GetLOModelScore (Estimator * const estimator,
                                  Model &lo_model,
                                  Sampler * const sampler,
                                  Quality *const quality,
                                  cv::InputArray input_points,
                                  unsigned int points_size,
                                  unsigned int lo_sample_size,
                                  unsigned int best_sample_size,
                                  const int * const inliers,
                                  Score &lo_score) = 0;
};


#endif //USAC_LOCALOPTIMIZATION_H
