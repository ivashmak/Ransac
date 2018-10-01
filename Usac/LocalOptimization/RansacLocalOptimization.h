#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "LocalOptimization.h"

class RansacLocalOptimization : public LocalOptimization {
private:
    Model *lo_model;
    Quality *quality;
    TerminationCriteria *termination_criteria;
public:
    void GetLOModelScore (Estimator * const estimator,
                          Model &lo_model,
                          Sampler * const sampler,
                          Quality *const quality,
                          cv::InputArray input_points,
                          unsigned int points_size,
                          unsigned int lo_sample_size,
                          unsigned int best_sample_size,
                          const int * const inliers,
                          Score &lo_score) override {
        /*
         * Generate sample of lo_sample_size from reached best_sample in current iteration
         */
        int *lo_sample = new int [lo_sample_size];
        sampler->generateSample(lo_sample, lo_sample_size, best_sample_size);
        for (int smpl = 0; smpl < lo_sample_size; smpl++) {
            lo_sample[smpl] = inliers[lo_sample[smpl]];
        }
        /*
         * Estimate new model
         */
        estimator->EstimateModelNonMinimalSample(lo_sample, lo_sample_size, lo_model);

        /*
         * Get score of this model
         */
        quality->GetModelScore(estimator, lo_model, input_points, points_size, lo_score,
                               (std::vector &) nullptr, false);
    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
