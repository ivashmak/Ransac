#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "LocalOptimization.h"

class RansacLocalOptimization : public LocalOptimization {
public:
    void GetLOModelScore (Estimator * const estimator,
                          Model * const model,
                          Sampler * const sampler,
                          Quality *const quality,
                          cv::InputArray input_points,
                          unsigned int points_size,
                          unsigned int lo_sample_size,
                          unsigned int best_sample_size,
                          const int * const inliers,
                          Score &lo_score) override {
        /*
         * In our experiments the size of samples are set to min (Ik/2, 14)
         * for epipolar geometry and to min (Ik/2, 12) for the case of homography estimation
         *
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
        Model *lo_model = new Model(model->threshold, model->sample_number, model->desired_prob, model->k_nearest_neighbors, model->model_name);
        estimator->EstimateModelNonMinimalSample(lo_sample, lo_sample_size, *lo_model);

        /*
         * Get quality of this model
         */
        quality->GetModelScore(estimator, lo_model, input_points, points_size, lo_score,
                               (std::vector &) nullptr, false);
    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
