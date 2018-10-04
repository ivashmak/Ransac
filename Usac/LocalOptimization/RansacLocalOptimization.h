#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "LocalOptimization.h"
#include "../Quality.h"

/*
 * Reference:
 * http://cmp.felk.cvut.cz/~matas/papers/chum-dagm03.pdf
 */

class RansacLocalOptimization : public LocalOptimization {
private:
//    Model *lo_model;
//    Quality *quality;
//    TerminationCriteria *termination_criteria;
public:
    void GetLOModelScore (Estimator * const estimator,
                          Model &best_lo_model,
                          Sampler * const sampler,
                          Quality *const quality,
                          cv::InputArray input_points,
                          unsigned int points_size,
                          unsigned int best_sample_size,
                          const int * const inliers,
                          Score &best_lo_score) override {


        /*
         * Do not do local optimization for small number of inliers
         */
        if (best_sample_size < 2 * best_lo_model.lo_sample_size) {
            return;
        }

        /* In our experiments the size of samples are set to min (Ik/2, 14)
         * for epipolar geometry and to min (Ik/2, 12) for the case of homography estimation
         */
        unsigned int lo_sample_size = std::min((int) best_sample_size / 2 + 1, (int) best_lo_model.lo_sample_size);

        unsigned int lo_iterative_iterations = best_lo_model.lo_iterative_iterations;

        Model *current_lo_model = new Model (best_lo_model);
        Score *current_lo_score = new Score;

        current_lo_score->inlier_number = 0;
        current_lo_score->score = 0;
        best_lo_score.inlier_number = 0;
        best_lo_score.score = 0;
        /*
         * Generate sample of lo_sample_size from reached best_sample in current iteration
         */
        int *lo_sample = new int[lo_sample_size];
        sampler->generateSample(lo_sample, lo_sample_size, best_sample_size);
        for (int smpl = 0; smpl < lo_sample_size; smpl++) {
            lo_sample[smpl] = inliers[lo_sample[smpl]];
        }
        /*
         * Estimate model of best sample from k-th step of Ransac
         */
        estimator->EstimateModelNonMinimalSample(lo_sample, lo_sample_size, *current_lo_model);
        std::vector<int> current_inliers;
        // Evaluate model and get inliers
        quality->GetModelScore(estimator, current_lo_model, input_points, points_size, *current_lo_score, current_inliers, true);


        /*
         * reduce multiplier threshold K·θ by this number in each iteration.
         * In the last iteration there be original threshold θ.
         */
        float reduce_threshold = (current_lo_model->lo_threshold * current_lo_model->lo_threshold_multiplier - current_lo_model->threshold)
                                    / current_lo_model->lo_iterative_iterations;

        current_lo_model->threshold = current_lo_model->lo_threshold_multiplier * current_lo_model->lo_threshold;


        /*
         * Iterative Local Optimization
         * Reduce threshold of current model
         * Estimate model parametres
         * Evaluate model
         * Get inliers
         * Repeat until iteration < lo_iterations
         */
        for (int iteration = 0; iteration < lo_iterative_iterations; iteration++) {
            current_lo_model->threshold -= reduce_threshold;
            estimator->EstimateModelNonMinimalSample(&current_inliers[0], current_inliers.size(), *current_lo_model);

            current_inliers.clear();
            current_inliers.reserve(100);

            quality->GetModelScore(estimator, current_lo_model, input_points, points_size, *current_lo_score, current_inliers, true);

            if (quality->IsBetter (&best_lo_score, current_lo_score)) {
                best_lo_score.inlier_number = current_lo_score->inlier_number;
                best_lo_score.score = current_lo_score->score;

                best_lo_model.threshold = current_lo_model->threshold;
                best_lo_model.setDescriptor(current_lo_model->returnDescriptor());
            }
        }

    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
