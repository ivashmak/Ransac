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
    Model * model;
    Quality * quality;
    TerminationCriteria * termination_criteria;
    Sampler *sampler;
    Estimator * estimator;
    
public:

    RansacLocalOptimization (Model * model_,
                             Sampler *sampler_,
                             TerminationCriteria *termination_criteria_,
                             Quality *quality_,
                             Estimator *estimator_) {
        model = model_;
        sampler = sampler_;
        estimator = estimator_;
        termination_criteria = termination_criteria_;
        quality = quality_;
    }
    
    void GetLOModelScore (Model &best_lo_model,
                          Score &best_lo_score,
                          Score *kth_ransac_score,
                          cv::InputArray input_points,
                          unsigned int points_size,
                          const int * const inliers) override {

        /*
         * Do not do local optimization for small number of inliers
         */
        if (kth_ransac_score->inlier_number < 2 * model->lo_sample_size) {
            return;
        }

        /*
         * Let's best lo score is at least kth ransac score.
         * Avoid comparing between current lo score with best score and kth score.
         */
        best_lo_score.inlier_number = kth_ransac_score->inlier_number;
        best_lo_score.score = kth_ransac_score->score;

//        std::cout << "Run local optimization.\n";

        /* In our experiments the size of samples are set to min (Ik/2, 14)
         * for epipolar geometry and to min (Ik/2, 12) for the case of homography estimation
         */
        unsigned int lo_sample_size = std::min(kth_ransac_score->inlier_number / 2 + 1, (int) best_lo_model.lo_sample_size);
        unsigned int lo_max_iterations = best_lo_model.lo_max_iterations;
        unsigned int lo_iterative_iterations = best_lo_model.lo_iterative_iterations;

        Model *lo_model = new Model(*model);
        Score *lo_score = new Score;
        
        int *lo_sample = new int[lo_sample_size];
        int * current_inliers = new int[points_size];

//        std::cout << "lo sample size " << lo_sample_size << '\n';

        sampler->setSampleSize(lo_sample_size);
        sampler->setPointsSize(kth_ransac_score->inlier_number);
        sampler->initRandomGenerator();

        float reduce_threshold = (lo_model->lo_threshold * lo_model->lo_threshold_multiplier -
                                  lo_model->threshold)
                                 / lo_model->lo_iterative_iterations;

        /*
         * Inner Local Optimization Ransac
         */
        for (int iters = 0; iters < lo_max_iterations; iters++) {

            /*
             * Generate sample of lo_sample_size from reached best_sample in current iteration
             */
            sampler->generateSample(lo_sample);
            for (int smpl = 0; smpl < lo_sample_size; smpl++) {
                lo_sample[smpl] = inliers[lo_sample[smpl]];
            }

//            current_inliers = lo_sample;
//            lo_score->inlier_number = lo_sample_size;

            /*
             * Estimate model of best sample from k-th step of Ransac
             */
            estimator->EstimateModelNonMinimalSample(lo_sample, lo_sample_size, *lo_model);
            estimator->setModelParameters(lo_model);
//            std::cout << lo_model->returnDescriptor() << '\n';

            // Evaluate model and get inliers
            quality->GetModelScore(estimator, lo_model, input_points, points_size, *lo_score, current_inliers, true);

            std::cout << "lo inner score = " << lo_score->score << '\n';

            /*
             * reduce multiplier threshold K·θ by this number in each iteration.
             * In the last iteration there be original threshold θ.
             */

            lo_model->threshold = lo_model->lo_threshold_multiplier * lo_model->lo_threshold;

            /*
             * Iterative LO Ransac
             * Reduce threshold of current model
             * Estimate model parametres
             * Evaluate model
             * Get inliers
             * Repeat until iteration < lo_iterations
             */
            Model ** lo_models = new Model*[1];
            lo_models[0] = lo_model;

            for (int iterations = 0; iterations < lo_iterative_iterations; iterations++) {
                lo_model->threshold -= reduce_threshold;
//                std::cout << "begin lo score " << lo_score->inlier_number << '\n';

                estimator->EstimateModelNonMinimalSample(current_inliers, lo_score->inlier_number, *lo_model);
//                estimator->EstimateModel(current_inliers, lo_models);
                estimator->setModelParameters(lo_model);

                quality->GetModelScore(estimator, lo_model, input_points, points_size, *lo_score, current_inliers, true);
                std::cout << "lo iterative score  = " << lo_score->inlier_number << '\n';

                // if current model is not better then break
                if (best_lo_score > lo_score) {
                    break;
                }
            }

            std::cout << "lo inner score  = " << lo_score->inlier_number << '\n';

            if (*lo_score > best_lo_score) {
                best_lo_model.setDescriptor(lo_model->returnDescriptor());
                best_lo_score.inlier_number = lo_score->inlier_number;
                best_lo_score.score = lo_score->score;
            }
        }

        sampler->setSampleSize(model->sample_number);
        sampler->setPointsSize(points_size);
        sampler->initRandomGenerator();

        /*
         * TODO: add weights?
         */
    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
