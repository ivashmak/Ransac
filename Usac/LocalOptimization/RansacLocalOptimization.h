#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "LocalOptimization.h"
#include "../Quality/Quality.h"
#include "../Helper/Drawing/Drawing.h"
#include "../Sampler/UniformSampler.h"

/*
 * Reference:
 * http://cmp.felk.cvut.cz/~matas/papers/chum-dagm03.pdf
 */

class RansacLocalOptimization : public LocalOptimization {
private:
    Model * model;
    Quality * quality;
    TerminationCriteria * termination_criteria;
    Sampler * sampler;
    Estimator * estimator;
    unsigned int points_size;

public:
    unsigned int lo_inner_iters = 0;
    unsigned int lo_iterative_iters = 0;

    virtual ~RansacLocalOptimization () {
        // std::cout << "cleaning in ransac lo\n";
    }

    RansacLocalOptimization (Model *model_,
                             Sampler *sampler_,
                             TerminationCriteria *termination_criteria_,
                             Quality *quality_,
                             Estimator *estimator_,
                             unsigned int points_size_) {
        model = model_;
//        sampler = sampler_;
        sampler = new UniformSampler;

        estimator = estimator_;
        termination_criteria = termination_criteria_;
        quality = quality_;
        points_size = points_size_;
    }
    
    void GetLOModelScore  (Model *best_lo_model,
                           Score *best_lo_score,
                           Score *kth_ransac_score,
                           const int * const inliers) override {
        /*
         * Do not do local optimization for small number of inliers
         */
//        if (kth_ransac_score->inlier_number < 2 * model->lo_sample_size) {
//            return;
//        }
        if (kth_ransac_score->inlier_number < 2 * model->sample_number) {
            return;
        }

        /*
         * Assume best lo score is at least kth ransac score.
         * Avoid comparing between current lo score with best lo score and kth score.
         */
        best_lo_score->inlier_number = kth_ransac_score->inlier_number;
        best_lo_score->score = kth_ransac_score->score;

        /* In our experiments the size of samples are set to min (Ik/2, 14)
         * for epipolar geometry and to min (Ik/2, 12) for the case of homography estimation
         */
        unsigned int lo_sample_size = std::min(kth_ransac_score->inlier_number / 2 + 1, (int) best_lo_model->lo_sample_size);
        unsigned int lo_inner_iterations = best_lo_model->lo_inner_iterations;
        unsigned int lo_iterative_iterations = best_lo_model->lo_iterative_iterations;

        // std::cout << "lo sample_size " << lo_sample_size << '\n';


        Model *lo_model = new Model;
        lo_model->copyFrom (model);
        Score *lo_score = new Score;
        

        int * lo_sample = new int[lo_sample_size];
        int * lo_inliers = new int[points_size];
        int * max_lo_inliers = new int [points_size];

        sampler->setSampleSize(lo_sample_size);
        sampler->setPointsSize(kth_ransac_score->inlier_number);
        sampler->initRandomGenerator();

        /*
         * reduce multiplier threshold K·θ by this number in each iteration.
         * In the last iteration there be original threshold θ.
         */
        float reduce_threshold = (lo_model->lo_threshold * lo_model->lo_threshold_multiplier -
                                  lo_model->threshold)
                                 / lo_model->lo_iterative_iterations;

        bool lo_better_than_kth_ransac = false;
        /*
         * Inner Local Optimization Ransac
         */
//        std::cout << "lo inner " << lo_max_iterations << "\n";
        for (int iters = 0; iters < lo_inner_iterations; iters++) {
            /*
             * Generate sample of lo_sample_size from inliers from best model of standart 
             * Ransac or from inliers of best LO model.
             */
            sampler->generateSample(lo_sample);
            if (lo_better_than_kth_ransac) {
                for (int smpl = 0; smpl < lo_sample_size; smpl++) {
                   lo_sample[smpl] = max_lo_inliers[lo_sample[smpl]];
                }
            } else {
                for (int smpl = 0; smpl < lo_sample_size; smpl++) {
                    lo_sample[smpl] = inliers[lo_sample[smpl]];
                }
            }

            // multiply threshold K * θ
            lo_model->threshold = lo_model->lo_threshold_multiplier * lo_model->lo_threshold;
            // estimate model from sample
            if (!estimator->LeastSquaresFitting(lo_sample, lo_sample_size, *lo_model)) continue;

            // Start evaluating a model with new threshold. And get inliers for iterative lo ransac.
            quality->getNumberInliers(lo_score, lo_model, true, lo_inliers);
//            std::cout << lo_score->inlier_number << " = |I|\n";

            // continue, if model is not good enough for iterative ransac
            if (lo_score->inlier_number < model->sample_number) {
                continue;
            }

            /*
             * Iterative LO Ransac
             * Reduce threshold of current model
             * Estimate model parametres
             * Evaluate model
             * Get inliers
             * Repeat until iteration < lo iterative iterations
             */

            // cv::Scalar color = cv::Scalar (random()%256,random()%256,random()%256);

            for (int iterations = 0; iterations < lo_iterative_iterations; iterations++) {
                lo_model->threshold -= reduce_threshold;
//                std::cout << lo_model->threshold << " = threshold\n";
                // estimate model from inliers
                if (!estimator->LeastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) continue;
                quality->getNumberInliers(lo_score, lo_model, true, lo_inliers);

//                std::cout << "lo iterative score  = " << lo_score->inlier_number << '\n';
//                std::cout << "lo model  = " << lo_model->returnDescriptor() << '\n';


                // only for test
                lo_iterative_iters++;
                //

                /*
                 * if current model is not better then break.
                 * After decreasing threshold it would not be better.
                 */
                if (best_lo_score->bigger(lo_score)) {
                    break;
                }
            }
            // get original threshold back in case lo iterative ransac had break.
            lo_model->threshold = best_lo_model->threshold;

            if (lo_score->bigger(best_lo_score)) {
                best_lo_model->setDescriptor(lo_model->returnDescriptor());
                best_lo_score->copyFrom (lo_score);
                lo_better_than_kth_ransac = true;
                std::copy (lo_inliers, lo_inliers+lo_score->inlier_number, max_lo_inliers);
                sampler->setPointsSize(lo_score->inlier_number);
                sampler->initRandomGenerator();
            }

            // only for test
            lo_inner_iters++;
            //
        }

        // reinit sampler back
        sampler->setSampleSize(model->sample_number);
        sampler->setPointsSize(points_size);
        sampler->initRandomGenerator();

        delete lo_score, lo_model, lo_sample, lo_inliers;
    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
