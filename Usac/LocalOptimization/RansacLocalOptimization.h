#ifndef USAC_RANSACLOCALOPTIMIZATION_H
#define USAC_RANSACLOCALOPTIMIZATION_H

#include "LocalOptimization.h"
#include "../Quality/Quality.h"
#include "../Helper/Drawing/Drawing.h"
#include "../Sampler/UniformSampler.h"
#include "../../RandomGenerator/UniformRandomGenerator.h"

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

    int * inliers;
    int * lo_inliers;
    int * max_lo_inliers;
    int * lo_sample;

    Score * lo_score;
    Model * lo_model;

    unsigned int lo_inner_iterations;
    unsigned int lo_iterative_iterations;
    unsigned int lo_sample_size;

    UniformRandomGenerator * uniform_random_generator;
public:
    unsigned int lo_inner_iters = 0;
    unsigned int lo_iterative_iters = 0;

    virtual ~RansacLocalOptimization () {
        // std::cout << "cleaning in ransac lo\n";
        delete inliers, lo_inliers, lo_sample, max_lo_inliers, lo_score, lo_model, uniform_random_generator;
    }

    RansacLocalOptimization (Model *model_,
                             Sampler *sampler_,
                             TerminationCriteria *termination_criteria_,
                             Quality *quality_,
                             Estimator *estimator_,
                             unsigned int points_size_) {
        model = model_;
        sampler = sampler_;

        estimator = estimator_;
        termination_criteria = termination_criteria_;
        quality = quality_;
        points_size = points_size_;

        lo_inner_iterations = model->lo_inner_iterations;
        lo_iterative_iterations = model->lo_iterative_iterations;
        lo_sample_size = model->lo_sample_size;

        // Allocate max memory to avoid reallocation
        inliers = new int [points_size];
        lo_inliers = new int [points_size];
        max_lo_inliers = new int [points_size];
        lo_sample = new int [lo_sample_size];

        lo_score = new Score;
        lo_model = new Model;
        lo_model->copyFrom(model);

        uniform_random_generator = new UniformRandomGenerator;

        // comment to debug
        uniform_random_generator->resetTime();
    }
    
    void GetLOModelScore  (Model * best_model,
                           Score * best_score) override {
        // minimum required inliers for LO is 2 * sample_size + 1, see Ransac.run()

        quality->getInliers(best_model->returnDescriptor(), inliers);

        /*
         * Assume best lo score is at least kth ransac score.
         * Avoid comparing between current lo score with best lo score and kth score.
         */
//        lo_score->inlier_number = best_score->inlier_number;
//        lo_score->score = best_score->score;

        /* In our experiments the size of samples are set to min (Ik/2, 14)
         * for epipolar geometry and to min (Ik/2, 12) for the case of homography estimation
         */

        /*
         * reduce multiplier threshold K·θ by this number in each iteration.
         * In the last iteration there be original threshold θ.
         */
        lo_model->threshold = model->threshold;
        float reduce_threshold = (lo_model->lo_threshold * lo_model->lo_threshold_multiplier -
                                  lo_model->threshold)
                                 / lo_model->lo_iterative_iterations;

        unsigned int generate_sample_size = std::min(best_score->inlier_number / 2 + 1, (int) lo_sample_size);

        uniform_random_generator->setSubsetSize(generate_sample_size);
        uniform_random_generator->resetGenerator(0, best_score->inlier_number - 1);

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
            uniform_random_generator->generateUniqueRandomSet(lo_sample);
            if (lo_better_than_kth_ransac) {
                // get inliers from maximum inliers from lo
                for (int smpl = 0; smpl < generate_sample_size; smpl++) {
                    lo_sample[smpl] = max_lo_inliers[lo_sample[smpl]];
//                    std::cout << lo_sample[smpl] << " ";
                }
            } else {
                // get inliers from ransac
                for (int smpl = 0; smpl < generate_sample_size; smpl++) {
                    lo_sample[smpl] = inliers[lo_sample[smpl]];
//                    std::cout << lo_sample[smpl] << " ";
                }
            }

//            std::cout << "\n";

            // multiply threshold K * θ
            lo_model->threshold = lo_model->lo_threshold_multiplier * lo_model->lo_threshold;

//            std::cout << "estimate model from sample\n";
            if (!estimator->LeastSquaresFitting(lo_sample, generate_sample_size, *lo_model)) continue;

//            std::cout << "evaluate model from sample\n";
            // Start evaluating a model with new threshold. And get inliers for iterative lo ransac.
            quality->getNumberInliers(lo_score, lo_model, true, lo_inliers);
//            std::cout << lo_score->inlier_number << " = |I|\n";

            // continue, if score with bigger threshold is less than best score of ransac, it will not be better
            if (best_score->bigger(lo_score)) {
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
//                std::cout << "estimate model in iterative \n";
//                for (int i = 0; i < lo_score->inlier_number; i++) {
//                    std::cout << lo_inliers[i] << " ";
//                }
//                std::cout << "\n";
                if (!estimator->LeastSquaresFitting(lo_inliers, lo_score->inlier_number, *lo_model)) continue;
//                std::cout << "evaluate model in iterative \n";
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
                if (best_score->bigger(lo_score)) {
                    break;
                }
            }
            // get original threshold back in case lo iterative ransac had break.
            lo_model->threshold = model->threshold;

            // update best model
            if (lo_score->bigger(best_score)) {
//                std::cout << "update model\n";
                best_model->setDescriptor(lo_model->returnDescriptor());
                best_score->copyFrom(lo_score);
                lo_better_than_kth_ransac = true;
                //copy inliers to max inliers for sampling
                std::copy(lo_inliers, lo_inliers + lo_score->inlier_number, max_lo_inliers);

                // update generator
                generate_sample_size = std::min(best_score->inlier_number / 2 + 1, (int) lo_sample_size);
                uniform_random_generator->setSubsetSize(generate_sample_size);
                uniform_random_generator->resetGenerator(0, best_score->inlier_number - 1);
            }
            // only for test
            lo_inner_iters++;
            //
        }
    }
};


#endif //USAC_RANSACLOCALOPTIMIZATION_H
